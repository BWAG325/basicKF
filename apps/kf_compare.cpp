#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "kf/imm.hpp"
#include "kf/fusion.hpp"
#include "kf/models/cv.hpp"
#include "kf/models/ca.hpp"
#include "kf/cv_imm.hpp"
#include "kf/models/plate_cv.hpp"
#include "sim/config.hpp"
#include "sim/core/paths.hpp"
#include "sim/sensing/sensor.hpp"
#include "sim/simulator.hpp"

// 配置驱动的滤波对比：
//   python3 tools/compare.py configs/kf/shuttle_spin.yaml
// 配置文件决定 sim 场景、时长、以及要跑哪些模型（CV / CV9 / CA / IMM）。
const std::string keys =
  "{help h usage ? |                                   | 输出命令行参数说明}"
  "{config-path c  | configs/kf/shuttle_spin.yaml      | 对比配置 yaml}";

namespace {

int g_armor_id = 0;
int g_armor_num = 4;

struct Spec {
    std::string name;
    std::string type;
    double q_xyz = 100.0;
    double q_yaw = 75.0;
    double q_xyz2 = 400.0;
    double q_yaw2 = 300.0;
    double q_jerk = 200.0;
    double ca_tau = 0.0;
    double p0_accel_var = 25.0;
    std::vector<double> mu0;
    double plate_v = 2.0;
    double plate_w = 2.0;
    double armor_radius = 0.22;
    std::vector<double> pi;
    std::vector<Spec> subs;
};

Spec parseSpec(const YAML::Node& n) {
    if (!n["type"]) throw std::runtime_error("模型缺少 type");
    Spec s;
    s.type = n["type"].as<std::string>();
    s.name = n["name"] ? n["name"].as<std::string>() : s.type;
    if (n["q_xyz"]) s.q_xyz = n["q_xyz"].as<double>();
    if (n["q_yaw"]) s.q_yaw = n["q_yaw"].as<double>();
    if (n["q_xyz2"]) s.q_xyz2 = n["q_xyz2"].as<double>();
    if (n["q_yaw2"]) s.q_yaw2 = n["q_yaw2"].as<double>();
    if (n["q_jerk"]) s.q_jerk = n["q_jerk"].as<double>();
    if (n["ca_tau"]) s.ca_tau = n["ca_tau"].as<double>();
    if (n["p0_accel_var"]) s.p0_accel_var = n["p0_accel_var"].as<double>();
    if (n["mu0"]) {
        for (const auto& v : n["mu0"]) s.mu0.push_back(v.as<double>());
    }
    if (n["plate_v"]) s.plate_v = n["plate_v"].as<double>();
    if (n["plate_w"]) s.plate_w = n["plate_w"].as<double>();
    if (n["pi"]) {
        for (const auto& v : n["pi"]) s.pi.push_back(v.as<double>());
    }
    if (n["models"]) {
        for (const auto& m : n["models"]) s.subs.push_back(parseSpec(m));
    }
    if (s.type == "imm" && s.subs.size() < 2) throw std::runtime_error("IMM 至少需要两个子模型");
    return s;
}

struct Sample {
    double t = 0.0;
    Eigen::Vector3d truth = Eigen::Vector3d::Zero();
    Eigen::Vector3d obs = Eigen::Vector3d::Zero();
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
};

struct Runner {
    std::string name;
    bool is_imm = false;
    std::unique_ptr<kf::IFilter> single;
    std::unique_ptr<kf::Imm> imm;
    std::vector<std::string> subs;
};

std::unique_ptr<kf::IFilter> makeLeaf(const Spec& s, const Eigen::Vector3d& z,
                                      const Eigen::Vector3d& v, const Eigen::Matrix3d& R) {
    if (s.type == "plate") {
        kf::PlateCvParams p; p.sigma_v = s.plate_v; p.sigma_w = s.plate_w; p.R = R;
        auto f = std::make_unique<kf::PlateCvKalman>(p);
        f->init(z, v);
        f->setMeasurementNoise(R);
        return f;
    }
    if (s.type == "ca") {
        kf::CaParams p;
        p.q_jerk = s.q_jerk;
        p.q_yaw = s.q_yaw;
        p.ca_tau = s.ca_tau;
        p.p0_accel_var = s.p0_accel_var;
        p.armor_radius = s.armor_radius;
        p.armor_id = g_armor_id;
        p.R = R;
        auto f = std::make_unique<kf::CaKalman>(p);
        f->init(z, v);
        f->setMeasurementNoise(R);
        return f;
    }
    if (s.type == "cv_imm") {
        kf::CvParams a; a.q_xyz = s.q_xyz; a.q_yaw = s.q_yaw; a.armor_radius = s.armor_radius;
        a.armor_id = g_armor_id; a.R = R;
        kf::CvParams b = a; b.q_xyz = s.q_xyz2; b.q_yaw = s.q_yaw2;
        Eigen::Matrix2d Pi; Pi << 0.95, 0.05, 0.05, 0.95;
        auto f = std::make_unique<kf::CvImm>(a, b, Pi, Eigen::Vector2d(0.5, 0.5));
        f->init(z, v);
        f->setMeasurementNoise(R);
        return f;
    }
    if (s.type == "fusion") {
        std::vector<std::unique_ptr<kf::IFilter>> models;
        for (auto q : {std::pair<double,double>{s.q_xyz, s.q_yaw}, {s.q_xyz2, s.q_yaw2}}) {
            kf::CvParams p; p.q_xyz = q.first; p.q_yaw = q.second; p.armor_radius = s.armor_radius;
            p.armor_id = g_armor_id; p.R = R;
            auto f = std::make_unique<kf::CvKalman>(p);
            f->init(z, v);
            f->setMeasurementNoise(R);
            models.push_back(std::move(f));
        }
        Eigen::Matrix2d Pi; Pi << 0.95, 0.05, 0.05, 0.95;
        return std::make_unique<kf::OutputFusion>(std::move(models), Pi, Eigen::Vector2d(0.5, 0.5));
    }
    if (s.type != "cv") throw std::runtime_error("未知模型类型: " + s.type);
    kf::CvParams p;
    p.q_xyz = s.q_xyz;
    p.q_yaw = s.q_yaw;
    p.armor_radius = s.armor_radius;
    p.armor_id = g_armor_id;
    p.R = R;
    auto f = std::make_unique<kf::CvKalman>(p);
    f->init(z, v);
    f->setMeasurementNoise(R);
    return f;
}

Runner makeRunner(const Spec& s, const Eigen::Vector3d& z, const Eigen::Vector3d& v,
                  const Eigen::Matrix3d& R) {
    Runner r;
    r.name = s.name;

    if (s.type != "imm") {
        r.single = makeLeaf(s, z, v, R);
        return r;
    }

    std::vector<kf::Imm::Model> models;
    for (const Spec& sub : s.subs) {
        r.subs.push_back(sub.name);
        models.push_back({sub.name, makeLeaf(sub, z, v, R)});
    }

    const int n = static_cast<int>(models.size());
    Eigen::MatrixXd Pi(n, n);
    for (int i = 0; i < n; ++i) {
        const double p = (static_cast<std::size_t>(i) < s.pi.size()) ? s.pi[static_cast<std::size_t>(i)] : 0.95;
        for (int j = 0; j < n; ++j) {
            Pi(i, j) = (i == j) ? p : (1.0 - p) / static_cast<double>(n - 1);
        }
    }
    Eigen::VectorXd mu = Eigen::VectorXd::Constant(n, 1.0 / n);
    if (static_cast<int>(s.mu0.size()) == n) {
        for (int i = 0; i < n; ++i) mu[i] = s.mu0[static_cast<std::size_t>(i)];
        if (mu.sum() > 0.0) mu /= mu.sum();
    }

    // 整车 CV/CA 的 yaw 在下标 6。板模型不要传这个下标。
    r.imm = std::make_unique<kf::Imm>(std::move(models), Pi, mu, std::vector<int>{6});
    r.is_imm = true;
    return r;
}

Eigen::Vector3d runnerPosition(const Runner& r) {
    if (r.is_imm) {
        // IMM 的“状态平均”在非线性板几何上会出问题：CV/CA 可能用不同的
        // center/yaw/r 组合表示同一块板，平均 state 后板位置会被拉偏。
        // 评估时按模型概率对各自的板位置加权，而不是对 state 加权。
        const Eigen::VectorXd mu = r.imm->probabilities();
        Eigen::Vector3d p = Eigen::Vector3d::Zero();
        for (int i = 0; i < r.imm->modelCount(); ++i) {
            p += mu[i] * r.imm->model(i).filter->position();
        }
        return p;
    }
    return r.single->position();
}

std::string jsonEscape(const std::string& s) {
    std::string out;
    for (char c : s) {
        if (c == '"' || c == '\\') out.push_back('\\');
        out.push_back(c);
    }
    return out;
}

}  // namespace

int main(int argc, char* argv[]) {
    cv::CommandLineParser cli(argc, argv, keys);
    if (cli.has("help")) {
        cli.printMessage();
        return 0;
    }

    const std::string config_path = sim::resolvePath(cli.get<std::string>("config-path"));
    try {
        const YAML::Node cfg = YAML::LoadFile(config_path);

        const std::string sim_path = sim::resolvePath(cfg["sim"].as<std::string>());
        const double duration = cfg["duration"] ? cfg["duration"].as<double>() : 16.0;
        const int warmup = cfg["warmup"] ? cfg["warmup"].as<int>() : 25;
        // 观测噪声：配置里给了就用真实值 σz，否则退回观测自带的 R
        const double meas_sigma = cfg["measurement_sigma"] ? cfg["measurement_sigma"].as<double>() : -1.0;
        const std::string csv_path = sim::resolvePath(
            cfg["output"] && cfg["output"]["csv"] ? cfg["output"]["csv"].as<std::string>()
                                                  : "build/kf/compare.csv");
        const std::string png_path = sim::resolvePath(
            cfg["output"] && cfg["output"]["png"] ? cfg["output"]["png"].as<std::string>()
                                                  : "build/kf/compare.png");
        const std::string meta_path = csv_path + ".json";

        std::vector<Spec> specs;
        for (const auto& m : cfg["models"]) specs.push_back(parseSpec(m));
        if (specs.empty()) throw std::runtime_error("models 为空");

        // ---- 仿真 + 观测（锁定第一帧最正对相机的那块板）----
        auto setup = sim::loadSimSetup(sim_path);
        if (cfg["sensor_sigma"]) {
            setup.sensor.position_sigma = Eigen::Vector3d::Constant(cfg["sensor_sigma"].as<double>());
        }
        sim::Simulator sim(setup.geometry, setup.initial, std::move(setup.controller), setup.min_dt);
        const sim::GaussianSensor sensor(setup.sensor);

        int locked = sensor.pickArmor(sim.truth());
        if (locked < 0) locked = 0;
        g_armor_id = locked;
        g_armor_num = setup.geometry.armor_count;

        const double min_interval = [&] {
            if (!setup.sensor.intervals.empty()) {
                return *std::min_element(setup.sensor.intervals.begin(), setup.sensor.intervals.end());
            }
            return std::max(1e-3, setup.sensor.interval - setup.sensor.jitter);
        }();
        const std::size_t frame_count =
            static_cast<std::size_t>(duration / min_interval) + 16;
        const std::vector<double> times = sim::makeObservationTimes(setup.sensor, frame_count);

        std::vector<Sample> samples;
        samples.reserve(times.size());

        for (std::size_t k = 0; k < times.size(); ++k) {
            if (times[k] > duration + 1e-9) break;
            sim.advanceTo(times[k]);
            const sim::RobotTruth truth = sim.truth();

            sim::ObservationTruth anno;
            const sim::Observation obs = sensor.observeArmorAt(truth, times[k], k, locked, &anno);
            if (!obs.valid) continue;

            samples.push_back(Sample{times[k], truth.armors[static_cast<std::size_t>(locked)].p,
                                     obs.p, obs.R_pos});
        }

        if (samples.size() < 3) throw std::runtime_error("有效观测太少");

        const double dt0 = std::max(1e-6, samples[1].t - samples[0].t);
        const Eigen::Vector3d v0 = (samples[1].obs - samples[0].obs) / dt0;

        Eigen::Matrix3d R = samples[1].R;
        if (meas_sigma > 0.0) R = Eigen::Matrix3d::Identity() * meas_sigma * meas_sigma;

        std::vector<Runner> runners;
        for (const Spec& s : specs) runners.push_back(makeRunner(s, samples[1].obs, v0, R));

        // ---- 跑同一批观测 ----
        struct Row {
            double t = 0.0;
            Eigen::Vector3d truth, obs;
            std::vector<Eigen::Vector3d> est;
            std::vector<double> err;
            std::vector<std::vector<double>> mu;
        };
        std::vector<Row> rows;
        rows.reserve(samples.size());

        double last_t = samples[1].t;
        for (std::size_t i = 2; i < samples.size(); ++i) {
            const double dt = std::max(1e-6, samples[i].t - last_t);
            last_t = samples[i].t;

            Row r;
            r.t = samples[i].t;
            r.truth = samples[i].truth;
            r.obs = samples[i].obs;

            for (Runner& run : runners) {
                if (run.is_imm) {
                    run.imm->predict(dt);
                    run.imm->update(samples[i].obs);
                } else {
                    run.single->predict(dt);
                    run.single->update(samples[i].obs);
                }

                r.est.push_back(runnerPosition(run));
                r.err.push_back((r.est.back() - r.truth).norm());
                if (run.is_imm) {
                    std::vector<double> mu;
                    const Eigen::VectorXd& p = run.imm->probabilities();
                    for (int j = 0; j < p.size(); ++j) mu.push_back(p[j]);
                    r.mu.push_back(std::move(mu));
                } else {
                    r.mu.emplace_back();
                }
            }
            rows.push_back(std::move(r));
        }

        // ---- RMSE ----
        std::vector<double> rmse(runners.size(), 0.0);
        int counted = 0;
        for (std::size_t i = 0; i < rows.size(); ++i) {
            if (static_cast<int>(i) < warmup) continue;
            ++counted;
            for (std::size_t m = 0; m < runners.size(); ++m) rmse[m] += rows[i].err[m] * rows[i].err[m];
        }
        for (double& v : rmse) v = std::sqrt(v / std::max(1, counted)) * 100.0;

        // ---- 写 CSV ----
        std::filesystem::create_directories(std::filesystem::path(csv_path).parent_path());
        std::ofstream csv(csv_path);
        if (!csv) throw std::runtime_error("无法写 " + csv_path);

        csv << "t,truth_x,truth_y,truth_z,obs_x,obs_y,obs_z";
        for (std::size_t m = 0; m < runners.size(); ++m) {
            csv << ",r" << m << "_x,r" << m << "_y,r" << m << "_z,"
                << "err_" << m;
        }
        for (std::size_t m = 0; m < runners.size(); ++m) {
            for (std::size_t j = 0; j < runners[m].subs.size(); ++j) {
                csv << ",mu_" << m << "_" << j;
            }
        }
        csv << '\n';

        for (const Row& r : rows) {
            csv << std::fixed << std::setprecision(6)
                << r.t << ',' << r.truth.x() << ',' << r.truth.y() << ',' << r.truth.z() << ','
                << r.obs.x() << ',' << r.obs.y() << ',' << r.obs.z();
            for (std::size_t m = 0; m < runners.size(); ++m) {
                csv << ',' << r.est[m].x() << ',' << r.est[m].y() << ',' << r.est[m].z()
                    << ',' << r.err[m];
            }
            for (std::size_t m = 0; m < runners.size(); ++m) {
                for (double p : r.mu[m]) csv << ',' << p;
            }
            csv << '\n';
        }

        // ---- 写 meta JSON（给画图脚本）----
        std::ofstream meta(meta_path);
        if (!meta) throw std::runtime_error("无法写 " + meta_path);
        meta << "{\n"
             << "  \"config\": \"" << jsonEscape(config_path) << "\",\n"
             << "  \"sim\": \"" << jsonEscape(sim_path) << "\",\n"
             << "  \"duration\": " << duration << ",\n"
             << "  \"warmup\": " << warmup << ",\n"
             << "  \"csv\": \"" << jsonEscape(csv_path) << "\",\n"
             << "  \"png\": \"" << jsonEscape(png_path) << "\",\n"
             << "  \"models\": [\n";
        for (std::size_t m = 0; m < runners.size(); ++m) {
            const Runner& r = runners[m];
            meta << "    {\"name\": \"" << jsonEscape(r.name) << "\", \"imm\": "
                 << (r.is_imm ? "true" : "false") << ", \"subs\": [";
            for (std::size_t j = 0; j < r.subs.size(); ++j) {
                if (j) meta << ", ";
                meta << "\"" << jsonEscape(r.subs[j]) << "\"";
            }
            meta << "]}" << (m + 1 == runners.size() ? "\n" : ",\n");
        }
        meta << "  ]\n}\n";

        std::cout << std::fixed << std::setprecision(2);
        for (std::size_t m = 0; m < runners.size(); ++m) {
            std::cout << std::left << std::setw(18) << runners[m].name
                      << " RMSE " << std::setw(6) << rmse[m] << " cm";
            if (runners[m].is_imm) {
                std::cout << "   (";
                for (std::size_t j = 0; j < runners[m].subs.size(); ++j) {
                    if (j) std::cout << " / ";
                    std::cout << runners[m].subs[j];
                }
                std::cout << ')';
            }
            std::cout << '\n';
        }
        std::cout << "\nsim   : " << sim_path << "\ncsv   : " << csv_path
                  << "\nmeta: " << meta_path << "\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
}
