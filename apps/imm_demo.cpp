#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "kf/imm.hpp"
#include "kf/models/ca.hpp"
#include "kf/models/cv.hpp"
#include "kf/models/cv9.hpp"
#include "sim/config.hpp"
#include "sim/core/paths.hpp"
#include "sim/sensing/sensor.hpp"
#include "sim/simulator.hpp"

// 折返跑上的滤波器对比：
//   单 CV / 单 CA / IMM(CV+CV) / IMM(CV9+CA) / IMM(CA+CA)
// 其中 CV9 = CV + 恒为 0 的虚拟加速度（9 维，和 CA 对齐）。
const std::string keys =
  "{help h usage ? |                                   | 输出命令行参数说明}"
  "{config-path c  | configs/sim/rm_shuttle.yaml       | yaml配置文件路径}"
  "{duration       | 16.0                              | 运行时长 [s]}"
  "{sigma-slow     | 0.2                               | CV 慢模型：加速度 std [m/s²]}"
  "{sigma-fast     | 20.0                              | CV 快模型：加速度 std [m/s²]}"
  "{sigma-cv       | 0.6                               | 单 CV 对照：加速度 std [m/s²]}"
  "{sigma-jerk     | 50.0                              | CA 快模型 / 单 CA：jerk std [m/s³]}"
  "{sigma-jerk-slow| 0.5                               | CA 慢模型：jerk std [m/s³]}"
  "{pi-slow        | 0.95                              | 慢模型自转移概率（每帧）}"
  "{pi-fast        | 0.95                              | 快模型自转移概率（每帧）}"
  "{warmup         | 25                                | 统计误差前跳过的帧数}"
  "{table t        | false                             | 逐帧打印}"
  "{csv            | false                             | 输出 CSV 到 stdout}";

namespace {

double minInterval(const sim::SensorConfig& cfg) {
    if (!cfg.intervals.empty()) return *std::min_element(cfg.intervals.begin(), cfg.intervals.end());
    return std::max(1e-3, cfg.interval - cfg.jitter);
}

struct Sample {
    double t = 0.0;
    sim::Vec3 truth = sim::Vec3::Zero();
    sim::Vec3 obs = sim::Vec3::Zero();
    sim::Mat3 R = sim::Mat3::Identity();
};

struct Row {
    double t = 0.0;
    sim::Vec3 truth, obs;
    sim::Vec3 cv, ca, cc, ca9, caca;
    double err_cv = 0.0, err_ca = 0.0, err_cc = 0.0, err_ca9 = 0.0, err_caca = 0.0;
    double mu_cc_slow = 0.0, mu_cc_fast = 0.0;
    double mu_ca9_cv = 0.0, mu_ca9_ca = 0.0;
    double mu_caca_slow = 0.0, mu_caca_fast = 0.0;
    double v_truth = 0.0, v_caca = 0.0;
};

double rmse(const std::vector<Row>& rows, int warmup, double Row::*field) {
    double se = 0.0;
    int n = 0;
    for (std::size_t i = 0; i < rows.size(); ++i) {
        if (static_cast<int>(i) < warmup) continue;
        const double e = rows[i].*field;
        se += e * e;
        ++n;
    }
    return std::sqrt(se / std::max(1, n));
}

}  // namespace

int main(int argc, char* argv[]) {
    cv::CommandLineParser cli(argc, argv, keys);
    if (cli.has("help")) {
        cli.printMessage();
        return 0;
    }

    const std::string config_path = sim::resolvePath(cli.get<std::string>("config-path"));
    if (!std::filesystem::exists(config_path)) {
        std::cerr << "error: 找不到配置文件: " << config_path << "\n";
        return 1;
    }

    const double duration = cli.get<double>("duration");
    const double sigma_slow = cli.get<double>("sigma-slow");
    const double sigma_fast = cli.get<double>("sigma-fast");
    const double sigma_cv = cli.get<double>("sigma-cv");
    const double sigma_jerk = cli.get<double>("sigma-jerk");
    const double sigma_jerk_slow = cli.get<double>("sigma-jerk-slow");
    const double pi_slow = cli.get<double>("pi-slow");
    const double pi_fast = cli.get<double>("pi-fast");
    const int warmup = std::max(0, cli.get<int>("warmup"));
    const bool print_table = cli.get<bool>("table");
    const bool print_csv = cli.get<bool>("csv");

    try {
        auto setup = sim::loadSimSetup(config_path);
        sim::Simulator sim(setup.geometry, setup.initial, std::move(setup.controller), setup.min_dt);
        const sim::GaussianSensor sensor(setup.sensor);

        const std::size_t frame_count =
            static_cast<std::size_t>(duration / minInterval(setup.sensor)) + 16;
        const std::vector<double> times = sim::makeObservationTimes(setup.sensor, frame_count);

        // ---- 收集真值 / 观测 ----
        std::vector<Sample> samples;
        samples.reserve(times.size());

        for (std::size_t k = 0; k < times.size(); ++k) {
            if (times[k] > duration + 1e-9) break;
            sim.advanceTo(times[k]);
            const sim::RobotTruth truth = sim.truth();

            sim::ObservationTruth anno;
            const sim::Observation obs = sensor.observeAt(truth, times[k], k, &anno);
            if (!obs.valid) continue;

            const sim::Vec3 plate_truth = truth.armors[static_cast<std::size_t>(anno.armor_index)].p;
            samples.push_back(Sample{times[k], plate_truth, obs.p, obs.R_pos});
        }

        if (samples.size() < 3) {
            std::cerr << "error: 有效观测太少（" << samples.size() << "），检查配置文件\n";
            return 1;
        }

        const double dt0 = std::max(1e-6, samples[1].t - samples[0].t);
        const sim::Vec3 v0 = (samples[1].obs - samples[0].obs) / dt0;

        // ---- 单模型 ----
        kf::CvKalman cv;
        cv.init(samples[1].obs, v0, 0.05, 1.5);
        cv.setAccelNoise(sigma_cv);
        cv.setMeasurementNoise(samples[1].R);

        kf::CaKalman ca;
        ca.init(samples[1].obs, v0, sim::Vec3::Zero(), 0.05, 1.5, 2.0);
        ca.setJerkNoise(sigma_jerk);
        ca.setMeasurementNoise(samples[1].R);

        // ---- IMM 的公共同步转移矩阵 ----
        Eigen::Matrix2d Pi;
        Pi << pi_slow, 1.0 - pi_slow,
              1.0 - pi_fast, pi_fast;
        const Eigen::Vector2d mu0(0.5, 0.5);

        auto make_cv = [&](double sigma) {
            auto f = std::make_unique<kf::CvKalman>();
            f->init(samples[1].obs, v0, 0.05, 1.5);
            f->setAccelNoise(sigma);
            f->setMeasurementNoise(samples[1].R);
            return f;
        };
        auto make_cv9 = [&](double sigma) {
            auto f = std::make_unique<kf::Cv9Kalman>();
            f->init(samples[1].obs, v0, 0.05, 1.5);
            f->setAccelNoise(sigma);
            f->setMeasurementNoise(samples[1].R);
            return f;
        };
        auto make_ca = [&](double jerk) {
            auto f = std::make_unique<kf::CaKalman>();
            f->init(samples[1].obs, v0, sim::Vec3::Zero(), 0.05, 1.5, 2.0);
            f->setJerkNoise(jerk);
            f->setMeasurementNoise(samples[1].R);
            return f;
        };

        std::vector<kf::Imm::Model> models_cc;
        models_cc.push_back({"cv-slow", make_cv(sigma_slow)});
        models_cc.push_back({"cv-fast", make_cv(sigma_fast)});
        kf::Imm imm_cc(std::move(models_cc), Pi, mu0);

        std::vector<kf::Imm::Model> models_ca9;
        models_ca9.push_back({"cv9-slow", make_cv9(sigma_slow)});
        models_ca9.push_back({"ca", make_ca(sigma_jerk)});
        kf::Imm imm_ca9(std::move(models_ca9), Pi, mu0);

        std::vector<kf::Imm::Model> models_caca;
        models_caca.push_back({"ca-slow", make_ca(sigma_jerk_slow)});
        models_caca.push_back({"ca-fast", make_ca(sigma_jerk)});
        kf::Imm imm_caca(std::move(models_caca), Pi, mu0);

        // ---- 同一批观测 ----
        std::vector<Row> rows;
        rows.reserve(samples.size());

        double last_t = samples[1].t;
        for (std::size_t i = 2; i < samples.size(); ++i) {
            const double dt = std::max(1e-6, samples[i].t - last_t);
            last_t = samples[i].t;

            cv.predict(dt);
            cv.update(samples[i].obs);

            ca.predict(dt);
            ca.update(samples[i].obs);

            imm_cc.predict(dt);
            imm_cc.update(samples[i].obs);

            imm_ca9.predict(dt);
            imm_ca9.update(samples[i].obs);

            imm_caca.predict(dt);
            imm_caca.update(samples[i].obs);

            Row r;
            r.t = samples[i].t;
            r.truth = samples[i].truth;
            r.obs = samples[i].obs;
            r.cv = cv.state().head<3>();
            r.ca = ca.state().head<3>();
            r.cc = imm_cc.state().head<3>();
            r.ca9 = imm_ca9.state().head<3>();
            r.caca = imm_caca.state().head<3>();

            r.err_cv = (r.cv - r.truth).norm();
            r.err_ca = (r.ca - r.truth).norm();
            r.err_cc = (r.cc - r.truth).norm();
            r.err_ca9 = (r.ca9 - r.truth).norm();
            r.err_caca = (r.caca - r.truth).norm();

            r.mu_cc_slow = imm_cc.probabilities()[0];
            r.mu_cc_fast = imm_cc.probabilities()[1];
            r.mu_ca9_cv = imm_ca9.probabilities()[0];
            r.mu_ca9_ca = imm_ca9.probabilities()[1];
            r.mu_caca_slow = imm_caca.probabilities()[0];
            r.mu_caca_fast = imm_caca.probabilities()[1];

            r.v_caca = imm_caca.state().segment<3>(3).norm();
            rows.push_back(r);
        }

        for (std::size_t i = 0; i < rows.size(); ++i) {
            const std::size_t a = i == 0 ? 0 : i - 1;
            if (rows[i].t > rows[a].t) {
                rows[i].v_truth = (rows[i].truth - rows[a].truth).norm() / (rows[i].t - rows[a].t);
            }
        }

        const double e_cv = rmse(rows, warmup, &Row::err_cv);
        const double e_ca = rmse(rows, warmup, &Row::err_ca);
        const double e_cc = rmse(rows, warmup, &Row::err_cc);
        const double e_ca9 = rmse(rows, warmup, &Row::err_ca9);
        const double e_caca = rmse(rows, warmup, &Row::err_caca);

        if (print_csv) {
            std::cout << "t,truth_x,truth_y,truth_z,obs_x,obs_y,obs_z,"
                         "cv_x,cv_y,cv_z,ca_x,ca_y,ca_z,cc_x,cc_y,cc_z,ca9_x,ca9_y,ca9_z,caca_x,caca_y,caca_z,"
                         "err_cv,err_ca,err_cc,err_ca9,err_caca,"
                         "mu_cc_slow,mu_cc_fast,mu_ca9_cv,mu_ca9_ca,mu_caca_slow,mu_caca_fast,"
                         "v_truth,v_caca\n";
        }

        for (const Row& r : rows) {
            if (print_table && !print_csv) {
                std::cout << std::fixed << std::setprecision(2) << std::setw(6) << r.t
                          << "  cv=" << std::setw(5) << r.err_cv * 100.0
                          << "  ca=" << std::setw(5) << r.err_ca * 100.0
                          << "  cc=" << std::setw(5) << r.err_cc * 100.0
                          << "  c9+ca=" << std::setw(5) << r.err_ca9 * 100.0
                          << "  ca+ca=" << std::setw(5) << r.err_caca * 100.0 << '\n';
            }
            if (print_csv) {
                std::cout << std::fixed << std::setprecision(6) << r.t << ','
                          << r.truth.x() << ',' << r.truth.y() << ',' << r.truth.z() << ','
                          << r.obs.x() << ',' << r.obs.y() << ',' << r.obs.z() << ','
                          << r.cv.x() << ',' << r.cv.y() << ',' << r.cv.z() << ','
                          << r.ca.x() << ',' << r.ca.y() << ',' << r.ca.z() << ','
                          << r.cc.x() << ',' << r.cc.y() << ',' << r.cc.z() << ','
                          << r.ca9.x() << ',' << r.ca9.y() << ',' << r.ca9.z() << ','
                          << r.caca.x() << ',' << r.caca.y() << ',' << r.caca.z() << ','
                          << r.err_cv << ',' << r.err_ca << ',' << r.err_cc << ','
                          << r.err_ca9 << ',' << r.err_caca << ','
                          << r.mu_cc_slow << ',' << r.mu_cc_fast << ','
                          << r.mu_ca9_cv << ',' << r.mu_ca9_ca << ','
                          << r.mu_caca_slow << ',' << r.mu_caca_fast << ','
                          << r.v_truth << ',' << r.v_caca << '\n';
            }
        }

        if (!print_csv) {
            std::cout << "\nconfig   : " << config_path << "\n"
                      << "duration : " << duration << " s   valid: " << samples.size()
                      << "   filtered: " << rows.size() << "\n\n"
                      << std::fixed << std::setprecision(2)
                      << "单 CV          (sigma=" << sigma_cv << ")        RMSE " << e_cv * 100.0 << " cm\n"
                      << "单 CA          (jerk=" << sigma_jerk << ")        RMSE " << e_ca * 100.0 << " cm\n"
                      << "IMM CV+CV      (" << sigma_slow << " / " << sigma_fast << ")     RMSE " << e_cc * 100.0 << " cm\n"
                      << "IMM CV9+CA     (" << sigma_slow << " / " << sigma_jerk << ")     RMSE " << e_ca9 * 100.0 << " cm\n"
                      << "IMM CA+CA      (" << sigma_jerk_slow << " / " << sigma_jerk << ")     RMSE " << e_caca * 100.0 << " cm\n";
        }
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
}
