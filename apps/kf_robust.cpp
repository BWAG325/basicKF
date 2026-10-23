#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <variant>
#include <vector>

#include "kf/models/ca.hpp"
#include "kf/models/ct.hpp"
#include "kf/models/cv.hpp"
#include "kf/models/spin.hpp"
#include "kf/models/spin_gen.hpp"
#include "sim/config.hpp"
#include "sim/core/paths.hpp"
#include "sim/sensing/sensor.hpp"
#include "sim/simulator.hpp"

// 观测噪声“估错”时的鲁棒性：真实噪声固定，滤波器假设的 R 乘一个 scale。
const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{config-path c  | configs/kf/robust.yaml | 评估配置}";

namespace {

struct Sample { double t; Eigen::Vector3d truth, obs; };
using FilterVar = std::variant<kf::CvKalman, kf::CaKalman, kf::CtKalman, kf::SpinKalman, kf::SpinGenKalman>;

Eigen::Vector3d posOf(const FilterVar& f) {
    return std::visit([](const auto& x) { return Eigen::Vector3d(x.position()); }, f);
}
void pred(FilterVar& f, double dt) { std::visit([dt](auto& x) { x.predict(dt); }, f); }
void upd(FilterVar& f, const Eigen::Vector3d& z) { std::visit([&z](auto& x) { x.update(z); }, f); }

FilterVar makeFilter(const std::string& type, const Eigen::Vector3d& z, const Eigen::Vector3d& v,
                     const Eigen::Matrix3d& R) {
    if (type == "cv") {
        kf::CvKalman f; f.init(z, v); f.setAccelNoise(10.0); f.setMeasurementNoise(R); return f;
    }
    if (type == "ca") {
        kf::CaKalman f; f.init(z, v, Eigen::Vector3d::Zero()); f.setJerkNoise(50.0);
        f.setMeasurementNoise(R); return f;
    }
    if (type == "ctrv") {
        kf::CtParams p; p.sigma_v = 2.0; p.sigma_w = 2.0; p.R = R;
        kf::CtKalman f(p); f.init(z, v); f.setMeasurementNoise(R); return f;
    }
    if (type == "spin_g") {
        kf::SpinGenParams p; p.sigma_v = 2.0; p.sigma_w = 2.0; p.R = R;
        kf::SpinGenKalman f(p); f.init(z, v); f.setMeasurementNoise(R); return f;
    }
    kf::SpinParams p; p.sigma_v = 2.0; p.sigma_w = 2.0; p.R = R;
    kf::SpinKalman f(p); f.init(z, v); f.setMeasurementNoise(R); return f;
}

}  // namespace

int main(int argc, char* argv[]) {
    cv::CommandLineParser cli(argc, argv, keys);
    if (cli.has("help")) { cli.printMessage(); return 0; }

    const std::string cfg_path = sim::resolvePath(cli.get<std::string>("config-path"));
    try {
        const YAML::Node cfg = YAML::LoadFile(cfg_path);
        const int warmup = cfg["warmup"] ? cfg["warmup"].as<int>() : 25;
        const double sigma_true = cfg["true_sigma"] ? cfg["true_sigma"].as<double>() : 0.02;
        const Eigen::Matrix3d R_true = Eigen::Matrix3d::Identity() * sigma_true * sigma_true;

        std::vector<double> scales;
        for (const auto& s : cfg["r_scales"]) scales.push_back(s.as<double>());
        std::vector<std::string> models;
        for (const auto& m : cfg["models"]) models.push_back(m.as<std::string>());

        std::vector<std::vector<std::pair<double, int>>> acc(
            models.size(), std::vector<std::pair<double, int>>(scales.size(), {0.0, 0}));

        for (const auto& sc : cfg["scenarios"]) {
            const double duration = sc["duration"] ? sc["duration"].as<double>() : 16.0;
            const std::string sim_path = sim::resolvePath(sc["path"].as<std::string>());

            auto setup = sim::loadSimSetup(sim_path);
            sim::Simulator sim(setup.geometry, setup.initial, std::move(setup.controller), setup.min_dt);
            sim::GaussianSensor sensor(setup.sensor);
            int locked = sensor.pickArmor(sim.truth());
            if (locked < 0) locked = 0;

            const double min_interval = [&] {
                if (!setup.sensor.intervals.empty())
                    return *std::min_element(setup.sensor.intervals.begin(), setup.sensor.intervals.end());
                return std::max(1e-3, setup.sensor.interval - setup.sensor.jitter);
            }();
            const std::size_t count = static_cast<std::size_t>(duration / min_interval) + 16;
            auto times = sim::makeObservationTimes(setup.sensor, count);

            std::vector<Sample> ss;
            for (std::size_t k = 0; k < times.size() && times[k] <= duration; ++k) {
                sim.advanceTo(times[k]);
                sim::ObservationTruth anno;
                auto o = sensor.observeArmorAt(sim.truth(), times[k], k, locked, &anno);
                if (o.valid) ss.push_back({times[k], sim.truth().armors[locked].p, o.p});
            }
            if (ss.size() < 3) continue;

            const Eigen::Vector3d v0 = (ss[1].obs - ss[0].obs) / (ss[1].t - ss[0].t);
            for (std::size_t mi = 0; mi < models.size(); ++mi) {
                for (std::size_t si = 0; si < scales.size(); ++si) {
                    const Eigen::Matrix3d R = R_true * scales[si];
                    FilterVar f = makeFilter(models[mi], ss[1].obs, v0, R);
                    double se = 0.0; int n = 0; double last = ss[1].t;
                    for (std::size_t i = 2; i < ss.size(); ++i) {
                        pred(f, std::max(1e-6, ss[i].t - last)); last = ss[i].t;
                        upd(f, ss[i].obs);
                        if (static_cast<int>(i) < warmup + 2) continue;
                        se += (posOf(f) - ss[i].truth).squaredNorm(); ++n;
                    }
                    acc[mi][si].first += se / std::max(1, n);
                    acc[mi][si].second += 1;
                }
            }
        }

        const std::string csv_path = sim::resolvePath(
            cfg["output"] && cfg["output"]["csv"] ? cfg["output"]["csv"].as<std::string>()
                                                  : "build/kf/robust.csv");
        std::filesystem::create_directories(std::filesystem::path(csv_path).parent_path());
        std::ofstream csv(csv_path);
        csv << "model,r_scale,rmse_cm\n";

        std::cout << "\nRMSE [cm] vs assumed R scale (true sigma=" << sigma_true << ")\n";
        std::cout << std::left << std::setw(8) << "model";
        for (double s : scales) std::cout << std::setw(8) << s;
        std::cout << "\n" << std::fixed << std::setprecision(2);
        for (std::size_t mi = 0; mi < models.size(); ++mi) {
            std::cout << std::left << std::setw(8) << models[mi];
            for (std::size_t si = 0; si < scales.size(); ++si) {
                const double mse = acc[mi][si].first / std::max(1, acc[mi][si].second);
                std::cout << std::setw(8) << std::sqrt(mse) * 100.0;
                csv << models[mi] << ',' << scales[si] << ',' << std::sqrt(mse) * 100.0 << '\n';
            }
            std::cout << "\n";
        }
        std::cout << "\ncsv: " << csv_path << "\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
}
