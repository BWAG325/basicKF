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
#include <vector>

#include "kf/models/cv.hpp"
#include "sim/config.hpp"
#include "sim/core/paths.hpp"
#include "sim/sensing/sensor.hpp"
#include "sim/simulator.hpp"

const std::string keys =
  "{help h usage ? |                          | 输出命令行参数说明}"
  "{config-path c  | configs/kf/future.yaml   | 评估配置}";

namespace {
struct Sample { double t; Eigen::Vector3d truth, obs; };

kf::CvKalman makeFilter(const Eigen::Vector3d& z, const Eigen::Vector3d& v,
                        const Eigen::Matrix3d& R) {
    kf::CvParams p; p.q_xyz = 100.0; p.q_yaw = 75.0; p.R = R;
    kf::CvKalman f(p); f.init(z, v); f.setMeasurementNoise(R); return f;
}

Eigen::Vector3d truthAt(const std::vector<Sample>& s, double t) {
    if (t <= s.front().t) return s.front().truth;
    if (t >= s.back().t) return s.back().truth;
    const auto it = std::lower_bound(s.begin(), s.end(), t,
                                     [](const Sample& a, double x) { return a.t < x; });
    const std::size_t j = static_cast<std::size_t>(it - s.begin());
    const Sample& b = s[j];
    const Sample& a = s[j - 1];
    const double w = (t - a.t) / std::max(1e-9, b.t - a.t);
    return a.truth + w * (b.truth - a.truth);
}
}  // namespace

int main(int argc, char* argv[]) {
    cv::CommandLineParser cli(argc, argv, keys);
    if (cli.has("help")) { cli.printMessage(); return 0; }

    const std::string cfg_path = sim::resolvePath(cli.get<std::string>("config-path"));
    try {
        const YAML::Node cfg = YAML::LoadFile(cfg_path);
        const int warmup = cfg["warmup"] ? cfg["warmup"].as<int>() : 25;
        const double sigma = cfg["measurement_sigma"] ? cfg["measurement_sigma"].as<double>() : 0.02;
        const Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * sigma * sigma;

        std::vector<double> horizons;
        for (const auto& h : cfg["horizons"]) horizons.push_back(h.as<double>());
        if (horizons.empty()) horizons = {0.0, 0.04, 0.08, 0.12, 0.16};

        const std::string csv_path = sim::resolvePath(
            cfg["output"] && cfg["output"]["csv"] ? cfg["output"]["csv"].as<std::string>()
                                                  : "build/kf/future.csv");
        std::filesystem::create_directories(std::filesystem::path(csv_path).parent_path());
        std::ofstream csv(csv_path);
        csv << "scenario,model,horizon_ms,rmse_cm\n";

        std::vector<std::pair<double, int>> acc(horizons.size(), {0.0, 0});

        for (const auto& sc : cfg["scenarios"]) {
            const std::string name = sc["name"] ? sc["name"].as<std::string>() : "scenario";
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
            const auto times = sim::makeObservationTimes(setup.sensor, count);

            std::vector<Sample> ss;
            for (std::size_t k = 0; k < times.size() && times[k] <= duration; ++k) {
                sim.advanceTo(times[k]);
                sim::ObservationTruth anno;
                auto o = sensor.observeArmorAt(sim.truth(), times[k], k, locked, &anno);
                if (o.valid) ss.push_back({times[k], sim.truth().armors[locked].p, o.p});
            }
            if (ss.size() < 3) continue;

            const Eigen::Vector3d v0 = (ss[1].obs - ss[0].obs) / (ss[1].t - ss[0].t);
            kf::CvKalman filter = makeFilter(ss[1].obs, v0, R);
            std::vector<std::pair<double, int>> sacc(horizons.size(), {0.0, 0});

            double last_t = ss[1].t;
            for (std::size_t i = 2; i < ss.size(); ++i) {
                const double dt = std::max(1e-6, ss[i].t - last_t);
                last_t = ss[i].t;
                filter.predict(dt);
                filter.update(ss[i].obs);
                if (static_cast<int>(i) < warmup + 2) continue;

                for (std::size_t hi = 0; hi < horizons.size(); ++hi) {
                    kf::CvKalman copy = filter;
                    double h = horizons[hi];
                    while (h > 1e-9) {
                        const double step = std::min(h, 0.01);
                        copy.predict(step);
                        h -= step;
                    }
                    const Eigen::Vector3d e = copy.position() - truthAt(ss, ss[i].t + horizons[hi]);
                    sacc[hi].first += e.squaredNorm();
                    sacc[hi].second += 1;
                }
            }

            for (std::size_t hi = 0; hi < horizons.size(); ++hi) {
                acc[hi].first += sacc[hi].first;
                acc[hi].second += sacc[hi].second;
                const double rmse = std::sqrt(sacc[hi].first / std::max(1, sacc[hi].second));
                csv << name << ",CV," << horizons[hi] * 1000.0 << ',' << rmse * 100.0 << '\n';
            }
            std::cout << "done " << name << "\n";
        }

        std::cout << "\nfuture-position RMSE [cm] (CV, all scenarios)\n";
        std::cout << std::left << std::setw(8) << "model";
        for (double h : horizons)
            std::cout << std::setw(9) << (std::to_string(static_cast<int>(h * 1000)) + "ms");
        std::cout << "\nCV      " << std::fixed << std::setprecision(2);
        for (const auto& a : acc) {
            const double rmse = std::sqrt(a.first / std::max(1, a.second));
            std::cout << std::setw(9) << rmse * 100.0;
        }
        std::cout << "\ncsv: " << csv_path << "\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
}
