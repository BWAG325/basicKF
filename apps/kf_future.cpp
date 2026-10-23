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

const std::string keys =
  "{help h usage ? |                          | 输出命令行参数说明}"
  "{config-path c  | configs/kf/future.yaml   | 评估配置}";

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

        std::vector<std::string> models;
        for (const auto& m : cfg["models"]) models.push_back(m.as<std::string>());

        const std::string csv_path = sim::resolvePath(
            cfg["output"] && cfg["output"]["csv"] ? cfg["output"]["csv"].as<std::string>()
                                                  : "build/kf/future.csv");
        std::filesystem::create_directories(std::filesystem::path(csv_path).parent_path());
        std::ofstream csv(csv_path);
        csv << "scenario,model,horizon_ms,rmse_cm\n";

        std::vector<std::vector<std::pair<double, int>>> acc(
            models.size(), std::vector<std::pair<double, int>>(horizons.size(), {0.0, 0}));

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
            std::vector<FilterVar> filters;
            for (const auto& m : models) filters.push_back(makeFilter(m, ss[1].obs, v0, R));

            std::vector<std::vector<std::pair<double, int>>> sacc(
                models.size(), std::vector<std::pair<double, int>>(horizons.size(), {0.0, 0}));

            double last_t = ss[1].t;
            for (std::size_t i = 2; i < ss.size(); ++i) {
                const double dt = std::max(1e-6, ss[i].t - last_t);
                last_t = ss[i].t;
                for (auto& f : filters) { pred(f, dt); upd(f, ss[i].obs); }
                if (static_cast<int>(i) < warmup + 2) continue;

                for (std::size_t mi = 0; mi < filters.size(); ++mi) {
                    for (std::size_t hi = 0; hi < horizons.size(); ++hi) {
                        FilterVar copy = filters[mi];
                        double h = horizons[hi];
                        while (h > 1e-9) {
                            const double step = std::min(h, 0.01);
                            pred(copy, step);
                            h -= step;
                        }
                        const Eigen::Vector3d e = posOf(copy) - truthAt(ss, ss[i].t + horizons[hi]);
                        sacc[mi][hi].first += e.squaredNorm();
                        sacc[mi][hi].second += 1;
                    }
                }
            }

            for (std::size_t mi = 0; mi < models.size(); ++mi) {
                for (std::size_t hi = 0; hi < horizons.size(); ++hi) {
                    acc[mi][hi].first += sacc[mi][hi].first;
                    acc[mi][hi].second += sacc[mi][hi].second;
                    const double rmse = std::sqrt(sacc[mi][hi].first / std::max(1, sacc[mi][hi].second));
                    csv << name << ',' << models[mi] << ',' << horizons[hi] * 1000.0 << ','
                        << rmse * 100.0 << '\n';
                }
            }
            std::cout << "done " << name << "\n";
        }

        std::cout << "\nfuture-position RMSE [cm]  (rows=model, cols=horizon, all scenarios)\n";
        std::cout << std::left << std::setw(10) << "model";
        for (double h : horizons)
            std::cout << std::setw(9) << (std::to_string(static_cast<int>(h * 1000)) + "ms");
        std::cout << "\n" << std::fixed << std::setprecision(2);
        for (std::size_t mi = 0; mi < models.size(); ++mi) {
            std::cout << std::left << std::setw(10) << models[mi];
            for (std::size_t hi = 0; hi < horizons.size(); ++hi) {
                const double rmse = std::sqrt(acc[mi][hi].first / std::max(1, acc[mi][hi].second));
                std::cout << std::setw(8) << rmse * 100.0;
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
