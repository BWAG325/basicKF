#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "kf/cv_imm.hpp"
#include "kf/fusion.hpp"
#include "kf/imm.hpp"
#include "kf/models/cv.hpp"
#include "kf/models/plate_cv.hpp"
#include "sim/config.hpp"
#include "sim/core/paths.hpp"
#include "sim/sensing/sensor.hpp"
#include "sim/simulator.hpp"

namespace {
int g_id = 0;

class ImmAdapter final : public kf::IFilter {
public:
    explicit ImmAdapter(kf::Imm&& imm) : imm_(std::move(imm)) {}
    void predict(double dt) override { imm_.predict(dt); }
    void update(const Eigen::VectorXd& z) override { imm_.update(z); }
    [[nodiscard]] Eigen::Vector3d position() const override { return imm_.state().head<3>(); }
    [[nodiscard]] Eigen::VectorXd state() const override { return imm_.state(); }
    [[nodiscard]] Eigen::MatrixXd covariance() const override { return imm_.covariance(); }
    [[nodiscard]] Eigen::VectorXd innovation() const override { return imm_.model(0).filter->innovation(); }
    [[nodiscard]] Eigen::MatrixXd innovationCov() const override { return imm_.model(0).filter->innovationCov(); }
    [[nodiscard]] double nis() const override { return imm_.model(0).filter->nis(); }
    [[nodiscard]] double logLikelihood() const override { return imm_.model(0).filter->logLikelihood(); }
    [[nodiscard]] int stateDim() const override { return imm_.commonDim(); }
    [[nodiscard]] int measDim() const override { return 3; }
    void setState(const Eigen::VectorXd&, const Eigen::MatrixXd&) override {}
private:
    kf::Imm imm_;
};

struct Sample { double t; Eigen::Vector3d truth, obs; };

std::unique_ptr<kf::IFilter> build(const std::string& type, const Eigen::Vector3d& z,
                                   const Eigen::Vector3d& v, const Eigen::Matrix3d& R,
                                   double qscale) {
    auto cvParams = [&](double qx, double qy) {
        kf::CvParams p;
        p.q_xyz = qx * qscale;
        p.q_yaw = qy * qscale;
        p.armor_radius = 0.22;
        p.armor_id = g_id;
        p.R = R;
        return p;
    };
    if (type == "cv") {
        auto f = std::make_unique<kf::CvKalman>(cvParams(100, 75));
        f->init(z, v);
        f->setMeasurementNoise(R);
        return f;
    }
    if (type == "cv_fusion") {
        std::vector<std::unique_ptr<kf::IFilter>> ms;
        for (auto q : {std::pair<double,double>{100,75}, {100,100}}) {
            auto f = std::make_unique<kf::CvKalman>(cvParams(q.first, q.second));
            f->init(z, v);
            f->setMeasurementNoise(R);
            ms.push_back(std::move(f));
        }
        Eigen::Matrix2d Pi; Pi << 0.95, 0.05, 0.05, 0.95;
        return std::make_unique<kf::OutputFusion>(std::move(ms), Pi, Eigen::Vector2d(0.5, 0.5));
    }
    if (type == "cv_imm") {
        Eigen::Matrix2d Pi; Pi << 0.95, 0.05, 0.05, 0.95;
        auto f = std::make_unique<kf::CvImm>(cvParams(100, 75), cvParams(100, 100), Pi,
                                             Eigen::Vector2d(0.5, 0.5));
        f->init(z, v);
        f->setMeasurementNoise(R);
        return f;
    }
    if (type == "plate_imm") {
        std::vector<kf::Imm::Model> ms;
        for (auto s : {0.5, 2.0}) {
            kf::PlateCvParams p;
            p.sigma_v = s * qscale;
            p.sigma_w = s * qscale;
            p.R = R;
            auto f = std::make_unique<kf::PlateCvKalman>(p);
            f->init(z, v);
            f->setMeasurementNoise(R);
            ms.push_back({"plate", std::move(f)});
        }
        Eigen::Matrix2d Pi; Pi << 0.95, 0.05, 0.05, 0.95;
        return std::make_unique<ImmAdapter>(kf::Imm(std::move(ms), Pi, Eigen::Vector2d(0.5, 0.5)));
    }
    throw std::runtime_error("unknown model " + type);
}

struct Result { double se = 0.0; int n = 0; };
}  // namespace

int main() {
    try {
        const YAML::Node cfg = YAML::LoadFile(sim::resolvePath("configs/kf/param_robust.yaml"));
        const int warmup = cfg["warmup"] ? cfg["warmup"].as<int>() : 25;
        const double sigma = cfg["true_sigma"] ? cfg["true_sigma"].as<double>() : 0.02;
        const Eigen::Matrix3d R0 = Eigen::Matrix3d::Identity() * sigma * sigma;

        std::vector<double> r_scales, q_scales;
        for (const auto& x : cfg["r_scales"]) r_scales.push_back(x.as<double>());
        for (const auto& x : cfg["q_scales"]) q_scales.push_back(x.as<double>());

        std::vector<std::pair<std::string, std::string>> models;
        for (const auto& m : cfg["models"])
            models.emplace_back(m["name"].as<std::string>(), m["type"].as<std::string>());

        // key: model, axis, scale
        std::map<std::tuple<std::string,std::string,double>, Result> acc;

        for (const auto& sc : cfg["scenarios"]) {
            const double duration = sc["duration"] ? sc["duration"].as<double>() : 8.0;
            auto setup = sim::loadSimSetup(sim::resolvePath(sc["path"].as<std::string>()));
            sim::Simulator sim(setup.geometry, setup.initial, std::move(setup.controller), setup.min_dt);
            sim::GaussianSensor sensor(setup.sensor);
            int locked = sensor.pickArmor(sim.truth());
            if (locked < 0) locked = 0;
            g_id = locked;

            const double min_interval = [&] {
                if (!setup.sensor.intervals.empty())
                    return *std::min_element(setup.sensor.intervals.begin(), setup.sensor.intervals.end());
                return std::max(1e-3, setup.sensor.interval - setup.sensor.jitter);
            }();
            const auto times = sim::makeObservationTimes(setup.sensor,
                                                         static_cast<std::size_t>(duration / min_interval) + 16);
            std::vector<Sample> ss;
            for (std::size_t k = 0; k < times.size() && times[k] <= duration; ++k) {
                sim.advanceTo(times[k]);
                auto o = sensor.observeArmorAt(sim.truth(), times[k], k, locked, nullptr);
                if (o.valid) ss.push_back({times[k], sim.truth().armors[locked].p, o.p});
            }
            if (ss.size() < 3) continue;
            const Eigen::Vector3d v0 = (ss[1].obs - ss[0].obs) / (ss[1].t - ss[0].t);

            auto run = [&](const std::string& name, const std::string& type,
                           const std::string& axis, double scale, double qscale,
                           const Eigen::Matrix3d& R) {
                auto f = build(type, ss[1].obs, v0, R, qscale);
                double last = ss[1].t;
                double se = 0.0;
                int n = 0;
                for (std::size_t i = 2; i < ss.size(); ++i) {
                    f->predict(std::max(1e-6, ss[i].t - last));
                    last = ss[i].t;
                    f->update(ss[i].obs);
                    if (static_cast<int>(i) < warmup + 2) continue;
                    se += (f->position() - ss[i].truth).squaredNorm();
                    ++n;
                }
                auto& a = acc[{name, axis, scale}];
                a.se += se / std::max(1, n);
                a.n += 1;
            };

            for (double s : r_scales)
                for (auto& [name, type] : models) run(name, type, "r", s, 1.0, R0 * s);
            for (double s : q_scales)
                for (auto& [name, type] : models) run(name, type, "q", s, s, R0);
            std::cerr << "done " << sc["path"].as<std::string>() << "\n";
        }

        std::filesystem::create_directories("build/kf");
        std::ofstream csv("build/kf/param_robust.csv");
        csv << "model,axis,scale,rmse_cm\n";
        for (auto& [key, r] : acc) {
            const auto& [name, axis, scale] = key;
            const double rmse = std::sqrt(r.se / std::max(1, r.n)) * 100.0;
            csv << name << ',' << axis << ',' << scale << ',' << rmse << '\n';
        }

        for (const std::string axis : {"r", "q"}) {
            std::cout << "\n" << (axis == "r" ? "R mismatch" : "Q mismatch") << ": RMSE [cm]\n";
            std::cout << std::left << std::setw(12) << "model";
            const auto& scales = axis == "r" ? r_scales : q_scales;
            for (double s : scales) std::cout << std::setw(8) << s;
            std::cout << "\n";
            for (auto& [name, type] : models) {
                std::cout << std::left << std::setw(12) << name << std::fixed << std::setprecision(2);
                for (double s : scales) {
                    const auto& a = acc[{name, axis, s}];
                    std::cout << std::setw(8) << std::sqrt(a.se / std::max(1, a.n)) * 100.0;
                }
                std::cout << "\n";
            }
        }
        std::cout << "\ncsv: build/kf/param_robust.csv\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
}
