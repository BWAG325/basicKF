#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "kf/imm.hpp"
#include "kf/models/ca_ypd.hpp"
#include "kf/models/cv_ypd.hpp"
#include "sim/config.hpp"
#include "sim/core/paths.hpp"
#include "sim/sensing/sensor.hpp"
#include "sim/simulator.hpp"

namespace {
struct Sample {
    double t = 0.0;
    Eigen::Vector3d truth = Eigen::Vector3d::Zero();
    Eigen::Vector3d p = Eigen::Vector3d::Zero();
    double yaw_in = 0.0;
    Eigen::Vector4d z = Eigen::Vector4d::Zero();
    Eigen::Matrix4d R = Eigen::Matrix4d::Identity();
    bool has_R = false;
};

class ImmAdapter final : public kf::IFilter {
public:
    ImmAdapter(kf::Imm&& imm, int id) : imm_(std::move(imm)), id_(id) {}
    void predict(double dt) override { imm_.predict(dt); }
    void update(const Eigen::VectorXd& z) override { imm_.update(z); }
    void setMeasurementNoise(const Eigen::MatrixXd& R) override {
        for (int i = 0; i < imm_.modelCount(); ++i) {
            imm_.model(i).filter->setMeasurementNoise(R);
        }
    }
    [[nodiscard]] Eigen::Vector3d position() const override {
        const Eigen::VectorXd& x = imm_.state();
        const double ang = x[6] + id_ * 2.0 * M_PI / 4.0;
        const bool lh = (id_ == 1 || id_ == 3);
        const double r = x[8] + (lh ? x[9] : 0.0);
        return Eigen::Vector3d(x[0] - r * std::cos(ang), x[2] - r * std::sin(ang),
                               x[4] + (lh ? x[10] : 0.0));
    }
    [[nodiscard]] int stateDim() const override { return 11; }
    [[nodiscard]] int measDim() const override { return 4; }
    [[nodiscard]] Eigen::VectorXd state() const override { return imm_.state(); }
    [[nodiscard]] Eigen::MatrixXd covariance() const override { return imm_.covariance(); }
    [[nodiscard]] Eigen::VectorXd innovation() const override { return imm_.model(0).filter->innovation(); }
    [[nodiscard]] Eigen::MatrixXd innovationCov() const override {
        return imm_.model(0).filter->innovationCov();
    }
    [[nodiscard]] double nis() const override { return imm_.model(0).filter->nis(); }
    [[nodiscard]] double logLikelihood() const override { return imm_.model(0).filter->logLikelihood(); }
    void setState(const Eigen::VectorXd&, const Eigen::MatrixXd&) override {}

private:
    kf::Imm imm_;
    int id_;
};

int g_id = 0;

std::unique_ptr<kf::IFilter> makeFilter(const std::string& kind, const Sample& s, double radius,
                                        bool match_sensor) {
    auto cv = [&](double q) {
        kf::CvYpdParams p;
        p.q_xyz = q;
        p.q_yaw = 75;
        p.armor_radius = radius;
        p.armor_id = g_id;
        p.match_sensor = match_sensor;
        auto f = std::make_unique<kf::CvYpdKalman>(p);
        f->init(s.p, s.yaw_in);
        return f;
    };
    if (kind == "cv") return cv(100.0);
    kf::CvYpdParams cp;
    cp.q_xyz = 100.0;
    cp.q_yaw = 75;
    cp.armor_radius = radius;
    cp.armor_id = g_id;
    cp.match_sensor = match_sensor;
    auto a = std::make_unique<kf::CvYpdKalman>(cp);
    a->init(s.p, s.yaw_in);
    kf::CaYpdParams cap;
    cap.q_jerk = 16;
    cap.q_yaw = 75;
    cap.armor_radius = radius;
    cap.armor_id = g_id;
    cap.match_sensor = match_sensor;
    auto b = std::make_unique<kf::CaYpdKalman>(cap);
    b->init(s.p, s.yaw_in);
    std::vector<kf::Imm::Model> ms;
    ms.push_back({"cv", std::move(a)});
    ms.push_back({"ca", std::move(b)});
    Eigen::Matrix2d Pi;
    Pi << 0.95, 0.05, 0.08, 0.92;
    return std::make_unique<ImmAdapter>(
        kf::Imm(std::move(ms), Pi, Eigen::Vector2d(0.6, 0.4), std::vector<int>{6}), g_id);
}

struct Score {
    double se = 0.0;
    double nis = 0.0;
    int n = 0;
};

Score runOne(const std::string& kind, const std::vector<Sample>& ss, double radius, bool match,
             bool use_pnp_r, int warmup) {
    auto f = makeFilter(kind, ss[1], radius, match);
    double last = ss[1].t;
    Score s;
    for (std::size_t i = 2; i < ss.size(); ++i) {
        f->predict(std::max(1e-6, ss[i].t - last));
        last = ss[i].t;
        if (use_pnp_r && ss[i].has_R) f->setMeasurementNoise(ss[i].R);
        f->update(ss[i].z);
        if (static_cast<int>(i) < warmup + 2) continue;
        s.se += (f->position() - ss[i].truth).squaredNorm();
        s.nis += f->nis();
        ++s.n;
    }
    return s;
}
}  // namespace

int main() {
    try {
        const YAML::Node cfg = YAML::LoadFile(sim::resolvePath("configs/kf/ypd.yaml"));
        const int warmup = cfg["warmup"] ? cfg["warmup"].as<int>() : 25;

        auto setup0 = sim::loadSimSetup(sim::resolvePath("configs/sim/rm_shuttle.yaml"));
        setup0.sensor.pnp.enabled = true;
        setup0.sensor.pnp.pixel_sigma = 0.0;
        setup0.sensor.dropout = 0.0;
        sim::Simulator sim0(setup0.geometry, setup0.initial, std::move(setup0.controller), setup0.min_dt);
        sim::GaussianSensor sensor0(setup0.sensor);
        const int id0 = std::max(0, sensor0.pickArmor(sim0.truth()));
        const auto o0 = sensor0.observeArmorAt(sim0.truth(), 0.0, 0, id0, nullptr);
        const auto& arm0 = sim0.truth().armors[static_cast<std::size_t>(id0)];
        const double zerr = o0.valid ? (o0.p - arm0.p).norm() : 1e9;
        const double yaw_t = std::atan2(-arm0.n.y(), -arm0.n.x());
        const double yaw_o = std::atan2(-o0.n.y(), -o0.n.x());
        const double yaw_e = std::atan2(std::sin(yaw_o - yaw_t), std::cos(yaw_o - yaw_t));
        std::cout << "zero-noise pnp err_mm " << zerr * 1000.0 << " yaw_err_deg " << yaw_e * 180.0 / M_PI
                  << " valid " << o0.valid << "\n";
        if (!o0.valid || zerr > 0.005) {
            std::cerr << "PnP 零噪声没有回到真值，先不跑对比\n";
            return 1;
        }
        std::cout << "n_obs " << o0.n.transpose() << " n_true " << arm0.n.transpose() << "\n";

        std::cout << "列是 rmse_cm/mean_nis。formula=哨兵 R，match=2cm，pnp=本帧 (J^T J)^{-1}。"
                     " 过程噪声 q_xyz=100。\n";

        for (double pixel_sigma : {1.0, 0.25}) {
        std::cout << "\npixel_sigma " << pixel_sigma << "\n";
        std::cout << "scenario obs_cm yaw_deg rdist_m rang_deg cv_formula cv_match cv_pnp imm_formula imm_pnp\n";

        for (const auto& sc : cfg["scenarios"]) {
            const std::string name = sc["name"].as<std::string>();
            const double duration = sc["duration"] ? sc["duration"].as<double>() : 8.0;
            auto setup = sim::loadSimSetup(sim::resolvePath(sc["path"].as<std::string>()));
            setup.sensor.pnp.enabled = true;
            setup.sensor.pnp.pixel_sigma = pixel_sigma;
            sim::Simulator sim(setup.geometry, setup.initial, std::move(setup.controller), setup.min_dt);
            sim::GaussianSensor sensor(setup.sensor);
            int locked = sensor.pickArmor(sim.truth());
            if (locked < 0) locked = 0;
            g_id = locked;

            const double dt_min = [&] {
                if (!setup.sensor.intervals.empty())
                    return *std::min_element(setup.sensor.intervals.begin(), setup.sensor.intervals.end());
                return std::max(1e-3, setup.sensor.interval - setup.sensor.jitter);
            }();
            const auto times = sim::makeObservationTimes(
                setup.sensor, static_cast<std::size_t>(duration / dt_min) + 16);

            std::vector<Sample> ss;
            double obs_se = 0.0;
            double yaw_se = 0.0;
            double rdist = 0.0;
            double rang = 0.0;
            for (std::size_t k = 0; k < times.size() && times[k] <= duration; ++k) {
                sim.advanceTo(times[k]);
                const auto o = sensor.observeArmorAt(sim.truth(), times[k], k, locked, nullptr);
                if (!o.valid || !o.has_ypd_cov) continue;
                const auto& arm = sim.truth().armors[static_cast<std::size_t>(locked)];
                const double err = (o.p - arm.p).norm();
                const double yaw_true = std::atan2(-arm.n.y(), -arm.n.x());
                const double yaw_obs = std::atan2(-o.n.y(), -o.n.x());
                const double dyaw = std::atan2(std::sin(yaw_obs - yaw_true), std::cos(yaw_obs - yaw_true));
                obs_se += err * err;
                yaw_se += dyaw * dyaw;
                rdist += std::sqrt(std::max(0.0, o.R_ypd(2, 2)));
                rang += std::sqrt(std::max(0.0, o.R_ypd(3, 3)));
                Sample s;
                s.t = times[k];
                s.truth = arm.p;
                s.p = o.p;
                s.yaw_in = std::atan2(-o.n.y(), -o.n.x());
                const double horiz = std::hypot(o.p.x(), o.p.y());
                s.z[0] = std::atan2(o.p.y(), o.p.x());
                s.z[1] = std::atan2(o.p.z(), horiz);
                s.z[2] = o.p.norm();
                s.z[3] = s.yaw_in;
                s.R = o.R_ypd;
                s.has_R = true;
                ss.push_back(s);
            }
            if (ss.size() < static_cast<std::size_t>(warmup + 5)) {
                std::cout << name << " too_few " << ss.size() << "\n";
                continue;
            }
            const double nobs = static_cast<double>(ss.size());
            const double obs_cm = 100.0 * std::sqrt(obs_se / nobs);
            const double yaw_deg = std::sqrt(yaw_se / nobs) * 180.0 / M_PI;
            const double rdist_m = rdist / nobs;
            const double rang_deg = rang / nobs * 180.0 / M_PI;
            const auto show = [](const Score& s) {
                if (s.n <= 0) return std::string("nan");
                char buf[64];
                std::snprintf(buf, sizeof(buf), "%.2f/%.2f",
                              100.0 * std::sqrt(s.se / s.n), s.nis / s.n);
                return std::string(buf);
            };
            const Score cv_f = runOne("cv", ss, setup.geometry.armor_radius, false, false, warmup);
            const Score cv_m = runOne("cv", ss, setup.geometry.armor_radius, true, false, warmup);
            const Score cv_p = runOne("cv", ss, setup.geometry.armor_radius, false, true, warmup);
            const Score im_f = runOne("imm", ss, setup.geometry.armor_radius, false, false, warmup);
            const Score im_p = runOne("imm", ss, setup.geometry.armor_radius, false, true, warmup);
            std::cout << name << " " << obs_cm << " " << yaw_deg << " " << rdist_m << " " << rang_deg
                      << " " << show(cv_f) << " " << show(cv_m) << " " << show(cv_p) << " "
                      << show(im_f) << " " << show(im_p) << "\n";
        }
        }
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << "\n";
        return 1;
    }
}
