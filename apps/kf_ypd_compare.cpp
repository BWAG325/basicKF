#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "kf/fusion.hpp"
#include "kf/imm.hpp"
#include "kf/so3_ekf.hpp"
#include "kf/models/ca_ypd.hpp"
#include "kf/models/cv_ypd.hpp"
#include "sim/config.hpp"
#include "sim/core/paths.hpp"
#include "sim/sensing/sensor.hpp"
#include "sim/simulator.hpp"

namespace {
struct Sample { double t; Eigen::Vector3d truth, p; double yaw_in, yaw_out; };
int g_id = 0;

class ImmAdapter final : public kf::IFilter {
public:
    ImmAdapter(kf::Imm&& imm, int id) : imm_(std::move(imm)), id_(id) {}
    void predict(double dt) override { imm_.predict(dt); }
    void update(const Eigen::VectorXd& z) override { imm_.update(z); }
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
    [[nodiscard]] Eigen::MatrixXd innovationCov() const override { return imm_.model(0).filter->innovationCov(); }
    [[nodiscard]] double nis() const override { return imm_.model(0).filter->nis(); }
    [[nodiscard]] double logLikelihood() const override { return imm_.model(0).filter->logLikelihood(); }
    void setState(const Eigen::VectorXd&, const Eigen::MatrixXd&) override {}
    [[nodiscard]] const Eigen::VectorXd& probabilities() const { return imm_.probabilities(); }
    [[nodiscard]] Eigen::Vector3d modelPosition(int i) const { return imm_.model(i).filter->position(); }
private:
    kf::Imm imm_;
    int id_;
};

std::unique_ptr<kf::IFilter> make(const std::string& type, const Sample& s, double radius, double q2scale) {
    auto mk = [&](double qx, double qy) {
        kf::CvYpdParams p;
        p.q_xyz = qx;
        p.q_yaw = qy;
        p.armor_radius = radius;
        p.armor_id = g_id;
        p.match_sensor = true;
        auto f = std::make_unique<kf::CvYpdKalman>(p);
        f->init(s.p, s.yaw_in);
        return f;
    };
    if (type == "cv") return mk(1.0, 75);
    if (type == "so3") {
        kf::So3Ekf::Params p;
        p.q_xyz = 100; p.q_yaw = 75; p.armor_id = g_id; p.armor_num = 4;
        p.Rm = Eigen::MatrixXd::Zero(4, 4);
        p.Rm(0, 0) = p.Rm(1, 1) = 1e-4;
        p.Rm(2, 2) = 4e-4;
        p.Rm(3, 3) = 1e-4;
        auto f = std::make_unique<kf::So3Ekf>(p);
        f->init(s.p, kf::So3Ekf::wrap(s.yaw_out + M_PI));
        f->setArmor(g_id, 4);
        return f;
    }
    if (type == "ukf") {
        kf::CvYpdParams p; p.q_xyz = 100; p.q_yaw = 75; p.armor_radius = radius; p.armor_id = g_id;
        p.match_sensor = true;
        auto f = std::make_unique<kf::CvYpdUkf>(p);
        f->init(s.p, s.yaw_in);
        return f;
    }
    if (type == "ca") {
        kf::CaYpdParams p;
        p.q_jerk = 16;
        p.q_yaw = 75;
        p.armor_radius = radius;
        p.armor_id = g_id;
        p.match_sensor = true;
        auto f = std::make_unique<kf::CaYpdKalman>(p);
        f->init(s.p, s.yaw_in);
        return f;
    }
    if (type == "cv_ca") {
        std::vector<kf::Imm::Model> ms;
        {
            kf::CvYpdParams p;
            p.q_xyz = 1.0;
            p.q_yaw = 75;
            p.armor_radius = radius;
            p.armor_id = g_id;
            p.match_sensor = true;
            auto f = std::make_unique<kf::CvYpdKalman>(p);
            f->init(s.p, s.yaw_in);
            ms.push_back({"cv", std::move(f)});
        }
        {
            kf::CaYpdParams p;
            p.q_jerk = 16;
            p.q_yaw = 75;
            p.armor_radius = radius;
            p.armor_id = g_id;
            p.match_sensor = true;
            auto f = std::make_unique<kf::CaYpdKalman>(p);
            f->init(s.p, s.yaw_in);
            ms.push_back({"ca", std::move(f)});
        }
        Eigen::Matrix2d Pi;
        Pi << 0.95, 0.05, 0.08, 0.92;
        return std::make_unique<ImmAdapter>(
            kf::Imm(std::move(ms), Pi, Eigen::Vector2d(0.6, 0.4), std::vector<int>{6}), g_id);
    }
    if (type == "imm") {
        std::vector<kf::Imm::Model> ms;
        {
            kf::CvYpdParams p; p.q_xyz = 100; p.q_yaw = 75; p.armor_radius = radius; p.armor_id = g_id;
            p.match_sensor = true;
            auto f = std::make_unique<kf::CvYpdKalman>(p);
            f->init(s.p, s.yaw_in);
            ms.push_back({"ekf", std::move(f)});
        }
        {
            kf::CvYpdParams p; p.q_xyz = 100; p.q_yaw = 75; p.armor_radius = radius; p.armor_id = g_id;
            p.match_sensor = true;
            auto f = std::make_unique<kf::CvYpdUkf>(p);
            f->init(s.p, s.yaw_in);
            ms.push_back({"ukf", std::move(f)});
        }
        Eigen::Matrix2d Pi; Pi << 0.95, 0.05, 0.05, 0.95;
        return std::make_unique<ImmAdapter>(
            kf::Imm(std::move(ms), Pi, Eigen::Vector2d(0.5, 0.5), std::vector<int>{6}), g_id);
    }
    if (type == "fusion") {
        std::vector<std::unique_ptr<kf::IFilter>> ms;
        ms.push_back(mk(100, 75));
        ms.push_back(mk(100, 100 * q2scale));
        Eigen::Matrix2d Pi; Pi << 0.95, 0.05, 0.05, 0.95;
        return std::make_unique<kf::OutputFusion>(std::move(ms), Pi, Eigen::Vector2d(0.5, 0.5));
    }
    throw std::runtime_error("unknown " + type);
}

Eigen::VectorXd meas(const Sample& s, bool outward = false) {
    Eigen::VectorXd z(4);
    const double q = std::sqrt(s.p.x() * s.p.x() + s.p.y() * s.p.y());
    z[0] = std::atan2(s.p.y(), s.p.x());
    z[1] = std::atan2(s.p.z(), q);
    z[2] = s.p.norm();
    z[3] = outward ? s.yaw_out : s.yaw_in;
    return z;
}
}  // namespace

int main() {
    try {
        const YAML::Node cfg = YAML::LoadFile(sim::resolvePath("configs/kf/ypd.yaml"));
        const int warmup = cfg["warmup"] ? cfg["warmup"].as<int>() : 25;
        std::vector<std::string> models;
        for (const auto& m : cfg["models"]) models.push_back(m.as<std::string>());

        std::vector<std::pair<std::string, std::pair<double,int>>> per;  // scenario, (se,n)
        std::ofstream csv("build/kf/ypd_compare.csv");
        csv << "scenario,model,rmse_cm\n";

        for (const auto& sc : cfg["scenarios"]) {
            const std::string name = sc["name"].as<std::string>();
            const double duration = sc["duration"] ? sc["duration"].as<double>() : 8.0;
            auto setup = sim::loadSimSetup(sim::resolvePath(sc["path"].as<std::string>()));
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
            const auto times = sim::makeObservationTimes(setup.sensor,
                                                         static_cast<std::size_t>(duration / dt_min) + 16);
            std::vector<Sample> ss;
            for (std::size_t k = 0; k < times.size() && times[k] <= duration; ++k) {
                sim.advanceTo(times[k]);
                auto o = sensor.observeArmorAt(sim.truth(), times[k], k, locked, nullptr);
                if (o.valid) {
                    const double yaw_out = std::atan2(o.n.y(), o.n.x());
                    const double yaw_in = std::atan2(-o.n.y(), -o.n.x());
                    ss.push_back({times[k], sim.truth().armors[locked].p, o.p, yaw_in, yaw_out});
                }
            }
            if (ss.size() < 3) continue;

            std::cout << name;
            std::ofstream traj;
            if (name == "shuttle" || name == "ellipse_spin") {
                const auto path = std::filesystem::path(sim::projectRoot()) / "build/kf" /
                                  ("ypd_traj_" + name + ".csv");
                std::filesystem::create_directories(path.parent_path());
                traj.open(path);
                traj << "t,truth_x,truth_y,cv_x,cv_y,ca_x,ca_y,imm_x,imm_y\n";
                traj << std::fixed << std::setprecision(5);
            }
            std::vector<double> ts, tx, ty, cx, cy, ax, ay, ix, iy;
            for (const auto& type : models) {
                auto f = make(type, ss[1], setup.geometry.armor_radius, 1.0);
                double last = ss[1].t, se = 0.0;
                int n = 0;
                Eigen::Vector2d mu_sum = Eigen::Vector2d::Zero();
                double se_cv = 0.0, se_ca = 0.0;
                auto* imm = dynamic_cast<ImmAdapter*>(f.get());
                for (std::size_t i = 2; i < ss.size(); ++i) {
                    f->predict(std::max(1e-6, ss[i].t - last));
                    last = ss[i].t;
                    f->update(meas(ss[i], type == "so3"));
                    if (traj && ss[i].t <= 7.0) {
                        if (type == "cv") {
                            ts.push_back(ss[i].t);
                            tx.push_back(ss[i].truth.x());
                            ty.push_back(ss[i].truth.y());
                            cx.push_back(f->position().x());
                            cy.push_back(f->position().y());
                        } else if (type == "ca") {
                            ax.push_back(f->position().x());
                            ay.push_back(f->position().y());
                        } else if (type == "cv_ca") {
                            ix.push_back(f->position().x());
                            iy.push_back(f->position().y());
                        }
                    }
                    if (static_cast<int>(i) < warmup + 2) continue;
                    se += (f->position() - ss[i].truth).squaredNorm();
                    if (imm && type == "cv_ca") {
                        mu_sum += imm->probabilities();
                        se_cv += (imm->modelPosition(0) - ss[i].truth).squaredNorm();
                        se_ca += (imm->modelPosition(1) - ss[i].truth).squaredNorm();
                    }
                    ++n;
                }
                if (imm && type == "cv_ca" && n > 0) {
                    const Eigen::Vector2d mu = mu_sum / n;
                    std::cout << "  mu " << std::setprecision(2) << mu[0] << "/" << mu[1]
                              << "  leaf " << std::sqrt(se_cv / n) * 100.0 << "/"
                              << std::sqrt(se_ca / n) * 100.0;
                }
                const double rmse = std::sqrt(se / std::max(1, n)) * 100.0;
                std::cout << "  " << type << " " << std::fixed << std::setprecision(2) << rmse;
                csv << name << ',' << type << ',' << rmse << '\n';
                per.push_back({name, {rmse, 1}});
            }
            if (traj && !ts.empty() && cx.size() == ts.size() && ax.size() == ts.size() &&
                ix.size() == ts.size()) {
                for (std::size_t i = 0; i < ts.size(); ++i) {
                    traj << ts[i] << ',' << tx[i] << ',' << ty[i] << ',' << cx[i] << ',' << cy[i]
                         << ',' << ax[i] << ',' << ay[i] << ',' << ix[i] << ',' << iy[i] << '\n';
                }
            }
            std::cout << "\n";
        }
        std::cout << "csv: build/kf/ypd_compare.csv\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
}
