// 演示 RTS 对恒定 r/l/h 的估计：
// 为了让 r/l/h 单独可观测，这里固定 center/yaw 为 sim 真值（只估 3 个参数）。
// 状态 x=[r,l,h]，Q=0；观测是所有可见装甲板相对 center 的位置。
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "sim/config.hpp"
#include "sim/core/paths.hpp"
#include "sim/sensing/sensor.hpp"
#include "sim/simulator.hpp"

namespace {
using Vec = Eigen::VectorXd;
using Mat = Eigen::MatrixXd;
constexpr int NX = 3;

struct Obs {
    int id = -1;
    Eigen::Vector3d z;  // 相对 center
};

Mat H_of(int id, int armor_num, double yaw)
{
    const double step = 2.0 * M_PI / armor_num;
    const double ang = yaw + id * step;
    const bool lh = (armor_num == 4) && (id == 1 || id == 3);
    Mat H = Mat::Zero(3, NX);
    H(0, 0) = -std::cos(ang);
    H(1, 0) = -std::sin(ang);
    if (lh) {
        H(0, 1) = -std::cos(ang);
        H(1, 1) = -std::sin(ang);
        H(2, 2) = 1.0;
    }
    return H;
}

Vec h_of(const Vec& x, int id, int armor_num, double yaw)
{
    const double step = 2.0 * M_PI / armor_num;
    const double ang = yaw + id * step;
    const bool lh = (armor_num == 4) && (id == 1 || id == 3);
    const double r = x[0] + (lh ? x[1] : 0.0);
    return Vec(Eigen::Vector3d(-r * std::cos(ang), -r * std::sin(ang), lh ? x[2] : 0.0));
}

}  // namespace

int main(int argc, char** argv)
{
    const std::string path = argc > 1 ? argv[1] : "configs/sim/rm_geom_spin.yaml";
    const double duration = 6.0;
    const double sigma = 0.02;
    auto setup = sim::loadSimSetup(path);
    sim::Simulator sim(setup.geometry, setup.initial, std::move(setup.controller), setup.min_dt);
    sim::GaussianSensor sensor(setup.sensor);
    const int armor_num = setup.geometry.armor_count;
    const double min_dt = setup.sensor.intervals.empty()
                            ? std::max(1e-3, setup.sensor.interval - setup.sensor.jitter)
                            : *std::min_element(setup.sensor.intervals.begin(), setup.sensor.intervals.end());
    const auto times = sim::makeObservationTimes(
        setup.sensor, static_cast<std::size_t>(duration / min_dt) + 32);

    Vec x{{0.22, 0.0, 0.0}};
    Mat P = Mat::Identity(NX, NX) * 0.1 * 0.1;
    bool init = false;
    struct Step {
        double t;
        std::vector<Obs> obs;
        double yaw;
        Vec xf;
        Mat Pf;
        Mat F;
        Mat Q;
    };
    std::vector<Step> hist;
    std::array<int, 4> id_count{0, 0, 0, 0};
    std::array<int, 4> id_frames{0, 0, 0, 0};

    for (std::size_t k = 0; k < times.size() && times[k] <= duration; ++k) {
        sim.advanceTo(times[k]);
        const auto truth = sim.truth();
        std::vector<Obs> obs;
        for (int id = 0; id < armor_num; ++id) {
            auto o = sensor.observeArmorAt(truth, times[k], k, id, nullptr);
            if (!o.valid) continue;
            obs.push_back({id, o.p - truth.chassis.p});
            id_count[id]++;
        }
        if (obs.empty()) continue;
        const double yaw = std::atan2(-truth.armors[0].n.y(), -truth.armors[0].n.x());
        // 观测相对 id0 板的高度，避免把 ±dh/2 的公共偏移算进 h
        const double z_ref = truth.armors[0].p.z() - truth.chassis.p.z();
        for (auto& o : obs) o.z.z() = o.z.z() - z_ref;

        for (const auto& o : obs) id_frames[o.id]++;
        if (!init) {
            init = true;
        } else {
            // 参数恒定：F=I, Q=0，不需要预测
            x = x;
            P = P;
        }

        // 批量更新所有可见板
        const int m = static_cast<int>(obs.size()) * 3;
        Vec z(m);
        Mat H = Mat::Zero(m, NX);
        for (int i = 0; i < static_cast<int>(obs.size()); ++i) {
            z.segment<3>(3 * i) = obs[i].z;
            H.block(3 * i, 0, 3, NX) = H_of(obs[i].id, armor_num, yaw);
        }
        const Mat R = Mat::Identity(m, m) * sigma * sigma;
        const Mat S = H * P * H.transpose() + R;
        const Mat K = P * H.transpose() * S.inverse();
        Vec nu = z;
        for (int i = 0; i < static_cast<int>(obs.size()); ++i) {
            nu.segment<3>(3 * i) -= h_of(x, obs[i].id, armor_num, yaw);
        }
        x = x + K * nu;
        P = (Mat::Identity(NX, NX) - K * H) * P;
        hist.push_back({times[k], obs, yaw, x, P, Mat::Identity(NX, NX), Mat::Zero(NX, NX)});
    }

    // RTS，F = I、Q = 0：
    //   x_{k+1|k} = x_{k|k}，P_{k+1|k} = P_{k|k}，增益 C = I
    //   x_{k|N} = x_{k|k} + (x_{k+1|N} - x_{k|k}) = x_{k+1|N}
    // 所以每一拍的平滑值和协方差都等于最后一拍滤波值。
    // 不能写成 P_{k|k} P_{k+1|k+1}^{-1} (x_{k+1|N} - x_{k+1|k+1})：
    // 最后一拍残差是 0，修正会整段消掉，平滑结果退回 causal。
    std::vector<Vec> xs(hist.size());
    std::vector<Mat> Ps(hist.size());
    xs.back() = hist.back().xf;
    Ps.back() = hist.back().Pf;
    for (int k = static_cast<int>(hist.size()) - 2; k >= 0; --k) {
        const Mat C = Mat::Identity(NX, NX);
        const Vec& xp = hist[k].xf;
        const Mat& Pp = hist[k].Pf;
        xs[k] = hist[k].xf + C * (xs[k + 1] - xp);
        Ps[k] = hist[k].Pf + C * (Ps[k + 1] - Pp) * C.transpose();
    }

    const double rt = setup.geometry.armor_a > 0 ? setup.geometry.armor_a
                                                  : setup.geometry.armor_radius;
    const double lt = (setup.geometry.armor_b > 0 ? setup.geometry.armor_b : rt) - rt;
    const double ht = -setup.geometry.armor_dh;  // id1/3 相对 id0/2 的高度差

    const auto csv_path = std::filesystem::path(sim::projectRoot()) / "build/kf/rts_params.csv";
    std::filesystem::create_directories(csv_path.parent_path());
    std::ofstream csv(csv_path);
    csv << "t,r_true,l_true,h_true,r_causal,l_causal,h_causal,r_smooth,l_smooth,h_smooth,"
           "sr_c,sl_c,sh_c,sr_s,sl_s,sh_s\n";
    for (std::size_t i = 0; i < hist.size(); ++i) {
        csv << hist[i].t << ',' << rt << ',' << lt << ',' << ht << ',' << hist[i].xf[0] << ','
            << hist[i].xf[1] << ',' << hist[i].xf[2] << ',' << xs[i][0] << ',' << xs[i][1] << ','
            << xs[i][2] << ',' << std::sqrt(std::max(0.0, hist[i].Pf(0, 0))) << ','
            << std::sqrt(std::max(0.0, hist[i].Pf(1, 1))) << ','
            << std::sqrt(std::max(0.0, hist[i].Pf(2, 2))) << ','
            << std::sqrt(std::max(0.0, Ps[i](0, 0))) << ','
            << std::sqrt(std::max(0.0, Ps[i](1, 1))) << ','
            << std::sqrt(std::max(0.0, Ps[i](2, 2))) << '\n';
    }
    std::cout << "truth r/l/h = " << rt << " / " << lt << " / " << ht << " m\n";
    std::cout << "causal final  = " << hist.back().xf.transpose() << " m\n";
    std::cout << "RTS    final  = " << xs.back().transpose() << " m\n";
    std::cout << "obs per id: " << id_count[0] << " " << id_count[1] << " " << id_count[2] << " "
              << id_count[3] << "\n";
    std::cout << "csv: " << csv_path << "\n";
    return 0;
}
