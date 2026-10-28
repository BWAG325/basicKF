// 恒定 r/l/h 能不能拿去改进实时卡尔曼。
//
// 同一段「原地小陀螺」观测上比较。板的可见性来自仿真（朝向/FOV/丢帧），
// 板的位置按滤波器自己的 r/l/h 模型生成，再叠上仿真的位置噪声。
// 这样 r/l/h 是模型里真正的常数；仿真椭圆半轴并不是这个常数。
//   1. causal：11 维 EKF 在线同时估运动学和 r/l/h
//   2. RTS：对这条滤波做后向平滑（用了未来量测，不是实时）
//   3. replay：用整段 causal/RTS 终值冻住 r/l/h，从头再跑实时滤波
//   4. oracle：冻住真值 r/l/h 的实时滤波（上界）
//   5. nominal：冻住错误标称值 0.22/0/0 的实时滤波
//   6. freeze：实时跑到 1.5 s，把当时的 r/l/h 冻住再继续（不用未来数据）
//
// Q=0 时 RTS 的 r/l/h 终值等于 causal 终值。replay 用的就是这个数。
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cmath>
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
constexpr int NX = 11;  // [cx,vx,cy,vy,cz,vz,yaw,w,r,l,h]
constexpr double kFreezeT = 1.5;

double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }

Vec f_model(const Vec& x, double dt)
{
    Vec y = x;
    y[0] += x[1] * dt;
    y[2] += x[3] * dt;
    y[4] += x[5] * dt;
    y[6] = wrap(x[6] + x[7] * dt);
    return y;
}

Mat F_numeric(const Vec& x, double dt)
{
    Mat F(NX, NX);
    const Vec f0 = f_model(x, dt);
    for (int j = 0; j < NX; ++j) {
        const double eps = 1e-6 * std::max(1.0, std::abs(x[j]));
        Vec xp = x;
        xp[j] += eps;
        F.col(j) = (f_model(xp, dt) - f0) / eps;
    }
    return F;
}

Mat Q_model(double dt)
{
    const double a = dt * dt * dt * dt / 4.0;
    const double b = dt * dt * dt / 2.0;
    const double c = dt * dt;
    Mat Q = Mat::Zero(NX, NX);
    for (int k : {0, 2, 4}) {
        Q(k, k) = a * 100.0;
        Q(k, k + 1) = b * 100.0;
        Q(k + 1, k) = b * 100.0;
        Q(k + 1, k + 1) = c * 100.0;
    }
    Q(6, 6) = a * 75.0;
    Q(6, 7) = b * 75.0;
    Q(7, 6) = b * 75.0;
    Q(7, 7) = c * 75.0;
    return Q;
}

void measurement(const Vec& x, int id, int armor_num, Eigen::Vector3d& h, Mat& H)
{
    const double step = 2.0 * M_PI / armor_num;
    const double ang = x[6] + id * step;
    const bool lh = (armor_num == 4) && (id == 1 || id == 3);
    const double r = x[8] + (lh ? x[9] : 0.0);
    h = Eigen::Vector3d(x[0] - r * std::cos(ang), x[2] - r * std::sin(ang),
                        x[4] + (lh ? x[10] : 0.0));
    H = Mat::Zero(3, NX);
    H(0, 0) = 1.0;
    H(1, 2) = 1.0;
    H(2, 4) = 1.0;
    H(0, 6) = r * std::sin(ang);
    H(1, 6) = -r * std::cos(ang);
    H(0, 8) = -std::cos(ang);
    H(1, 8) = -std::sin(ang);
    if (lh) {
        H(0, 9) = -std::cos(ang);
        H(1, 9) = -std::sin(ang);
        H(2, 10) = 1.0;
    }
}

struct Obs {
    int id = -1;
    Eigen::Vector3d z;
    Eigen::Vector3d truth;
    Eigen::Vector3d n;
};

struct Frame {
    double t = 0.0;
    std::vector<Obs> obs;
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    double yaw = 0.0;
    double z_ref = 0.0;
};

enum class ParamMode { Estimate, Frozen, FreezeAfter };

struct Run {
    std::vector<Vec> x;
    std::vector<Mat> P;
};

void freeze_params(Vec& x, Mat& P, double r, double l, double h)
{
    x[8] = r;
    x[9] = l;
    x[10] = h;
    for (int i : {8, 9, 10}) {
        P.row(i).setZero();
        P.col(i).setZero();
        P(i, i) = 1e-12;
    }
}

void update(Vec& x, Mat& P, const std::vector<Obs>& obs, int armor_num, double sigma)
{
    const int m = static_cast<int>(obs.size()) * 3;
    Vec z(m);
    Vec hx(m);
    Mat H = Mat::Zero(m, NX);
    const Mat R = Mat::Identity(m, m) * sigma * sigma;
    for (int i = 0; i < static_cast<int>(obs.size()); ++i) {
        Eigen::Vector3d hi;
        Mat Hi;
        measurement(x, obs[i].id, armor_num, hi, Hi);
        hx.segment<3>(3 * i) = hi;
        z.segment<3>(3 * i) = obs[i].z;
        H.block(3 * i, 0, 3, NX) = Hi;
    }
    Mat S = H * P * H.transpose() + R;
    S = 0.5 * (S + S.transpose());
    const Mat K = P * H.transpose() * S.ldlt().solve(Mat::Identity(m, m));
    x += K * (z - hx);
    x[6] = wrap(x[6]);
    const Mat IKH = Mat::Identity(NX, NX) - K * H;
    P = IKH * P * IKH.transpose() + K * R * K.transpose();
    P = 0.5 * (P + P.transpose());
}

Run filter_forward(const std::vector<Frame>& frames, int armor_num, double sigma, ParamMode mode,
                   double r0, double l0, double h0, std::vector<Vec>* pred_x, std::vector<Mat>* pred_P,
                   std::vector<Mat>* pred_F)
{
    Run out;
    const Obs& o0 = frames.front().obs.front();
    const double step = 2.0 * M_PI / armor_num;
    const double ang = wrap(std::atan2(-o0.n.y(), -o0.n.x()));
    Vec x = Vec::Zero(NX);
    x[6] = wrap(ang - o0.id * step);
    x[8] = r0;
    x[9] = l0;
    x[10] = h0;
    const double a0 = x[6] + o0.id * step;
    const bool lh0 = (armor_num == 4) && (o0.id == 1 || o0.id == 3);
    const double r_eff = x[8] + (lh0 ? x[9] : 0.0);
    x[0] = o0.z.x() + r_eff * std::cos(a0);
    x[2] = o0.z.y() + r_eff * std::sin(a0);
    x[4] = o0.z.z() - (lh0 ? x[10] : 0.0);

    Mat P = Mat::Zero(NX, NX);
    P(0, 0) = P(2, 2) = 0.30 * 0.30;
    P(4, 4) = 0.10 * 0.10;
    P(1, 1) = P(3, 3) = 1.5 * 1.5;
    P(5, 5) = 0.5 * 0.5;
    P(6, 6) = 0.40 * 0.40;
    P(7, 7) = 3.0 * 3.0;
    P(8, 8) = P(9, 9) = 0.08 * 0.08;
    P(10, 10) = 0.05 * 0.05;
    if (mode == ParamMode::Frozen) freeze_params(x, P, r0, l0, h0);

    bool frozen = mode == ParamMode::Frozen;
    for (std::size_t k = 0; k < frames.size(); ++k) {
        if (k > 0) {
            const double dt = std::max(1e-6, frames[k].t - frames[k - 1].t);
            const Mat F = F_numeric(x, dt);
            const Vec xp = f_model(x, dt);
            Mat Pp = F * P * F.transpose() + Q_model(dt);
            Pp = 0.5 * (Pp + Pp.transpose());
            if (pred_x) pred_x->push_back(xp);
            if (pred_P) pred_P->push_back(Pp);
            if (pred_F) pred_F->push_back(F);
            x = xp;
            P = Pp;
        }
        if (mode == ParamMode::FreezeAfter && !frozen && frames[k].t >= kFreezeT) {
            freeze_params(x, P, x[8], x[9], x[10]);
            frozen = true;
        }
        update(x, P, frames[k].obs, armor_num, sigma);
        if (frozen) freeze_params(x, P, x[8], x[9], x[10]);
        out.x.push_back(x);
        out.P.push_back(P);
    }
    return out;
}

std::vector<Vec> rts_smooth(const std::vector<Vec>& xf, const std::vector<Mat>& Pf,
                            const std::vector<Vec>& xp, const std::vector<Mat>& Pp,
                            const std::vector<Mat>& F)
{
    const std::size_t n = xf.size();
    std::vector<Vec> xs(n);
    xs.back() = xf.back();
    xs.back()[6] = wrap(xs.back()[6]);
    for (int k = static_cast<int>(n) - 2; k >= 0; --k) {
        const std::size_t i = static_cast<std::size_t>(k);
        // xp/Pp/F 的第 i 项是从 i 预测到 i+1（pred 比 xf 少存初值那一拍）
        Vec d = xs[i + 1] - xp[i];
        d[6] = wrap(d[6]);
        const Vec g = Pp[i].ldlt().solve(d);
        xs[i] = xf[i] + Pf[i] * (F[i].transpose() * g);
        xs[i][6] = wrap(xs[i][6]);
    }
    return xs;
}

double armor_sse(const Vec& x, const std::vector<Obs>& obs, int armor_num, double& count)
{
    double sse = 0.0;
    for (const Obs& o : obs) {
        Eigen::Vector3d h;
        Mat H;
        measurement(x, o.id, armor_num, h, H);
        sse += (h - o.truth).squaredNorm();
        count += 1.0;
    }
    return sse;
}

double rmse_from_sse(double sse, double n) { return std::sqrt(sse / std::max(1.0, n)); }

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
                            : *std::min_element(setup.sensor.intervals.begin(),
                                                setup.sensor.intervals.end());
    const auto times = sim::makeObservationTimes(
        setup.sensor, static_cast<std::size_t>(duration / min_dt) + 32);

    const double rt = setup.geometry.armor_a > 0 ? setup.geometry.armor_a : setup.geometry.armor_radius;
    const double lt = (setup.geometry.armor_b > 0 ? setup.geometry.armor_b : rt) - rt;
    const double ht = -setup.geometry.armor_dh;

    const Eigen::Vector3d cam = setup.sensor.camera.position;
    const double facing_min = setup.sensor.facing_min_cos;
    std::array<int, 4> id_count{0, 0, 0, 0};
    std::vector<Frame> frames;
    for (std::size_t k = 0; k < times.size() && times[k] <= duration; ++k) {
        sim.advanceTo(times[k]);
        const auto truth = sim.truth();
        Frame fr;
        fr.t = times[k];
        fr.center = truth.chassis.p;
        fr.yaw = std::atan2(-truth.armors[0].n.y(), -truth.armors[0].n.x());
        fr.z_ref = truth.chassis.p.z();
        Vec xt = Vec::Zero(NX);
        xt[0] = fr.center.x();
        xt[2] = fr.center.y();
        xt[4] = fr.z_ref;
        xt[6] = fr.yaw;
        xt[8] = rt;
        xt[9] = lt;
        xt[10] = ht;
        for (int id = 0; id < armor_num; ++id) {
            const auto& plate = truth.armors[static_cast<std::size_t>(id)];
            const Eigen::Vector3d to_cam = cam - plate.p;
            const double range = to_cam.norm();
            if (!(range > 1e-6) || plate.n.dot(to_cam / range) < facing_min) continue;
            auto o = sensor.observeArmorAt(truth, times[k], k, id, nullptr);
            if (!o.valid) continue;
            Eigen::Vector3d h;
            Mat H;
            measurement(xt, id, armor_num, h, H);
            // 保留这一拍仿真采到的位置噪声，只把几何换成滤波器模型。
            const Eigen::Vector3d z = h + (o.p - plate.p);
            fr.obs.push_back({id, z, h, o.n});
            if (id >= 0 && id < 4) id_count[static_cast<std::size_t>(id)]++;
        }
        if (!fr.obs.empty()) frames.push_back(std::move(fr));
    }
    if (frames.size() < 5) {
        std::cerr << "too few frames\n";
        return 1;
    }

    std::vector<Vec> xp;
    std::vector<Mat> Pp, Fp;
    const Run causal = filter_forward(frames, armor_num, sigma, ParamMode::Estimate, 0.22, 0.0, 0.0,
                                      &xp, &Pp, &Fp);
    const std::vector<Vec> smooth = rts_smooth(causal.x, causal.P, xp, Pp, Fp);
    const double r_hat = causal.x.back()[8];
    const double l_hat = causal.x.back()[9];
    const double h_hat = causal.x.back()[10];
    const Run replay =
        filter_forward(frames, armor_num, sigma, ParamMode::Frozen, r_hat, l_hat, h_hat, nullptr,
                       nullptr, nullptr);
    const Run oracle =
        filter_forward(frames, armor_num, sigma, ParamMode::Frozen, rt, lt, ht, nullptr, nullptr,
                       nullptr);
    const Run nominal =
        filter_forward(frames, armor_num, sigma, ParamMode::Frozen, 0.22, 0.0, 0.0, nullptr, nullptr,
                       nullptr);
    const Run freeze =
        filter_forward(frames, armor_num, sigma, ParamMode::FreezeAfter, 0.22, 0.0, 0.0, nullptr,
                       nullptr, nullptr);

    const auto index_of = [&](const Run& run) -> const std::vector<Vec>& { return run.x; };
    struct Acc {
        double center = 0, armor = 0, yaw = 0, n = 0, plates = 0;
        double center_late = 0, armor_late = 0, n_late = 0, plates_late = 0;
    };
    auto accumulate = [&](const std::vector<Vec>& xs) {
        Acc a;
        for (std::size_t i = 0; i < frames.size(); ++i) {
            const Vec& x = xs[i];
            const double ce =
                Eigen::Vector3d(x[0] - frames[i].center.x(), x[2] - frames[i].center.y(),
                                x[4] - frames[i].z_ref)
                    .squaredNorm();
            const double ye = wrap(x[6] - frames[i].yaw);
            double plates = 0.0;
            const double ae = armor_sse(x, frames[i].obs, armor_num, plates);
            a.center += ce;
            a.yaw += ye * ye;
            a.armor += ae;
            a.n += 1.0;
            a.plates += plates;
            if (frames[i].t >= kFreezeT) {
                a.center_late += ce;
                a.armor_late += ae;
                a.n_late += 1.0;
                a.plates_late += plates;
            }
        }
        return a;
    };
    const Acc ac = accumulate(index_of(causal));
    const Acc as = accumulate(smooth);
    const Acc ar = accumulate(index_of(replay));
    const Acc ao = accumulate(index_of(oracle));
    const Acc an = accumulate(index_of(nominal));
    const Acc af = accumulate(index_of(freeze));

    double r_spread = 0.0;
    for (const Vec& x : smooth) r_spread = std::max(r_spread, std::abs(x[8] - r_hat));
    double l_spread = 0.0;
    for (const Vec& x : smooth) l_spread = std::max(l_spread, std::abs(x[9] - l_hat));
    double h_spread = 0.0;
    for (const Vec& x : smooth) h_spread = std::max(h_spread, std::abs(x[10] - h_hat));

    const auto csv_path = std::filesystem::path(sim::projectRoot()) / "build/kf/rts_feasibility.csv";
    std::filesystem::create_directories(csv_path.parent_path());
    std::ofstream csv(csv_path);
    if (!csv) {
        std::cerr << "cannot write " << csv_path << "\n";
        return 1;
    }
    csv << "t,r_true,l_true,h_true,r_causal,l_causal,h_causal,r_rts,l_rts,h_rts,"
           "center_causal,center_rts,center_replay,center_oracle,center_nominal,center_freeze,"
           "armor_causal,armor_rts,armor_replay,armor_oracle,armor_nominal,armor_freeze,"
           "yaw_causal,yaw_rts,yaw_replay,yaw_oracle,yaw_nominal,yaw_freeze\n";
    csv.setf(std::ios::fixed);
    csv.precision(8);
    for (std::size_t i = 0; i < frames.size(); ++i) {
        const auto cerr = [&](const Vec& x) {
            return Eigen::Vector3d(x[0] - frames[i].center.x(), x[2] - frames[i].center.y(),
                                   x[4] - frames[i].z_ref)
                .norm();
        };
        const auto aerr = [&](const Vec& x) {
            double n = 0.0;
            return std::sqrt(armor_sse(x, frames[i].obs, armor_num, n) / std::max(1.0, n));
        };
        const auto yerr = [&](const Vec& x) { return std::abs(wrap(x[6] - frames[i].yaw)); };
        const Vec& c = causal.x[i];
        const Vec& s = smooth[i];
        const Vec& rp = replay.x[i];
        const Vec& o = oracle.x[i];
        const Vec& nm = nominal.x[i];
        const Vec& fz = freeze.x[i];
        csv << frames[i].t << ',' << rt << ',' << lt << ',' << ht << ',' << c[8] << ',' << c[9]
            << ',' << c[10] << ',' << s[8] << ',' << s[9] << ',' << s[10] << ',' << cerr(c) << ','
            << cerr(s) << ',' << cerr(rp) << ',' << cerr(o) << ',' << cerr(nm) << ',' << cerr(fz)
            << ',' << aerr(c) << ',' << aerr(s) << ',' << aerr(rp) << ',' << aerr(o) << ','
            << aerr(nm) << ',' << aerr(fz) << ',' << yerr(c) << ',' << yerr(s) << ',' << yerr(rp)
            << ',' << yerr(o) << ',' << yerr(nm) << ',' << yerr(fz) << '\n';
    }

    auto line = [](const char* name, const Acc& a) {
        std::cout << name << " center/armor/yaw RMSE mm,mm,deg = "
                  << rmse_from_sse(a.center, a.n) * 1000.0 << " "
                  << rmse_from_sse(a.armor, a.plates) * 1000.0 << " "
                  << rmse_from_sse(a.yaw, a.n) * 180.0 / M_PI
                  << " | after " << kFreezeT << "s "
                  << rmse_from_sse(a.center_late, a.n_late) * 1000.0 << " "
                  << rmse_from_sse(a.armor_late, a.plates_late) * 1000.0 << "\n";
    };
    std::cout << "truth r/l/h = " << rt << " " << lt << " " << ht << "\n";
    std::cout << "causal final r/l/h = " << r_hat << " " << l_hat << " " << h_hat << "\n";
    std::cout << "RTS r/l/h spread around final = " << r_spread << " " << l_spread << " "
              << h_spread << "\n";
    std::cout << "frames " << frames.size() << " obs/id " << id_count[0] << " " << id_count[1] << " "
              << id_count[2] << " " << id_count[3] << "\n";
    line("causal ", ac);
    line("RTS    ", as);
    line("replay ", ar);
    line("oracle ", ao);
    line("nominal", an);
    line("freeze ", af);
    std::cout << "csv: " << csv_path << "\n";
    return 0;
}
