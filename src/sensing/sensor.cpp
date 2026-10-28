#include "sim/sensing/sensor.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <utility>
#include <vector>

#include <Eigen/Eigenvalues>
#include <opencv2/calib3d.hpp>

namespace sim {

namespace {
constexpr std::uint64_t kChPos = 1;
constexpr std::uint64_t kChNormal = 2;
constexpr std::uint64_t kChDrop = 3;
constexpr std::uint64_t kChInterval = 4;
constexpr std::uint64_t kChPnp = 5;

double wrapToPi(double a) { return std::atan2(std::sin(a), std::cos(a)); }

struct PoseReading {
    Eigen::Vector4d z = Eigen::Vector4d::Zero();
    Vec3 p = Vec3::Zero();
    Vec3 n = Vec3::UnitX();
};

PoseReading readingFromPose(const Mat3& R_world_cam, const Vec3& eye, const cv::Vec3d& rvec,
                            const cv::Vec3d& tvec) {
    cv::Mat rmat;
    cv::Rodrigues(rvec, rmat);
    Mat3 R_ac;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) R_ac(r, c) = rmat.at<double>(r, c);
    }
    R_ac.row(1) *= -1.0;
    const Mat3 R_aw = R_world_cam * R_ac;
    const Vec3 n = R_aw.col(0).normalized();
    const Vec3 p = eye + R_world_cam * Vec3(tvec[0], -tvec[1], tvec[2]);
    PoseReading o;
    o.p = p;
    o.n = n;
    const double horiz = std::hypot(p.x(), p.y());
    o.z[0] = std::atan2(p.y(), p.x());
    o.z[1] = std::atan2(p.z(), horiz);
    o.z[2] = p.norm();
    o.z[3] = std::atan2(-n.y(), -n.x());
    return o;
}
}  // namespace

Mat3 lookAtRotation(const Vec3& eye, const Vec3& target) {
    Vec3 z = target - eye;
    if (z.norm() < 1e-9) {
        z = Vec3(0.0, 0.0, -1.0);
    } else {
        z.normalize();
    }

    Vec3 up(0.0, 0.0, 1.0);
    if (up.cross(z).norm() < 1e-6) up = Vec3(0.0, 1.0, 0.0);

    const Vec3 x = up.cross(z).normalized();
    const Vec3 y = z.cross(x);

    Mat3 R;
    R.col(0) = x;
    R.col(1) = y;
    R.col(2) = z;
    return R;
}

std::vector<double> makeObservationTimes(const SensorConfig& cfg, std::size_t count) {
    std::vector<double> times;
    times.reserve(count);

    double t = cfg.t0;
    for (std::size_t k = 0; k < count; ++k) {
        times.push_back(t);

        double step = cfg.interval;
        if (!cfg.intervals.empty()) {
            step = cfg.intervals[k % cfg.intervals.size()];
        } else {
            Rng rng(mixKey(cfg.seed, k, kChInterval));
            step += cfg.jitter * (2.0 * rng.uniform() - 1.0);
        }
        t += std::max(step, 1e-6);
    }
    return times;
}

GaussianSensor::GaussianSensor(SensorConfig cfg) : cfg_(std::move(cfg)) {
    if (cfg_.camera.enabled) {
        cfg_.camera.R_world_cam = lookAtRotation(cfg_.camera.position, cfg_.camera.target);
    }
}

bool GaussianSensor::inFov(const Vec3& p_world) const {
    if (!cfg_.camera.enabled) return true;
    const Vec3 pc = cfg_.camera.R_world_cam.transpose() * (p_world - cfg_.camera.position);
    if (pc.z() <= 1e-6) return false;
    const double ang = std::atan2(std::hypot(pc.x(), pc.y()), pc.z());
    return ang <= cfg_.camera.fov_half_deg * kPi / 180.0;
}

int GaussianSensor::pickArmor(const RobotTruth& truth) const {
    if (!cfg_.camera.enabled) return truth.armor_count > 0 ? 0 : -1;

    const int count = std::clamp(truth.armor_count, 0, kMaxArmors);
    int best = -1;
    double best_facing = -2.0;

    for (int i = 0; i < count; ++i) {
        const ArmorTruth& a = truth.armors[static_cast<std::size_t>(i)];
        const Vec3 d = cfg_.camera.position - a.p;
        const double range = d.norm();
        if (range > cfg_.max_range) continue;
        if (!inFov(a.p)) continue;

        const double facing = a.n.dot(d / range);
        if (facing < cfg_.facing_min_cos) continue;
        if (facing > best_facing) {
            best_facing = facing;
            best = i;
        }
    }
    return best;
}

Observation GaussianSensor::observeAt(const RobotTruth& truth, double t, std::uint64_t frame_index,
                                      ObservationTruth* anno) const {
    return observeArmorAt(truth, t, frame_index, -1, anno);
}

Observation GaussianSensor::observeArmorAt(const RobotTruth& truth, double t,
                                           std::uint64_t frame_index, int armor_index,
                                           ObservationTruth* anno) const {
    Observation obs;
    obs.t = t;
    obs.in_camera_frame = cfg_.camera.enabled && cfg_.camera.output_in_camera_frame;

    Rng drop_rng(mixKey(cfg_.seed, frame_index, kChDrop));
    if (cfg_.dropout > 0.0 && drop_rng.uniform() < cfg_.dropout) {
        obs.reason = InvalidReason::Dropped;
        return obs;
    }

    const int count = std::clamp(truth.armor_count, 0, kMaxArmors);

    int best = -1;
    double best_facing = 0.0;
    double best_range = 0.0;
    bool any_in_fov = false;
    bool any_in_range = false;

    if (armor_index >= 0) {
        // 锁定模式：只检查距离/FOV，不要求板面朝向相机
        if (armor_index >= count) {
            obs.reason = InvalidReason::NoVisibleArmor;
            return obs;
        }
        best = armor_index;
        const ArmorTruth& a = truth.armors[static_cast<std::size_t>(best)];
        const Vec3 d = cfg_.camera.position - a.p;
        best_range = d.norm();
        if (cfg_.camera.enabled) {
            if (best_range > cfg_.max_range) {
                obs.reason = InvalidReason::OutOfRange;
                return obs;
            }
            any_in_range = true;
            if (!inFov(a.p)) {
                obs.reason = InvalidReason::OutOfFov;
                return obs;
            }
            any_in_fov = true;
            best_facing = a.n.dot(d / best_range);
        }
    } else {
        for (int i = 0; i < count; ++i) {
            const ArmorTruth& a = truth.armors[static_cast<std::size_t>(i)];

            if (!cfg_.camera.enabled) {
                best = 0;
                best_facing = 0.0;
                best_range = 0.0;
                break;
            }

            const Vec3 d = cfg_.camera.position - a.p;
            const double range = d.norm();
            if (range > cfg_.max_range) continue;
            any_in_range = true;

            if (!inFov(a.p)) continue;
            any_in_fov = true;

            const double facing = a.n.dot(d / range);
            if (facing < cfg_.facing_min_cos) continue;

            if (facing > best_facing) {
                best_facing = facing;
                best = i;
                best_range = range;
            }
        }
    }

    if (best < 0) {
        if (!any_in_range) {
            obs.reason = InvalidReason::OutOfRange;
        } else if (!any_in_fov) {
            obs.reason = InvalidReason::OutOfFov;
        } else {
            obs.reason = InvalidReason::NoVisibleArmor;
        }
        return obs;
    }

    const ArmorTruth& a = truth.armors[static_cast<std::size_t>(best)];

    if (cfg_.pnp.enabled) {
        return observePnp(a, t, frame_index, best, best_facing, best_range,
                          any_in_fov || !cfg_.camera.enabled, anno);
    }

    Rng pos_rng(mixKey(cfg_.seed, frame_index, kChPos));
    const Vec3 noise(pos_rng.gauss() * cfg_.position_sigma.x(),
                     pos_rng.gauss() * cfg_.position_sigma.y(),
                     pos_rng.gauss() * cfg_.position_sigma.z());

    Rng n_rng(mixKey(cfg_.seed, frame_index, kChNormal));
    const Vec3 delta(n_rng.gauss() * cfg_.normal_sigma,
                     n_rng.gauss() * cfg_.normal_sigma,
                     n_rng.gauss() * cfg_.normal_sigma);
    obs.n = (a.n + delta.cross(a.n)).normalized();
    obs.R_n = cfg_.normal_sigma * cfg_.normal_sigma;

    obs.R_pos = Mat3::Zero();
    obs.R_pos.diagonal() = cfg_.position_sigma.cwiseProduct(cfg_.position_sigma);

    if (obs.in_camera_frame) {
        const Mat3 R_cw = cfg_.camera.R_world_cam.transpose();
        obs.p = R_cw * (a.p - cfg_.camera.position) + noise;
        obs.n = (R_cw * obs.n).normalized();
    } else {
        obs.p = a.p + noise;
    }

    obs.valid = true;
    obs.reason = InvalidReason::None;

    if (anno) {
        anno->armor_index = best;
        anno->facing_cos = best_facing;
        anno->range = best_range;
        anno->in_fov = any_in_fov || !cfg_.camera.enabled;
    }
    return obs;
}

Observation GaussianSensor::observePnp(const ArmorTruth& armor, double t, std::uint64_t frame_index,
                                       int armor_index, double facing, double range, bool in_fov,
                                       ObservationTruth* anno) const {
    Observation obs;
    obs.t = t;
    if (!cfg_.camera.enabled) {
        obs.reason = InvalidReason::OutOfFov;
        return obs;
    }

    const PnpConfig& pnp = cfg_.pnp;
    const double w = pnp.armor_width;
    const double h = pnp.armor_height;
    const std::vector<cv::Point3f> obj{
        {0.f, static_cast<float>(w / 2), static_cast<float>(h / 2)},
        {0.f, static_cast<float>(-w / 2), static_cast<float>(h / 2)},
        {0.f, static_cast<float>(-w / 2), static_cast<float>(-h / 2)},
        {0.f, static_cast<float>(w / 2), static_cast<float>(-h / 2)},
    };

    const Mat3 R_wc = cfg_.camera.R_world_cam;
    const Vec3 eye = cfg_.camera.position;
    const Mat3 R_aw = armor.q.toRotationMatrix();
    Mat3 R_ac = R_wc.transpose() * R_aw;
    R_ac.row(1) *= -1.0;
    const Vec3 t_cv = [&] {
        const Vec3 ps = R_wc.transpose() * (armor.p - eye);
        return Vec3(ps.x(), -ps.y(), ps.z());
    }();
    if (t_cv.z() < 0.05) {
        obs.reason = InvalidReason::OutOfFov;
        return obs;
    }

    cv::Mat Rcv(3, 3, CV_64F);
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) Rcv.at<double>(r, c) = R_ac(r, c);
    }
    cv::Vec3d rvec_true;
    cv::Rodrigues(Rcv, rvec_true);
    const cv::Vec3d tvec_true(t_cv.x(), t_cv.y(), t_cv.z());
    const cv::Mat K = (cv::Mat_<double>(3, 3) << pnp.fx, 0.0, pnp.cx, 0.0, pnp.fy, pnp.cy, 0.0, 0.0, 1.0);
    const cv::Mat dist = (cv::Mat_<double>(1, 5) << pnp.k1, pnp.k2, pnp.p1, pnp.p2, pnp.k3);

    std::vector<cv::Point2f> pixels;
    cv::projectPoints(obj, rvec_true, tvec_true, K, dist, pixels);
    Rng rng(mixKey(cfg_.seed, frame_index, kChPnp));
    for (auto& px : pixels) {
        px.x += static_cast<float>(rng.gauss() * pnp.pixel_sigma);
        px.y += static_cast<float>(rng.gauss() * pnp.pixel_sigma);
    }

    std::vector<cv::Mat> rvecs, tvecs;
    cv::Mat reproj;
    const int nsol = cv::solvePnPGeneric(obj, pixels, K, dist, rvecs, tvecs, false, cv::SOLVEPNP_IPPE,
                                         cv::noArray(), cv::noArray(), reproj);
    int best = -1;
    double best_err = 1e300;
    cv::Vec3d rvec, tvec;
    for (int i = 0; i < nsol; ++i) {
        cv::Vec3d ri, ti;
        if (rvecs[static_cast<std::size_t>(i)].rows == 3) {
            ri = cv::Vec3d(rvecs[static_cast<std::size_t>(i)].at<double>(0),
                           rvecs[static_cast<std::size_t>(i)].at<double>(1),
                           rvecs[static_cast<std::size_t>(i)].at<double>(2));
            ti = cv::Vec3d(tvecs[static_cast<std::size_t>(i)].at<double>(0),
                           tvecs[static_cast<std::size_t>(i)].at<double>(1),
                           tvecs[static_cast<std::size_t>(i)].at<double>(2));
        } else {
            continue;
        }
        if (ti[2] < 0.05) continue;
        const double err = reproj.empty()
                               ? 0.0
                               : (reproj.type() == CV_32F ? static_cast<double>(reproj.at<float>(i))
                                                          : reproj.at<double>(i));
        if (err < best_err) {
            best_err = err;
            best = i;
            rvec = ri;
            tvec = ti;
        }
    }
    if (best < 0) {
        obs.reason = InvalidReason::NoVisibleArmor;
        return obs;
    }

    const PoseReading solved = readingFromPose(R_wc, eye, rvec, tvec);
    obs.p = solved.p;
    obs.n = solved.n;
    obs.valid = true;
    obs.reason = InvalidReason::None;
    obs.R_n = 0.0;
    obs.R_pos.setZero();

    cv::Mat jac;
    std::vector<cv::Point2f> projected;
    cv::projectPoints(obj, rvec, tvec, K, dist, projected, jac);
    cv::Mat jac64;
    jac.convertTo(jac64, CV_64F);
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(8, 6);
    if (jac64.rows >= 8 && jac64.cols >= 6) {
        for (int r = 0; r < 8; ++r) {
            for (int c = 0; c < 6; ++c) J(r, c) = jac64.at<double>(r, c);
        }
    }
    Eigen::Matrix<double, 6, 6> JtJ = J.transpose() * J;
    JtJ.diagonal().array() += 1e-9;
    const double pix_var = pnp.pixel_sigma * pnp.pixel_sigma;
    const Eigen::Matrix<double, 6, 6> Sigma = pix_var * JtJ.ldlt().solve(Eigen::Matrix<double, 6, 6>::Identity());

    Eigen::Matrix<double, 4, 6> G;
    const double er = 1e-6;
    const double et = 1e-5;
    for (int k = 0; k < 3; ++k) {
        cv::Vec3d rp = rvec;
        rp[k] += er;
        const Eigen::Vector4d dz = readingFromPose(R_wc, eye, rp, tvec).z - solved.z;
        Eigen::Vector4d d = dz;
        d[0] = wrapToPi(dz[0]);
        d[1] = wrapToPi(dz[1]);
        d[3] = wrapToPi(dz[3]);
        G.col(k) = d / er;
        cv::Vec3d tp = tvec;
        tp[k] += et;
        const Eigen::Vector4d dzt = readingFromPose(R_wc, eye, rvec, tp).z - solved.z;
        Eigen::Vector4d dtv = dzt;
        dtv[0] = wrapToPi(dzt[0]);
        dtv[1] = wrapToPi(dzt[1]);
        dtv[3] = wrapToPi(dzt[3]);
        G.col(3 + k) = dtv / et;
    }
    Eigen::Matrix4d R = G * Sigma * G.transpose();
    R = 0.5 * (R + R.transpose());
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> es(R);
    if (es.info() == Eigen::Success) {
        Eigen::Vector4d ev = es.eigenvalues();
        for (int i = 0; i < 4; ++i) ev[i] = std::clamp(ev[i], 1e-12, 100.0);
        R = es.eigenvectors() * ev.asDiagonal() * es.eigenvectors().transpose();
    } else {
        R = Eigen::Vector4d(1e-4, 1e-4, 1e-2, 1e-3).asDiagonal();
    }
    obs.R_ypd = R;
    obs.has_ypd_cov = true;
    if (pnp.pixel_sigma == 0.0 && frame_index == 0) {
        double reproj_px = 0.0;
        for (int i = 0; i < 4; ++i) reproj_px += cv::norm(projected[static_cast<std::size_t>(i)] - pixels[static_cast<std::size_t>(i)]);
        cv::Mat Rm;
        cv::Rodrigues(rvec, Rm);
        Mat3 Rac;
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c) Rac(r, c) = Rm.at<double>(r, c);
        auto world_of = [&](const Vec3& pobj) {
            const Vec3 pc = Rac * pobj + Vec3(tvec[0], tvec[1], tvec[2]);
            return eye + R_wc * Vec3(pc.x(), -pc.y(), pc.z());
        };
        const Vec3 n_pts = (world_of(Vec3(1, 0, 0)) - world_of(Vec3::Zero())).normalized();
        const Vec3 y_pts = (world_of(Vec3(0, 1, 0)) - world_of(Vec3::Zero())).normalized();
        const Vec3 y_true = (armor.q * Vec3(0, 1, 0)).normalized();
        std::cerr << "reproj_px " << reproj_px << " Rm " << Rm.rows << "x" << Rm.cols
                  << " n_pts " << n_pts.transpose() << " ydot " << y_pts.dot(y_true)
                  << " Rac\n" << Rac << "\n";
    }

    if (anno) {
        anno->armor_index = armor_index;
        anno->facing_cos = facing;
        anno->range = range;
        anno->in_fov = in_fov;
    }
    return obs;
}

}  // namespace sim
