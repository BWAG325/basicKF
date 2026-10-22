#include "sim/sensing/sensor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace sim {

namespace {
constexpr std::uint64_t kChPos = 1;
constexpr std::uint64_t kChNormal = 2;
constexpr std::uint64_t kChDrop = 3;
constexpr std::uint64_t kChInterval = 4;
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

Observation GaussianSensor::observeAt(const RobotTruth& truth, double t, std::uint64_t frame_index,
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
    double best_facing = -2.0;
    double best_range = 0.0;
    bool any_in_fov = false;
    bool any_in_range = false;

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

        // 板法向要朝向相机：n·(cam - p) > 0
        const double facing = a.n.dot(d / range);
        if (facing < cfg_.facing_min_cos) continue;

        if (facing > best_facing) {
            best_facing = facing;
            best = i;
            best_range = range;
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

    // 位置噪声
    Rng pos_rng(mixKey(cfg_.seed, frame_index, kChPos));
    const Vec3 noise(pos_rng.gauss() * cfg_.position_sigma.x(),
                     pos_rng.gauss() * cfg_.position_sigma.y(),
                     pos_rng.gauss() * cfg_.position_sigma.z());

    // 法向噪声：小角度旋转矢量 δ 作用在 n 上（保证仍是单位向量）
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

}  // namespace sim
