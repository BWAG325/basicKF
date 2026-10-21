#include "sim/sensor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace sim {

namespace {
constexpr std::uint64_t kChPos = 1;
constexpr std::uint64_t kChYaw = 2;
constexpr std::uint64_t kChDrop = 3;
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

GaussianSensor::GaussianSensor(SensorConfig cfg) : cfg_(std::move(cfg)) {
    if (cfg_.camera.enabled) {
        // 相机朝向永远由 position/target 决定，避免配置里两处旋转打架。
        cfg_.camera.R_world_cam = lookAtRotation(cfg_.camera.position, cfg_.camera.target);
    }
}

std::uint64_t GaussianSensor::sampleIndex(double t) const noexcept {
    if (!(cfg_.dt > 0.0)) return 0;
    const double k = std::llround(t / cfg_.dt);
    return k < 0.0 ? 0u : static_cast<std::uint64_t>(k);
}

double GaussianSensor::sampleTime(double t) const noexcept {
    return static_cast<double>(sampleIndex(t)) * cfg_.dt;
}

bool GaussianSensor::dropped(std::uint64_t index) const {
    if (cfg_.dropout <= 0.0) return false;
    Rng rng(mixKey(cfg_.seed, index, kChDrop));
    return rng.uniform() < cfg_.dropout;
}

bool GaussianSensor::triggeredAt(double t, double eps) const {
    if (!(cfg_.dt > 0.0)) return false;
    const std::uint64_t idx = sampleIndex(t);
    if (std::abs(t - static_cast<double>(idx) * cfg_.dt) > eps) return false;
    return !dropped(idx);
}

bool GaussianSensor::inFov(const Vec3& p_world) const {
    if (!cfg_.camera.enabled) return true;
    const Vec3 pc = cfg_.camera.R_world_cam.transpose() * (p_world - cfg_.camera.position);
    if (pc.z() <= 1e-6) return false;
    const double ang = std::atan2(std::hypot(pc.x(), pc.y()), pc.z());
    return ang <= cfg_.camera.fov_half_deg * kPi / 180.0;
}

Observation GaussianSensor::observeAt(const RobotTruth& truth, double t, ObservationTruth* anno) const {
    Observation obs;
    obs.t = t;
    obs.in_camera_frame = cfg_.camera.enabled && cfg_.camera.output_in_camera_frame;

    const std::uint64_t idx = sampleIndex(t);

    if (dropped(idx)) {
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

        if (!cfg_.camera.enabled) {  // 无相机模型：默认第一块板可见
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

    Rng pos_rng(mixKey(cfg_.seed, idx, kChPos));
    const Vec3 noise(pos_rng.gauss() * cfg_.position_sigma.x(),
                     pos_rng.gauss() * cfg_.position_sigma.y(),
                     pos_rng.gauss() * cfg_.position_sigma.z());

    Rng yaw_rng(mixKey(cfg_.seed, idx, kChYaw));
    obs.yaw = wrapPi(a.yaw + yaw_rng.gauss() * cfg_.yaw_sigma);
    obs.R_yaw = cfg_.yaw_sigma * cfg_.yaw_sigma;

    obs.R_pos = Mat3::Zero();
    obs.R_pos.diagonal() = cfg_.position_sigma.cwiseProduct(cfg_.position_sigma);

    if (obs.in_camera_frame) {
        obs.p = cfg_.camera.R_world_cam.transpose() * (a.p - cfg_.camera.position) + noise;
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
