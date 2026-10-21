#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <Eigen/Geometry>

#include "sim/random.hpp"
#include "sim/types.hpp"

namespace sim {

/// 相机配置。R_world_cam 的列向量是相机三轴在世界系下的方向（列 2 = 光轴）。
struct CameraConfig {
    bool enabled = false;
    Vec3 position = Vec3(3.0, 0.0, 1.5);
    Vec3 target = Vec3(0.0, 0.0, 0.2);
    double fov_half_deg = 35.0;
    bool output_in_camera_frame = false;
    Mat3 R_world_cam = Mat3::Identity();
};

/// 由位置和目标点构造 look-at 旋转（相机 z 轴朝目标）。
Mat3 lookAtRotation(const Vec3& eye, const Vec3& target);

struct SensorConfig {
    std::uint64_t seed = 42;
    Vec3 position_sigma = Vec3::Constant(0.01);  ///< 位置噪声 std（输出坐标系逐轴）
    double normal_sigma = 0.01;                  ///< 法向角噪声 std [rad]
    double dropout = 0.0;
    double max_range = 20.0;
    double facing_min_cos = 0.2;
    CameraConfig camera{};

    // ---- 观测时刻表（与仿真器 min_dt 完全独立）----
    double t0 = 0.0;
    double interval = 0.04;
    double jitter = 0.0;
    std::vector<double> intervals;
};

/// 生成非均匀的观测时刻表：times[k] = t0 + Σ interval_i。
std::vector<double> makeObservationTimes(const SensorConfig& cfg, std::size_t count);

/// 观测模型：位置高斯 + 法向小角度旋转噪声 + 可见性门限 + 独立丢帧。
/// 噪声/丢帧的钥匙是**第几帧**，不是时间，所以帧间隔可以任意不均匀。
class GaussianSensor {
public:
    explicit GaussianSensor(SensorConfig cfg = {});

    [[nodiscard]] const SensorConfig& config() const noexcept { return cfg_; }

    [[nodiscard]] Observation observeAt(const RobotTruth& truth,
                                        double t,
                                        std::uint64_t frame_index,
                                        ObservationTruth* anno = nullptr) const;

    [[nodiscard]] bool inFov(const Vec3& p_world) const;

private:
    SensorConfig cfg_;
};

}  // namespace sim
