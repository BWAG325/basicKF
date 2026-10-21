#pragma once

#include <cstdint>

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
    double dt = 0.01;                        ///< 采样周期 [s]
    std::uint64_t seed = 42;                 ///< 随机流种子
    Vec3 position_sigma = Vec3::Constant(0.01);  ///< 位置噪声 std，**输出坐标系**下逐轴
    double yaw_sigma = 0.01;                 ///< 法向 yaw 噪声 std [rad]
    double dropout = 0.0;                    ///< 每帧丢帧概率 [0,1]
    double max_range = 20.0;                 ///< 量程 [m]
    double facing_min_cos = 0.2;             ///< 法向与「指向相机」方向夹角余弦门限
    CameraConfig camera{};
};

/// 观测模型：目前是「沿轴独立高斯 + 硬门限可见性 + 独立丢帧」。
/// 噪声的钥匙是 (seed, sample_index, channel)，与求值顺序完全无关——
/// 这是「任意 t 求值」能成立的前提。
class GaussianSensor {
public:
    explicit GaussianSensor(SensorConfig cfg = {});

    [[nodiscard]] const SensorConfig& config() const noexcept { return cfg_; }

    /// t 所属采样序号；dt<=0 时恒为 0。
    [[nodiscard]] std::uint64_t sampleIndex(double t) const noexcept;
    [[nodiscard]] double sampleTime(double t) const noexcept;

    /// 该序号是否被丢弃（由 hash 决定，不受调用顺序影响）。
    [[nodiscard]] bool dropped(std::uint64_t index) const;

    /// t 是否恰好落在采样时刻且未被丢弃（供「逐帧驱动」使用）。
    [[nodiscard]] bool triggeredAt(double t, double eps = 1e-9) const;

    /// 给定真值，产生 t 时刻的观测。anno 非空时写入仅供评估的旁注。
    [[nodiscard]] Observation observeAt(const RobotTruth& truth,
                                        double t,
                                        ObservationTruth* anno = nullptr) const;

    /// 世界系点是否落在相机视场内。
    [[nodiscard]] bool inFov(const Vec3& p_world) const;

private:
    SensorConfig cfg_;
};

}  // namespace sim
