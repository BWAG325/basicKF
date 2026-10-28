#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <Eigen/Geometry>

#include "sim/core/random.hpp"
#include "sim/core/types.hpp"

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

/// auto_aim Solver 的小装甲物点与哨兵相机内参。enabled 为 false 时观测仍是中心高斯。
struct PnpConfig {
    bool enabled = false;
    double pixel_sigma = 1.0;  ///< 每个角点像素的独立标准差
    double fx = 1818.3669452465165;
    double fy = 1822.494494078506;
    double cx = 751.062265747035;
    double cy = 530.4367155611213;
    double k1 = -0.077944626599568856;
    double k2 = 0.15447826031486889;
    double p1 = -0.0025714394278524674;
    double p2 = 0.00083016311301273629;
    double k3 = 0.0;
    double armor_width = 0.135;   ///< 与 SMALL_ARMOR_WIDTH 一致
    double armor_height = 0.056;  ///< 与 LIGHTBAR_LENGTH 一致
};

struct SensorConfig {
    std::uint64_t seed = 42;
    Vec3 position_sigma = Vec3::Constant(0.01);  ///< 位置噪声 std（输出坐标系逐轴）
    double normal_sigma = 0.01;                  ///< 法向角噪声 std [rad]
    double dropout = 0.0;
    PnpConfig pnp{};
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

    /// 强制观测指定装甲板（用于“锁定一块板”的滤波对比）：
    /// 跳过朝向门限，但仍检查距离/FOV/丢帧。
    [[nodiscard]] Observation observeArmorAt(const RobotTruth& truth,
                                             double t,
                                             std::uint64_t frame_index,
                                             int armor_index,
                                             ObservationTruth* anno = nullptr) const;

    /// 当前帧最适合观测的装甲板下标（没有则 -1）。
    [[nodiscard]] int pickArmor(const RobotTruth& truth) const;

    [[nodiscard]] bool inFov(const Vec3& p_world) const;

private:
    SensorConfig cfg_;

    [[nodiscard]] Observation observePnp(const ArmorTruth& armor, double t,
                                         std::uint64_t frame_index, int armor_index,
                                         double facing, double range, bool in_fov,
                                         ObservationTruth* anno) const;
};

}  // namespace sim
