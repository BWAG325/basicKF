#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string_view>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "sim/angle.hpp"

namespace sim {

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
using Quat = Eigen::Quaterniond;

inline constexpr int kMaxArmors = 4;

/// 四元数 → ZYX 欧拉角（yaw / pitch / roll），只用于显示和调试。
inline void eulerZYX(const Quat& q, double& yaw, double& pitch, double& roll) {
    const Mat3 R = q.toRotationMatrix();
    yaw = std::atan2(R(1, 0), R(0, 0));
    pitch = std::asin(std::clamp(-R(2, 0), -1.0, 1.0));
    roll = std::atan2(R(2, 1), R(2, 2));
}

// ---------------------------------------------------------------------------
// 控制量（都在车体系下）
// ---------------------------------------------------------------------------

/// 一段控制量：本段 `duration` 内保持恒定。
/// - `accel`    沿车体 +x 的切向加速度 [m/s²]
/// - `jerk`     沿车体 +x 的加加速度 [m/s³]
/// - `angular_rate` 车体角速度（body 系：x=roll, y=pitch, z=yaw）[rad/s]
/// - `spin_rate` 装甲环相对车体的自转 [rad/s]
struct Control {
    double accel = 0.0;
    double jerk = 0.0;
    Vec3 angular_rate = Vec3::Zero();
    double spin_rate = 0.0;
    double duration = 1.0;
};

// ---------------------------------------------------------------------------
// 状态（仿真器内部持有，逐帧推进）
// ---------------------------------------------------------------------------

/// 速度方向始终沿车体 +x（无侧滑）；姿态用四元数，角速度在 body 系。
struct RobotState {
    double t = 0.0;
    Vec3 p = Vec3::Zero();
    Vec3 v = Vec3::Zero();
    Vec3 a = Vec3::Zero();
    Quat q = Quat::Identity();     ///< body -> world
    Vec3 omega = Vec3::Zero();     ///< body 系角速度 [rad/s]
    double speed = 0.0;            ///< 沿车体 +x 的速率 [m/s]
    double accel = 0.0;            ///< 当前切向加速度 [m/s²]
    double spin_phase = 0.0;       ///< 装甲环相对车体的相位，连续累加
    double spin_rate = 0.0;
};

// ---------------------------------------------------------------------------
// 几何 / 真值
// ---------------------------------------------------------------------------

struct RobotGeometry {
    int armor_count = 4;
    double armor_radius = 0.22;  ///< 板中心到车体 z 轴的水平距离 [m]
    double armor_dz = 0.0;       ///< 板中心相对车体中心的车体 z 偏移 [m]
    double armor_width = 0.13;
    double armor_height = 0.12;
    double body_radius = 0.30;
};

struct ArmorTruth {
    int index = -1;
    Vec3 p = Vec3::Zero();                                ///< 板中心，世界系
    Vec3 n = Vec3(1.0, 0.0, 0.0);                         ///< 外法向，世界系
    Vec3 up = Vec3(0.0, 0.0, 1.0);                        ///< 板「向上」轴，世界系
    Quat q = Quat::Identity();                            ///< 板坐标系 -> 世界
    double yaw = 0.0;                                     ///< 法向的偏航角（便于读）
    double width = 0.0;
    double height = 0.0;
};

struct ChassisState {
    double t = 0.0;
    Vec3 p = Vec3::Zero();
    Vec3 v = Vec3::Zero();
    Vec3 a = Vec3::Zero();
    Quat q = Quat::Identity();
    Vec3 omega = Vec3::Zero();
    double yaw = 0.0;
    double pitch = 0.0;
    double roll = 0.0;
};

struct RobotTruth {
    ChassisState chassis;
    double spin_phase = 0.0;      ///< 折到 (-π, π]
    double spin_phase_raw = 0.0;  ///< 连续累加值
    double spin_rate = 0.0;
    int armor_count = 0;
    std::array<ArmorTruth, kMaxArmors> armors{};
};

// ---------------------------------------------------------------------------
// 观测
// ---------------------------------------------------------------------------

enum class InvalidReason : std::uint8_t {
    None = 0,
    Dropped,
    OutOfRange,
    OutOfFov,
    NoVisibleArmor,
};

inline std::string_view toString(InvalidReason r) noexcept {
    switch (r) {
        case InvalidReason::None: return "none";
        case InvalidReason::Dropped: return "dropped";
        case InvalidReason::OutOfRange: return "out_of_range";
        case InvalidReason::OutOfFov: return "out_of_fov";
        case InvalidReason::NoVisibleArmor: return "no_visible_armor";
    }
    return "unknown";
}

/// 观测：位置 + 法向（三维），不含装甲板编号。
struct Observation {
    double t = 0.0;
    bool valid = false;
    InvalidReason reason = InvalidReason::None;
    Vec3 p = Vec3::Zero();
    Vec3 n = Vec3(1.0, 0.0, 0.0);
    Mat3 R_pos = Mat3::Identity();
    double R_n = 0.0;  ///< 法向角噪声方差 [rad²]
    bool in_camera_frame = false;
};

/// 仅用于评估 / 调试的旁注。**滤波器禁止读取**。
struct ObservationTruth {
    int armor_index = -1;
    double facing_cos = 0.0;
    double range = 0.0;
    bool in_fov = false;
};

}  // namespace sim
