#pragma once

#include <array>
#include <cstdint>
#include <string_view>

#include <Eigen/Core>

#include "sim/angle.hpp"

namespace sim {

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;

inline constexpr int kMaxArmors = 4;

// ---------------------------------------------------------------------------
// 自由度状态（三个正交自由度，各自独立的时间线）
// ---------------------------------------------------------------------------

/// 平移自由度，世界系，单位 m / (m/s) / (m/s²)。
struct TranslationState {
    Vec3 p = Vec3::Zero();
    Vec3 v = Vec3::Zero();
    Vec3 a = Vec3::Zero();
};

/// 车体朝向，绕世界 +z 轴，弧度。
struct YawState {
    double yaw = 0.0;
    double rate = 0.0;
    double acc = 0.0;
};

/// 装甲板相对车体的自转相位。
/// phase 保持连续累加（不折返），只在对外输出时归一化，避免跨 2π 时数值跳变。
struct PhaseState {
    double phase = 0.0;
    double omega = 0.0;
    double alpha = 0.0;
};

// ---------------------------------------------------------------------------
// 几何 / 真值
// ---------------------------------------------------------------------------

struct RobotGeometry {
    int armor_count = 4;
    double armor_radius = 0.22;  ///< 板中心到旋转轴的水平距离 [m]
    double armor_dz = 0.0;       ///< 板中心相对 chassis.p.z 的高度差 [m]
    double armor_width = 0.13;   ///< 板宽 [m]（预留给后续 PnP / 可视化）
    double armor_height = 0.12;  ///< 板高 [m]
    double body_radius = 0.30;   ///< 车体等效半径 [m]
};

struct ArmorTruth {
    int index = -1;
    Vec3 p = Vec3::Zero();                  ///< 板中心，世界系
    Vec3 n = Vec3(1.0, 0.0, 0.0);           ///< 单位外法向
    double yaw = 0.0;                        ///< 法向对应 yaw，折到 (-π, π]
    double width = 0.0;
    double height = 0.0;
};

struct ChassisState {
    double t = 0.0;
    Vec3 p = Vec3::Zero();
    Vec3 v = Vec3::Zero();
    Vec3 a = Vec3::Zero();
    double yaw = 0.0;
    double yaw_rate = 0.0;
    double yaw_acc = 0.0;
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
    Dropped,        ///< 本帧被丢弃（模拟检测失败 / 丢帧）
    OutOfRange,     ///< 超出量程
    OutOfFov,       ///< 完全不在相机视场内
    NoVisibleArmor, ///< 在视场内，但没有装甲板法向朝向相机
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

/// 滤波器唯一应该看到的观测。
/// 注意：**故意不包含装甲板编号**——真实 PnP 拿不到这个信息，
/// 上层必须靠 yaw 在 2π/n 的等价类里做关联。
struct Observation {
    double t = 0.0;
    bool valid = false;
    InvalidReason reason = InvalidReason::None;
    Vec3 p = Vec3::Zero();        ///< 板中心，输出坐标系（默认世界系）
    double yaw = 0.0;             ///< 法向 yaw（世界系，与 p 的坐标系无关）
    Mat3 R_pos = Mat3::Identity();
    double R_yaw = 0.0;
    bool in_camera_frame = false;
};

/// 仅用于评估 / 调试的旁注。**滤波器禁止读取**。
struct ObservationTruth {
    int armor_index = -1;
    double facing_cos = 0.0;  ///< 板法向与指向相机方向的余弦
    double range = 0.0;
    bool in_fov = false;
};

}  // namespace sim
