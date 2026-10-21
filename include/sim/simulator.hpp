#pragma once

#include <memory>

#include "sim/controller.hpp"
#include "sim/integrator.hpp"
#include "sim/robot_model.hpp"
#include "sim/types.hpp"

namespace sim {

/// 增量式 3D 运动仿真器：持有当前状态，每次 step/advanceTo 用控制量往前推进。
///
/// - 姿态是四元数（body→world），角速度在 body 系；
/// - 速度沿车体 +x（无侧滑）；
/// - `min_dt` 是**积分步长上限**：`advanceTo` 会按它把时间切成小步，
///   所以位置积分精度由它决定，而请求的 dt 大小不影响结果。
class Simulator {
public:
    Simulator() = default;
    Simulator(RobotGeometry geom, const RobotState& initial,
              std::unique_ptr<IController> controller = nullptr, double min_dt = 0.001);

    void setController(std::unique_ptr<IController> c) { ctrl_ = std::move(c); }
    void reset() {
        if (ctrl_) ctrl_->reset();
        state_ = init_;
        current_ = Control{};
        remain_ = 0.0;
        segment_time_ = 0.0;
        has_current_ = false;
    }

    /// 推进 dt（内部按 min_dt 与当前控制剩余时长切分）。
    RobotTruth step(double dt);
    /// 显式控制推进 dt：不消耗计划队列，忽略 control.duration（键盘用这个）。
    /// 语义：这是一段**新的**控制，加速度从 control.accel 重新开始。
    RobotTruth step(double dt, const Control& ctrl);
    /// 推进到绝对时刻 t_end（观测时刻表就是靠它对齐的）。
    RobotTruth advanceTo(double t_end);

    void setMinDt(double dt) noexcept { min_dt_ = dt; }
    [[nodiscard]] double minDt() const noexcept { return min_dt_; }

    [[nodiscard]] const RobotState& state() const { return state_; }
    [[nodiscard]] double time() const { return state_.t; }
    [[nodiscard]] RobotTruth truth() const;

    [[nodiscard]] const RobotGeometry& geometry() const { return geom_; }

private:
    RobotGeometry geom_{};

    RobotState init_{};
    RobotState state_{};

    std::unique_ptr<IController> ctrl_;
    Control current_{};
    double remain_ = 0.0;
    double segment_time_ = 0.0;
    bool has_current_ = false;
    double min_dt_ = 0.001;
};

}  // namespace sim
