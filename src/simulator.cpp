#include "sim/simulator.hpp"

#include <algorithm>
#include <cmath>

namespace sim {

Simulator::Simulator(RobotGeometry geom, const RobotState& initial,
                     std::unique_ptr<IController> controller, double min_dt)
    : geom_(geom), init_(initial), state_(initial), ctrl_(std::move(controller)),
      min_dt_(std::max(1e-6, min_dt)) {}

RobotTruth Simulator::step(double dt) {
    if (!(dt > 0.0)) return truth();
    return advanceTo(state_.t + dt);
}

RobotTruth Simulator::advanceTo(double t_end) {
    constexpr double kEps = 1e-12;
    if (!(t_end > state_.t + kEps)) return truth();

    while (state_.t < t_end - kEps) {
        if (!has_current_ || remain_ <= kEps) {
            current_ = ctrl_ ? ctrl_->next(state_) : Control{};
            if (ctrl_ && ctrl_->consumeReset()) {
                // plan 循环：物理状态回到初值，但仿真时钟继续走
                const double now = state_.t;
                state_ = init_;
                state_.t = now;
            }
            if (!(current_.duration > 0.0)) current_.duration = t_end - state_.t;  // 防死循环
            remain_ = current_.duration;
            segment_time_ = 0.0;
            has_current_ = true;
        }

        double h = std::min(t_end - state_.t, remain_);
        h = std::min(h, min_dt_);
        if (!(h > 0.0)) h = t_end - state_.t;  // 兜底

        Control local = current_;
        local.accel = current_.accel + current_.jerk * segment_time_;  // 段内加速度演化

        integrate(state_, local, h);
        remain_ -= h;
        segment_time_ += h;
    }
    return truth();
}

RobotTruth Simulator::step(double dt, const Control& ctrl) {
    constexpr double kEps = 1e-12;

    double left = dt;
    double local_t = 0.0;
    while (left > kEps) {
        const double h = std::min(left, min_dt_);
        Control local = ctrl;
        local.accel = ctrl.accel + ctrl.jerk * local_t;
        integrate(state_, local, h);
        left -= h;
        local_t += h;
    }
    return truth();
}

RobotTruth Simulator::truth() const {
    RobotTruth out;

    out.chassis.t = state_.t;
    out.chassis.p = state_.p;
    out.chassis.v = state_.v;
    out.chassis.a = state_.a;
    out.chassis.q = state_.q;
    out.chassis.omega = state_.omega;
    eulerZYX(state_.q, out.chassis.yaw, out.chassis.pitch, out.chassis.roll);

    out.spin_phase = wrapPi(state_.spin_phase);
    out.spin_phase_raw = state_.spin_phase;
    out.spin_rate = state_.spin_rate;

    out.armor_count = std::clamp(geom_.armor_count, 1, kMaxArmors);
    out.armors = makeArmors(geom_, state_.p, state_.q, state_.spin_phase);
    return out;
}

}  // namespace sim
