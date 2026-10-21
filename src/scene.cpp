#include "sim/scene.hpp"

#include <algorithm>
#include <optional>
#include <vector>

namespace sim {

double Scene::duration() const {
    std::optional<double> shortest;
    const auto take = [&shortest](double d) {
        shortest = shortest ? std::min(*shortest, d) : d;
    };
    if (!trans_.empty()) take(trans_.duration());
    if (!yaw_.empty()) take(yaw_.duration());
    if (!spin_.empty()) take(spin_.duration());
    return shortest.value_or(0.0);
}

RobotTruth Scene::truthAt(double t) const {
    const double tc = std::clamp(t, 0.0, duration());

    // 三条线现场折叠：各自从初值推到 t 所在段，再解析求值。
    const TranslationState tr = trans_.at(tc);
    const YawState ys = yaw_.at(tc);
    const PhaseState sp = spin_.at(tc);

    // 装甲环总转角：两种语义都等于 yaw + phase，只有 chassis 的语义不同。
    const double ring = ys.yaw + sp.phase;

    RobotTruth out;
    out.chassis.t = tc;
    out.chassis.p = tr.p;
    out.chassis.v = tr.v;
    out.chassis.a = tr.a;

    if (semantics_ == SpinSemantics::BodyRotation) {
        out.chassis.yaw = wrapPi(ring);
        out.chassis.yaw_rate = ys.rate + sp.omega;
        out.chassis.yaw_acc = ys.acc + sp.alpha;
    } else {
        out.chassis.yaw = wrapPi(ys.yaw);
        out.chassis.yaw_rate = ys.rate;
        out.chassis.yaw_acc = ys.acc;
    }

    out.spin_phase = wrapPi(sp.phase);
    out.spin_phase_raw = sp.phase;
    out.spin_rate = sp.omega;

    const int n = std::clamp(geom_.armor_count, 1, kMaxArmors);
    out.armor_count = n;
    out.armors = makeArmors(geom_, tr.p, ring);
    return out;
}

bool Scene::Validation::anyWarn() const {
    const auto any = [](const std::vector<JunctionReport>& v) {
        return std::any_of(v.begin(), v.end(), [](const JunctionReport& r) { return r.warn; });
    };
    return any(translation) || any(yaw) || any(spin);
}

Scene::Validation Scene::validate(double eps) const {
    Validation out;
    if (!trans_.empty()) out.translation = trans_.validate(eps);
    if (!yaw_.empty()) out.yaw = yaw_.validate(eps);
    if (!spin_.empty()) out.spin = spin_.validate(eps);
    return out;
}

}  // namespace sim
