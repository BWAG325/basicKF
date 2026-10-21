#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string_view>
#include <vector>

#include "sim/types.hpp"

namespace sim {

// ---------------------------------------------------------------------------
// 段间连续性
// ---------------------------------------------------------------------------

/// 相邻段之间的状态缺口。逐项报告，不阻断构建。
struct JunctionGap {
    double dp = 0.0;
    double dv = 0.0;
    double dyaw = 0.0;
    double domega = 0.0;

    [[nodiscard]] double worst() const noexcept { return std::max({dp, dv, dyaw, domega}); }
};

inline JunctionGap junctionGap(const TranslationState& prev, const TranslationState& next) {
    return {(prev.p - next.p).norm(), (prev.v - next.v).norm(), 0.0, 0.0};
}

inline JunctionGap junctionGap(const YawState& prev, const YawState& next) {
    return {0.0, 0.0, std::abs(angleDiff(prev.yaw, next.yaw)), std::abs(prev.rate - next.rate)};
}

inline JunctionGap junctionGap(const PhaseState& prev, const PhaseState& next) {
    return {0.0, 0.0, std::abs(angleDiff(prev.phase, next.phase)), std::abs(prev.omega - next.omega)};
}

/// 段首显式跳变（比如被击中瞬间掉速）的幅值。
inline double jumpMagnitude(const TranslationState& d) { return std::max(d.p.norm(), d.v.norm()); }
inline double jumpMagnitude(const YawState& d) { return std::max(std::abs(d.yaw), std::abs(d.rate)); }
inline double jumpMagnitude(const PhaseState& d) { return std::max(std::abs(d.phase), std::abs(d.omega)); }

inline TranslationState addDelta(const TranslationState& s, const TranslationState& d) {
    return {s.p + d.p, s.v + d.v, s.a + d.a};
}
inline YawState addDelta(const YawState& s, const YawState& d) {
    return {s.yaw + d.yaw, s.rate + d.rate, s.acc + d.acc};
}
inline PhaseState addDelta(const PhaseState& s, const PhaseState& d) {
    return {s.phase + d.phase, s.omega + d.omega, s.alpha + d.alpha};
}

struct JunctionReport {
    std::size_t segment = 0;
    double t = 0.0;
    JunctionGap entry_gap{};
    JunctionGap internal_gap{};
    bool declared_jump = false;
    bool warn = false;
};

// ---------------------------------------------------------------------------
// 分段解析时间线（查询时计算，构建期零运算）
// ---------------------------------------------------------------------------

/// 只登记段（时长 / jump / primitive）；**构建期不推导任何状态**。
/// at(t) 现场从初值折叠到 t 所在段：每段一次解析求值，复杂度 O(t 之前的段数)。
/// 没有缓存、没有累加器、没有可变状态，所以调用顺序不影响结果。
template <class Prim, class State>
class Timeline {
public:
    struct Entry {
        std::unique_ptr<Prim> prim;
        State jump{};
        double t0 = 0.0;
        double dur = 0.0;
    };

    Timeline() = default;
    Timeline(const Timeline&) = delete;
    Timeline& operator=(const Timeline&) = delete;
    Timeline(Timeline&&) noexcept = default;
    Timeline& operator=(Timeline&&) noexcept = default;
    ~Timeline() = default;

    Timeline& add(std::unique_ptr<Prim> prim, State jump = State{}) {
        if (!prim) throw std::invalid_argument("Timeline::add: null primitive");
        const double d = prim->duration();
        if (!(d >= 0.0) || !std::isfinite(d)) {
            throw std::invalid_argument("Timeline::add: invalid duration");
        }
        Entry e;
        e.prim = std::move(prim);
        e.jump = jump;
        e.t0 = total_;
        e.dur = d;
        total_ += d;
        entries_.push_back(std::move(e));
        return *this;
    }

    void setInitial(const State& s) noexcept { initial_ = s; }
    [[nodiscard]] const State& initial() const noexcept { return initial_; }

    [[nodiscard]] bool empty() const noexcept { return entries_.empty(); }
    [[nodiscard]] double duration() const noexcept { return total_; }
    [[nodiscard]] const std::vector<Entry>& entries() const noexcept { return entries_; }

    /// 现场计算 t 时刻的状态：从初值开始，把 t 所在段之前的段逐段推过去，再解析求本段。
    [[nodiscard]] State at(double t) const {
        if (entries_.empty()) return initial_;

        const double tc = std::clamp(t, 0.0, total_);
        const std::size_t i = locate(tc);

        State s = initial_;
        for (std::size_t k = 0; k < i; ++k) {
            s = addDelta(s, entries_[k].jump);
            s = entries_[k].prim->eval(s, entries_[k].dur);
        }
        s = addDelta(s, entries_[i].jump);
        return entries_[i].prim->eval(s, std::clamp(tc - entries_[i].t0, 0.0, entries_[i].dur));
    }

    /// 检查段间连续性（同样现场折叠，不做预计算）。
    [[nodiscard]] std::vector<JunctionReport> validate(double eps = 1e-9) const {
        std::vector<JunctionReport> out;
        State prevExit{};
        for (std::size_t i = 0; i < entries_.size(); ++i) {
            const Entry& e = entries_[i];
            State entry = (i == 0) ? initial_ : prevExit;
            entry = addDelta(entry, e.jump);

            JunctionReport r;
            r.segment = i;
            r.t = e.t0;
            r.declared_jump = jumpMagnitude(e.jump) > eps;
            r.internal_gap = junctionGap(entry, e.prim->eval(entry, 0.0));
            if (i > 0) r.entry_gap = junctionGap(prevExit, entry);
            r.warn = r.internal_gap.worst() > eps || (!r.declared_jump && r.entry_gap.worst() > eps);
            out.push_back(r);

            prevExit = e.prim->eval(entry, e.dur);
        }
        return out;
    }

    [[nodiscard]] std::string_view nameOf(std::size_t i) const { return entries_.at(i).prim->name(); }

private:
    /// 二分定位：满足 t0 <= t < t0+dur 的段下标。
    [[nodiscard]] std::size_t locate(double t) const {
        auto it = std::upper_bound(entries_.begin(), entries_.end(), t,
                                   [](double v, const Entry& e) { return v < e.t0; });
        if (it == entries_.begin()) return 0;
        --it;
        return static_cast<std::size_t>(std::distance(entries_.begin(), it));
    }

    std::vector<Entry> entries_;
    State initial_{};
    double total_ = 0.0;
};

}  // namespace sim
