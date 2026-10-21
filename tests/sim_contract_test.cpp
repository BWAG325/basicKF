// 模拟器「契约测试」：这些测试定义了接口必须永远满足的性质。
// 一旦它们变红，说明「任意 t 求值」「导数为真」「观测可复现」这些承诺被破坏了。

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "sim/config.hpp"
#include "sim/primitives.hpp"
#include "sim/scene.hpp"
#include "sim/sensor.hpp"

namespace {

int g_fail = 0;

void check(bool ok, const std::string& what) {
    if (!ok) {
        std::cerr << "[FAIL] " << what << "\n";
        ++g_fail;
    }
}

void checkNear(double got, double want, double tol, const std::string& what) {
    if (!(std::abs(got - want) <= tol)) {
        std::cerr << "[FAIL] " << what << ": got " << got << " want " << want << " (tol " << tol << ")\n";
        ++g_fail;
    }
}

bool sameTruth(const sim::RobotTruth& a, const sim::RobotTruth& b) {
    if (a.chassis.p != b.chassis.p || a.chassis.v != b.chassis.v) return false;
    if (a.chassis.yaw != b.chassis.yaw || a.chassis.yaw_rate != b.chassis.yaw_rate) return false;
    if (a.spin_phase_raw != b.spin_phase_raw || a.spin_rate != b.spin_rate) return false;
    for (int i = 0; i < a.armor_count; ++i) {
        if (a.armors[static_cast<std::size_t>(i)].p != b.armors[static_cast<std::size_t>(i)].p) return false;
    }
    return true;
}

sim::Scene makeScene(sim::SpinSemantics sem = sim::SpinSemantics::BodyRotation, double spin_rate = 3.0) {
    sim::Scene sc;
    sim::RobotGeometry g;
    g.armor_count = 4;
    sc.setGeometry(g);
    sc.setSpinSemantics(sem);

    sim::TranslationState tr;
    tr.p = sim::Vec3(0.0, 0.0, 0.18);
    tr.v = sim::Vec3(2.0, 0.0, 0.0);
    sim::YawState ys;
    sim::PhaseState sp;
    sp.omega = spin_rate;
    sc.setInitial(tr, ys, sp);

    sc.translation().add(std::make_unique<sim::Line>(2.0, sim::kSpeedInherit, 0.0));
    sc.translation().add(std::make_unique<sim::Arc>(1.0, sim::kSpeedInherit, 1.0));
    sc.yaw().add(std::make_unique<sim::YawHold>(3.0));
    sc.spin().add(std::make_unique<sim::SpinRate>(3.0, spin_rate));
    return sc;
}

/// 只数 eval 次数的直线：用来证明“构建期零计算、查询时才折叠”。
struct EvalCounter {
    static inline int calls = 0;
};

class CountingLine final : public sim::ITranslationPrimitive {
public:
    explicit CountingLine(double d) : dur_(d) {}
    [[nodiscard]] double duration() const override { return dur_; }
    [[nodiscard]] std::string_view name() const override { return "counting_line"; }
    [[nodiscard]] sim::TranslationState eval(const sim::TranslationState& s, double tau) const override {
        ++EvalCounter::calls;
        return sim::TranslationState{s.p + sim::Vec3(1.0, 0.0, 0.0) * tau,
                                     sim::Vec3(1.0, 0.0, 0.0), sim::Vec3::Zero()};
    }

private:
    double dur_;
};

sim::Scene makeStaticScene(double duration = 10.0) {
    sim::Scene sc;
    sim::RobotGeometry g;
    g.armor_count = 4;
    sc.setGeometry(g);

    sim::TranslationState tr;
    tr.p = sim::Vec3(0.0, 0.0, 0.18);
    sc.setInitial(tr, sim::YawState{}, sim::PhaseState{});

    sc.translation().add(std::make_unique<sim::Hold>(duration));
    sc.yaw().add(std::make_unique<sim::YawHold>(duration));
    sc.spin().add(std::make_unique<sim::SpinOff>(duration));
    return sc;
}

// ---------------------------------------------------------------------------

void testOrderIndependence() {
    const sim::Scene sc = makeScene();

    std::vector<double> ts;
    ts.reserve(400);
    for (int i = 0; i < 400; ++i) ts.push_back(static_cast<double>(i) * 0.0037);

    std::map<double, sim::RobotTruth> forward;
    for (double t : ts) forward[t] = sc.truthAt(t);

    std::mt19937_64 rng(12345);
    std::shuffle(ts.begin(), ts.end(), rng);

    bool all_same = true;
    for (double t : ts) {
        const sim::RobotTruth got = sc.truthAt(t);
        if (!sameTruth(got, forward.at(t))) all_same = false;
    }
    check(all_same, "乱序求值必须与顺序求值逐位一致");
}

void testQueryTimeCompute() {
    sim::Scene sc;
    sim::RobotGeometry g;
    g.armor_count = 4;
    sc.setGeometry(g);
    sim::TranslationState tr;
    tr.p = sim::Vec3(0.0, 0.0, 0.18);
    sc.setInitial(tr, sim::YawState{}, sim::PhaseState{});

    constexpr int kSegments = 20;
    for (int i = 0; i < kSegments; ++i) sc.translation().add(std::make_unique<CountingLine>(1.0));
    sc.yaw().add(std::make_unique<sim::YawHold>(20.0));
    sc.spin().add(std::make_unique<sim::SpinOff>(20.0));

    check(EvalCounter::calls == 0, "add 阶段必须一次 eval 都不做（构建期零计算）");

    EvalCounter::calls = 0;
    (void)sc.truthAt(0.5);  // 第 0 段
    check(EvalCounter::calls == 1, "t 落在首段：只求值 1 次");

    EvalCounter::calls = 0;
    (void)sc.truthAt(5.5);  // 第 5 段：前缀 5 段 + 本段
    check(EvalCounter::calls == 6, "t 落在第 5 段：现场折叠 6 次（前缀 + 本段）");

    EvalCounter::calls = 0;
    (void)sc.truthAt(19.5);  // 末段
    check(EvalCounter::calls == kSegments, "t 落在末段：折叠 20 次");

    EvalCounter::calls = 0;
    (void)sc.truthAt(19.5);  // 再问一次：不需要缓存也已算过的段
    check(EvalCounter::calls == kSegments, "重复查询不改结果、也不依赖缓存（每次都按 t 重算）");
}

void testDerivatives() {
    const sim::Scene sc = makeScene();
    const double h = 1e-6;

    for (double t : {0.30, 1.20, 2.50}) {
        const sim::RobotTruth a = sc.truthAt(t - h);
        const sim::RobotTruth b = sc.truthAt(t + h);
        const sim::RobotTruth c = sc.truthAt(t);

        const sim::Vec3 v_fd = (b.chassis.p - a.chassis.p) / (2.0 * h);
        const sim::Vec3 a_fd = (b.chassis.v - a.chassis.v) / (2.0 * h);

        checkNear((v_fd - c.chassis.v).norm(), 0.0, 1e-5, "v 必须等于 dp/dt");
        checkNear((a_fd - c.chassis.a).norm(), 0.0, 1e-4, "a 必须等于 dv/dt");
        checkNear(std::abs(c.chassis.a.z()), 0.0, 1e-12, "平面运动 a.z 应为 0");
    }
}

void testAnalyticPrimitives() {
    // 直线：p = p0 + v t
    {
        sim::Scene sc;
        sim::RobotGeometry g;
        g.armor_count = 4;
        sc.setGeometry(g);
        sim::TranslationState tr;
        tr.p = sim::Vec3(1.0, 2.0, 0.18);
        tr.v = sim::Vec3(2.0, 0.0, 0.0);
        sc.setInitial(tr, sim::YawState{}, sim::PhaseState{});
        sc.translation().add(std::make_unique<sim::Line>(1.0));
        sc.yaw().add(std::make_unique<sim::YawHold>(1.0));
        sc.spin().add(std::make_unique<sim::SpinOff>(1.0));
            const sim::RobotTruth t = sc.truthAt(1.0);
        checkNear((t.chassis.p - sim::Vec3(3.0, 2.0, 0.18)).norm(), 0.0, 1e-12, "line: p = p0 + v t");
    }

    // 等加速：p = p0 + 0.5 a t², v = a t
    {
        sim::Scene sc;
        sim::RobotGeometry g;
        g.armor_count = 4;
        sc.setGeometry(g);
        sim::TranslationState tr;
        tr.p = sim::Vec3(0.0, 0.0, 0.18);
        sc.setInitial(tr, sim::YawState{}, sim::PhaseState{});
        sc.translation().add(std::make_unique<sim::Accel>(1.0, 2.0, 0.0));
        sc.yaw().add(std::make_unique<sim::YawHold>(1.0));
        sc.spin().add(std::make_unique<sim::SpinOff>(1.0));
            const sim::RobotTruth t = sc.truthAt(1.0);
        checkNear((t.chassis.p - sim::Vec3(1.0, 0.0, 0.18)).norm(), 0.0, 1e-12, "accel: p = 0.5 a t²");
        checkNear(t.chassis.v.x(), 2.0, 1e-12, "accel: v = a t");
    }

    // 圆弧：s=2, ω=1, 1s ⇒ p = (2 sin1, 2(1-cos1)), |a| = s·ω
    {
        sim::Scene sc;
        sim::RobotGeometry g;
        g.armor_count = 4;
        sc.setGeometry(g);
        sim::TranslationState tr;
        tr.p = sim::Vec3(0.0, 0.0, 0.18);
        tr.v = sim::Vec3(2.0, 0.0, 0.0);
        sc.setInitial(tr, sim::YawState{}, sim::PhaseState{});
        sc.translation().add(std::make_unique<sim::Arc>(1.0, 2.0, 1.0, 0.0));
        sc.yaw().add(std::make_unique<sim::YawHold>(1.0));
        sc.spin().add(std::make_unique<sim::SpinOff>(1.0));
            const sim::RobotTruth t = sc.truthAt(1.0);
        const sim::Vec3 want(2.0 * std::sin(1.0), 2.0 * (1.0 - std::cos(1.0)), 0.18);
        checkNear((t.chassis.p - want).norm(), 0.0, 1e-12, "arc: 位置解析式");
        checkNear(t.chassis.a.norm(), 2.0, 1e-12, "arc: |a| = s·ω（向心）");
        checkNear(std::abs(sim::angleDiff(std::atan2(t.chassis.v.y(), t.chassis.v.x()), 1.0)), 0.0, 1e-12,
                  "arc: 速度方向转过 ωt");
    }
}

void testPhaseIntegral() {
    sim::Scene sc;
    sim::RobotGeometry g;
    g.armor_count = 4;
    sc.setGeometry(g);

    sim::TranslationState tr;
    tr.p = sim::Vec3(0.0, 0.0, 0.18);
    sc.setInitial(tr, sim::YawState{}, sim::PhaseState{});

    sc.translation().add(std::make_unique<sim::Hold>(5.0));
    sc.yaw().add(std::make_unique<sim::YawHold>(5.0));
    sc.spin().add(std::make_unique<sim::SpinOff>(1.0));
    sc.spin().add(std::make_unique<sim::SpinRamp>(1.0, 0.0, 8.0));
    sc.spin().add(std::make_unique<sim::SpinRate>(3.0, 8.0));

    // 解析值：0.5*8*1 + 8*3 = 28 rad
    checkNear(sc.truthAt(5.0).spin_phase_raw, 28.0, 1e-12, "ramp 相位解析积分");

    // 数值积分对拍
    const double h = 1e-4;
    double acc = 0.0;
    for (double t = 0.0; t < 5.0; t += h) {
        const double w0 = sc.truthAt(t).spin_rate;
        const double w1 = sc.truthAt(t + h).spin_rate;
        acc += 0.5 * (w0 + w1) * h;
    }
    checkNear(sc.truthAt(5.0).spin_phase_raw, acc, 1e-6, "phase 必须等于 ∫ω dt");
}

void testContinuity() {
    const sim::Scene good = makeScene();
    check(!good.validate().anyWarn(), "C1 连续场景不应产生 warn");

    sim::Scene bad;
    sim::RobotGeometry g;
    g.armor_count = 4;
    bad.setGeometry(g);
    sim::TranslationState tr;
    tr.v = sim::Vec3(2.0, 0.0, 0.0);
    bad.setInitial(tr, sim::YawState{}, sim::PhaseState{});
    bad.translation().add(std::make_unique<sim::Line>(1.0, sim::kSpeedInherit, 0.0));
    bad.translation().add(std::make_unique<sim::Hold>(1.0));  // 瞬时归零 → 未声明跳变
    bad.yaw().add(std::make_unique<sim::YawHold>(2.0));
    bad.spin().add(std::make_unique<sim::SpinOff>(2.0));

    check(bad.validate().anyWarn(), "运动段后接 hold 必须报未声明速度跳变");
}

void testDeclaredJump() {
    sim::Scene sc;
    sim::RobotGeometry g;
    g.armor_count = 4;
    sc.setGeometry(g);

    sim::TranslationState tr;
    tr.v = sim::Vec3(2.0, 0.0, 0.0);
    sc.setInitial(tr, sim::YawState{}, sim::PhaseState{});

    sc.translation().add(std::make_unique<sim::Line>(1.0));  // 继承 2 m/s
    sim::TranslationState jump;
    jump.v = sim::Vec3(-1.0, 0.0, 0.0);                      // 模拟受击掉速
    sc.translation().add(std::make_unique<sim::Line>(1.0), jump);
    sc.yaw().add(std::make_unique<sim::YawHold>(2.0));
    sc.spin().add(std::make_unique<sim::SpinOff>(2.0));

    check(!sc.validate().anyWarn(), "显式 jump 不应报 warn");
    checkNear(sc.truthAt(1.5).chassis.v.x(), 1.0, 1e-12, "jump 后的速度应被后续段继承");
}

void testWrapContinuity() {
    const sim::Scene sc = makeScene(sim::SpinSemantics::BodyRotation, 12.0);
    const double h = 1e-4;
    const double bound = (0.22 * 12.0 + 2.0 + 1.0) * h;

    double max_dp = 0.0;
    sim::Vec3 prev = sc.truthAt(0.0).armors[0].p;
    for (double t = h; t <= 1.0; t += h) {
        const sim::Vec3 cur = sc.truthAt(t).armors[0].p;
        max_dp = std::max(max_dp, (cur - prev).norm());
        prev = cur;
    }
    check(max_dp < bound * 1.5, "跨 2π 时装甲板位置必须连续");
}

void testClampOutsideRange() {
    const sim::Scene sc = makeScene();
    check(sameTruth(sc.truthAt(-1.0), sc.truthAt(0.0)), "t<0 应 clamp 到 0");
    check(sameTruth(sc.truthAt(100.0), sc.truthAt(sc.duration())), "t>T 应 clamp 到 T");
    checkNear(sc.truthAt(100.0).chassis.t, sc.duration(), 1e-12, "clamp 后时间戳应为 T");
}

void testGeometry() {
    sim::RobotGeometry g;
    g.armor_count = 4;
    g.armor_radius = 0.22;
    g.armor_dz = 0.05;

    const sim::Vec3 c(1.0, 2.0, 0.18);
    const double ring = 0.7;
    const auto armors = sim::makeArmors(g, c, ring);

    for (int i = 0; i < 4; ++i) {
        const auto& a = armors[static_cast<std::size_t>(i)];
        checkNear(a.n.norm(), 1.0, 1e-12, "法向必须是单位向量");
        checkNear((a.p - c - sim::Vec3(0, 0, g.armor_dz)).norm(), g.armor_radius, 1e-12,
                  "板中心必须在半径圆上");
        checkNear(std::abs(sim::angleDiff(a.yaw, std::atan2(a.n.y(), a.n.x()))), 0.0, 1e-12,
                  "yaw 必须与法向一致");
    }

    for (int i = 1; i < 4; ++i) {
        const double d = sim::angleDiff(armors[static_cast<std::size_t>(i)].yaw,
                                        armors[static_cast<std::size_t>(i - 1)].yaw);
        checkNear(std::abs(d), sim::kPi / 2.0, 1e-12, "相邻板夹角 90°");
    }
}

void testSpinSemantics() {
    const sim::Scene body = makeScene(sim::SpinSemantics::BodyRotation, 2.0);
    const sim::Scene orbit = makeScene(sim::SpinSemantics::ArmorOrbit, 2.0);

    const sim::RobotTruth tb = body.truthAt(1.0);
    const sim::RobotTruth to = orbit.truthAt(1.0);

    checkNear(std::abs(sim::angleDiff(tb.armors[0].yaw, to.armors[0].yaw)), 0.0, 1e-12,
              "两种自转语义下装甲环角度必须一致");
    checkNear(std::abs(sim::angleDiff(tb.chassis.yaw, 2.0)), 0.0, 1e-12,
              "body_rotation 下 chassis.yaw 应含自转");
    checkNear(std::abs(sim::angleDiff(to.chassis.yaw, 0.0)), 0.0, 1e-12,
              "armor_orbit 下 chassis.yaw 不应含自转");
}

void testSensorStats() {
    const sim::Scene sc = makeStaticScene();

    sim::SensorConfig cfg;
    cfg.dt = 0.01;
    cfg.seed = 7;
    cfg.position_sigma = sim::Vec3(0.01, 0.02, 0.03);
    cfg.yaw_sigma = 0.02;
    cfg.dropout = 0.0;
    cfg.camera.enabled = true;
    cfg.camera.position = sim::Vec3(3.0, 0.0, 1.0);
    cfg.camera.target = sim::Vec3(0.0, 0.0, 0.18);

    const sim::GaussianSensor sensor(cfg);

    const int N = 20000;
    sim::Observation last;
    sim::Vec3 sum = sim::Vec3::Zero();
    sim::Vec3 sum2 = sim::Vec3::Zero();
    double yaw_sum = 0.0;
    double yaw_sum2 = 0.0;
    int valid = 0;

    for (int i = 0; i < N; ++i) {
        const double t = static_cast<double>(i) * cfg.dt;
        const sim::RobotTruth tr = sc.truthAt(t);
        sim::ObservationTruth anno;
        const sim::Observation obs = sensor.observeAt(tr, t, &anno);
        if (!obs.valid) continue;
        ++valid;
        last = obs;
        const sim::Vec3 ref = tr.armors[static_cast<std::size_t>(anno.armor_index)].p;
        const sim::Vec3 r = obs.p - ref;
        sum += r;
        sum2 += r.cwiseProduct(r);
        const double dy = sim::angleDiff(obs.yaw, tr.armors[static_cast<std::size_t>(anno.armor_index)].yaw);
        yaw_sum += dy;
        yaw_sum2 += dy * dy;
    }

    check(valid == N, "静态正对相机时应每帧都有效");
    const sim::Vec3 mean = sum / static_cast<double>(N);
    const sim::Vec3 var = sum2 / static_cast<double>(N) - mean.cwiseProduct(mean);
    for (int k = 0; k < 3; ++k) {
        const double sigma = cfg.position_sigma[k];
        checkNear(mean[k], 0.0, 5.0 * sigma / std::sqrt(static_cast<double>(N)), "位置噪声均值应为 0");
        checkNear(std::sqrt(var[k]), sigma, 0.05 * sigma, "位置噪声标准差应与配置一致");
    }

    const double yaw_mean = yaw_sum / static_cast<double>(N);
    const double yaw_var = yaw_sum2 / static_cast<double>(N) - yaw_mean * yaw_mean;
    checkNear(yaw_mean, 0.0, 5.0 * cfg.yaw_sigma / std::sqrt(static_cast<double>(N)), "yaw 噪声均值应为 0");
    checkNear(std::sqrt(yaw_var), cfg.yaw_sigma, 0.06 * cfg.yaw_sigma, "yaw 噪声标准差应与配置一致");

    // R 必须由观测层给出，且与配置一致
    for (int k = 0; k < 3; ++k) {
        checkNear(last.R_pos(k, k), cfg.position_sigma[k] * cfg.position_sigma[k], 1e-15,
                  "R_pos 对角线应为 sigma²");
        for (int j = 0; j < 3; ++j) {
            if (j != k) checkNear(last.R_pos(k, j), 0.0, 1e-18, "R_pos 非对角应为 0");
        }
    }
    checkNear(last.R_yaw, cfg.yaw_sigma * cfg.yaw_sigma, 1e-15, "R_yaw 应为 sigma²");
}

void testSensorDeterminism() {
    const sim::Scene sc = makeScene();

    sim::SensorConfig cfg;
    cfg.dt = 0.01;
    cfg.seed = 99;
    cfg.dropout = 0.3;
    cfg.camera.enabled = true;
    cfg.camera.position = sim::Vec3(3.0, 0.0, 1.0);
    cfg.camera.target = sim::Vec3(0.0, 0.0, 0.18);
    const sim::GaussianSensor sensor(cfg);

    std::vector<double> ts;
    for (int i = 0; i < 300; ++i) ts.push_back(static_cast<double>(i) * 0.01);

    std::map<double, sim::Observation> forward;
    for (double t : ts) forward[t] = sensor.observeAt(sc.truthAt(t), t);

    std::mt19937_64 rng(5);
    std::shuffle(ts.begin(), ts.end(), rng);

    bool same = true;
    for (double t : ts) {
        const sim::Observation got = sensor.observeAt(sc.truthAt(t), t);
        const sim::Observation& ref = forward.at(t);
        if (got.valid != ref.valid) same = false;
        if (got.valid && ref.valid && (got.p != ref.p || got.yaw != ref.yaw)) same = false;
    }
    check(same, "观测必须与求值顺序无关（无状态随机数）");

    const sim::Observation a = sensor.observeAt(sc.truthAt(0.5), 0.5);
    const sim::Observation b = sensor.observeAt(sc.truthAt(0.5), 0.5);
    check(a.p == b.p && a.yaw == b.yaw && a.valid == b.valid, "同一 t 重复求值必须一致");
}

void testTriggeredAt() {
    const sim::Scene sc = makeStaticScene();
    sim::SensorConfig cfg;
    cfg.dt = 0.01;
    cfg.dropout = 0.0;
    const sim::GaussianSensor sensor(cfg);

    check(sensor.triggeredAt(0.5), "t 落在采样网格上应触发");
    check(!sensor.triggeredAt(0.5004), "t 不在网格上不应触发");
    checkNear(sensor.sampleTime(0.5004), 0.50, 1e-12, "sampleTime 应回到最近网格");

    cfg.dropout = 0.5;
    const sim::GaussianSensor lossy(cfg);
    int dropped = 0;
    for (int i = 0; i < 5000; ++i) {
        if (!lossy.triggeredAt(static_cast<double>(i) * 0.01)) ++dropped;
    }
    const double frac = static_cast<double>(dropped) / 5000.0;
    check(frac > 0.45 && frac < 0.55, "丢帧比例应接近配置值");
}

void testCameraFrame() {
    const sim::Scene sc = makeStaticScene();
    sim::SensorConfig cfg;
    cfg.dt = 0.01;
    cfg.dropout = 0.0;
    cfg.position_sigma = sim::Vec3(0.001, 0.001, 0.001);
    cfg.camera.enabled = true;
    cfg.camera.position = sim::Vec3(2.0, -1.0, 1.5);
    cfg.camera.target = sim::Vec3(0.0, 0.0, 0.18);
    cfg.camera.output_in_camera_frame = true;

    const sim::GaussianSensor sensor(cfg);
    sim::ObservationTruth anno;
    const sim::Observation obs = sensor.observeAt(sc.truthAt(0.0), 0.0, &anno);

    check(obs.valid, "相机系输出时观测应有效");
    check(obs.in_camera_frame, "in_camera_frame 标志应置位");

    const sim::Vec3 ref = sc.truthAt(0.0).armors[static_cast<std::size_t>(anno.armor_index)].p;
    const double expect_range = (ref - cfg.camera.position).norm();
    checkNear(obs.p.norm(), expect_range, 1e-2, "相机系下 |p| 应等于到相机的距离");
    check(obs.p.z() > 0.0, "相机系下 z 应大于 0（在相机前方）");
}

void testConfigFiles() {
    const std::string dir = std::string(SIM_SOURCE_DIR) + "/configs/sim/";
    for (const std::string name : {"rm_straight_spin.yaml", "rm_weave_spin.yaml"}) {
        auto setup = sim::loadSimSetup(dir + name);
        check(setup.scene.duration() > 0.0, name + ": duration 应 > 0（构建期零计算，duration 只是求和）");

        const sim::RobotTruth tr = setup.scene.truthAt(setup.scene.duration() * 0.5);
        check(tr.chassis.p.allFinite(), name + ": 真值必须有限");
        check(tr.armor_count == 4, name + ": 默认 4 块装甲板");

        const sim::GaussianSensor sensor(setup.sensor);
        sim::ObservationTruth anno;
        const sim::Observation obs = sensor.observeAt(tr, setup.scene.duration() * 0.5, &anno);
        check(obs.p.allFinite(), name + ": 观测必须有限");
        check(!setup.scene.validate().anyWarn(), name + ": 配置不应有未声明跳变");
    }
}

}  // namespace

int main() {
    testOrderIndependence();
    testQueryTimeCompute();
    testDerivatives();
    testAnalyticPrimitives();
    testPhaseIntegral();
    testContinuity();
    testDeclaredJump();
    testWrapContinuity();
    testClampOutsideRange();
    testGeometry();
    testSpinSemantics();
    testSensorStats();
    testSensorDeterminism();
    testTriggeredAt();
    testCameraFrame();
    testConfigFiles();

    if (g_fail == 0) {
        std::cout << "all sim contract tests passed\n";
        return 0;
    }
    std::cerr << g_fail << " check(s) failed\n";
    return 1;
}
