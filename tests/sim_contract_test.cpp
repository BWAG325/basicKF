// 3D 仿真器契约测试：四元数姿态、三维积分、控制段切换、非均匀观测。

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "sim/config.hpp"
#include "sim/core/paths.hpp"
#include "sim/motion/controller.hpp"
#include "sim/motion/integrator.hpp"
#include "sim/motion/preset.hpp"
#include "sim/sensing/sensor.hpp"
#include "sim/simulator.hpp"

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

sim::RobotGeometry makeGeom() {
    sim::RobotGeometry g;
    g.armor_count = 4;
    return g;
}

sim::RobotState makeState(double speed = 0.0) {
    sim::RobotState s;
    s.p = sim::Vec3(0.0, 0.0, 0.18);
    s.speed = speed;
    s.v = sim::Vec3(speed, 0.0, 0.0);
    return s;
}

sim::Simulator makeSim(std::unique_ptr<sim::IController> c = nullptr, double min_dt = 0.001) {
    return sim::Simulator(makeGeom(), makeState(), std::move(c), min_dt);
}

sim::Simulator makeSim(const sim::RobotState& init, std::unique_ptr<sim::IController> c = nullptr,
                       double min_dt = 0.001) {
    return sim::Simulator(makeGeom(), init, std::move(c), min_dt);
}

// ---------------------------------------------------------------------------
// 积分
// ---------------------------------------------------------------------------

void testStraightAccel() {
    sim::Control c;
    c.accel = 2.0;

    sim::Simulator sim = makeSim();
    sim.step(1.0, c);

    checkNear(sim.state().speed, 2.0, 1e-12, "匀加速: v = at");
    checkNear(sim.state().p.x(), 1.0, 1e-12, "匀加速: p = ½at²");
    checkNear(sim.state().a.x(), 2.0, 1e-12, "匀加速: a = accel");
}

void testJerk() {
    sim::Control c;
    c.jerk = 2.0;

    sim::Simulator sim = makeSim();
    sim.step(1.0, c);

    checkNear(sim.state().speed, 1.0, 1e-12, "jerk: v = ½jt²");
    checkNear(sim.state().p.x(), 1.0 / 3.0, 1e-12, "jerk: p = jt³/6");
    checkNear(sim.state().accel, 2.0, 1e-12, "jerk: a = jt");
}

void testStop() {
    sim::Simulator sim = makeSim(makeState(2.0));
    sim::Control c;
    c.jerk = -4.0;

    sim.step(1.0, c);
    checkNear(sim.state().p.x(), 4.0 / 3.0, 1e-12, "jerk 刹停: 位移 = ∫(2-2τ²)");
    checkNear(sim.state().speed, 0.0, 1e-12, "jerk 刹停: v = 0");

    c.jerk = -4.0;
    sim.step(1.0, c);
    checkNear(sim.state().p.x(), 4.0 / 3.0, 1e-12, "停住后不再移动");
}

void testCircle() {
    sim::Control c;
    c.angular_rate = sim::Vec3(0.0, 0.0, 1.0);

    sim::Simulator sim = makeSim(makeState(2.0));
    sim.step(1.0, c);

    const sim::Vec3 want(2.0 * std::sin(1.0), 2.0 * (1.0 - std::cos(1.0)), 0.18);
    checkNear((sim.state().p - want).norm(), 0.0, 1e-9, "圆周: 位置闭式解");
    checkNear(sim.state().a.norm(), 2.0, 1e-9, "圆周: |a| = s·ω");
}

void testPitch() {
    sim::Control c;
    c.angular_rate = sim::Vec3(0.0, sim::kPi / 2.0, 0.0);  // 绕 body y 转 90°

    sim::Simulator sim = makeSim(makeState(1.0));
    sim.step(1.0, c);

    checkNear(sim.state().v.z(), -1.0, 1e-9, "pitch: 车头转 90° 后速度朝 -z");
    checkNear(sim.state().p.x(), 2.0 / sim::kPi, 1e-9, "pitch: x 位移 = 2/π");
    checkNear(sim.state().p.z() - 0.18, -2.0 / sim::kPi, 1e-9, "pitch: z 位移 = -2/π");
    checkNear(sim.state().q.norm(), 1.0, 1e-12, "姿态四元数保持单位长度");
}

void testQuatStaysUnit() {
    sim::Simulator sim = makeSim(makeState(1.5));
    sim::Control c;
    c.angular_rate = sim::Vec3(0.7, -0.4, 1.3);

    for (int i = 0; i < 1000; ++i) sim.step(0.01, c);
    checkNear(sim.state().q.norm(), 1.0, 1e-12, "长时间积分后四元数仍为单位长度");
}

// ---------------------------------------------------------------------------
// 增量推进 / 时间分辨率
// ---------------------------------------------------------------------------

void testSplitEquivalence() {
    sim::Control c;
    c.accel = 2.0;
    c.angular_rate = sim::Vec3(0.0, 0.0, 0.7);
    c.spin_rate = 3.0;

    sim::Simulator a = makeSim(makeState(1.0));
    a.step(0.5, c);

    sim::Simulator b = makeSim(makeState(1.0));
    for (int i = 0; i < 50; ++i) b.step(0.01, c);

    checkNear((a.state().p - b.state().p).norm(), 0.0, 1e-9, "一大步 = 50 小步（位置）");
    checkNear((a.state().v - b.state().v).norm(), 0.0, 1e-9, "一大步 = 50 小步（速度）");
    checkNear((a.state().q.coeffs() - b.state().q.coeffs()).norm(), 0.0, 1e-9, "一大步 = 50 小步（姿态）");
}

void testMinDtConvergence() {
    sim::Control c;
    c.accel = 1.0;
    c.angular_rate = sim::Vec3(0.0, 0.2, 0.9);

    sim::Simulator coarse = makeSim(makeState(1.0), nullptr, 1e-3);
    sim::Simulator fine = makeSim(makeState(1.0), nullptr, 1e-4);

    for (int i = 0; i < 200; ++i) {
        coarse.step(0.01, c);
        fine.step(0.01, c);
    }
    checkNear((coarse.state().p - fine.state().p).norm(), 0.0, 1e-8, "min_dt 越小结果越收敛");
}

void testPlanDuration() {
    auto mkplan = [] {
        std::vector<sim::Control> p(2);
        p[0].accel = 2.0;
        p[0].duration = 0.3;
        p[1].angular_rate = sim::Vec3(0.0, 0.0, 1.0);
        p[1].duration = 0.2;
        return p;
    };

    sim::Simulator a = makeSim(makeState(1.0), std::make_unique<sim::PlanController>(mkplan()));
    a.step(0.5);

    sim::Simulator b = makeSim(makeState(1.0), std::make_unique<sim::PlanController>(mkplan()));
    b.step(0.3);
    b.step(0.2);

    sim::Simulator c = makeSim(makeState(1.0), std::make_unique<sim::PlanController>(mkplan()));
    for (int i = 0; i < 10; ++i) c.step(0.05);

    checkNear((a.state().p - b.state().p).norm(), 0.0, 1e-9, "跨段一步 = 两段两步");
    checkNear((a.state().p - c.state().p).norm(), 0.0, 1e-9, "跨段一步 = 十小步");
}

void testExplicitControl() {
    std::vector<sim::Control> plan(2);
    plan[0].accel = 1.0;
    plan[0].duration = 0.2;
    plan[1].accel = 0.0;
    plan[1].duration = 0.2;

    auto pc = std::make_unique<sim::PlanController>(plan);
    auto* raw = pc.get();
    sim::Simulator sim = makeSim(makeState(), std::move(pc));

    sim.step(0.1);
    check(raw->remaining() == 1, "默认推进应取走第一段控制");

    sim.step(0.1, sim::Control{});
    check(raw->remaining() == 1, "显式控制不应消耗计划队列");

    const double v_before = sim.state().speed;
    sim.step(0.1);
    check(sim.state().speed > v_before, "之后回到计划继续加速");
}

void testPlanLoop() {
    std::vector<sim::Control> plan(1);
    plan[0].angular_rate = sim::Vec3(0.0, 0.0, 1.0);
    plan[0].duration = 0.5;

    // 循环 = 每圈结束重置回初值重放：t = k·0.5 + r 的状态应等于非循环仿真 t = r 的状态
    sim::Simulator loopSim(makeGeom(), makeState(2.0),
                           std::make_unique<sim::PlanController>(plan, true), 0.001);
    sim::Simulator ref(makeGeom(), makeState(2.0),
                       std::make_unique<sim::PlanController>(plan, false), 0.001);

    loopSim.advanceTo(2.25);  // 4 圈 + 0.25
    ref.advanceTo(0.25);

    checkNear((loopSim.state().p - ref.state().p).norm(), 0.0, 1e-9, "循环每圈重置回初值（位置）");
    checkNear((loopSim.state().v - ref.state().v).norm(), 0.0, 1e-9, "循环每圈重置回初值（速度）");
    checkNear(loopSim.time(), 2.25, 1e-12, "仿真时钟不回退");
}

void testPlanSeam() {
    sim::RobotGeometry g;
    g.armor_count = 4;
    const sim::RobotState init = makeState();

    std::vector<sim::Control> accel_only(1);
    accel_only[0].accel = 2.0;
    accel_only[0].duration = 1.0;
    const auto bad = sim::checkPlanSeam(g, init, accel_only, 1e-3);
    check(bad.warn, "只加速、不回初速 → 接缝应告警");
    checkNear(bad.dv, 2.0, 1e-6, "接缝速度差应等于末速度");

    std::vector<sim::Control> closed(2);
    closed[0].accel = 2.0;
    closed[0].duration = 1.0;
    closed[1].accel = -2.0;
    closed[1].duration = 1.0;
    const auto good = sim::checkPlanSeam(g, init, closed, 1e-3);
    check(!good.warn, "加速后等减速回 0 → 接缝闭合");
    checkNear(good.dv, 0.0, 1e-9, "闭合时速度差为 0");
}

void testRandomDeterminism() {
    sim::RandomController::Config cfg;
    cfg.seed = 5;
    cfg.min_pitch_rate = -0.3;
    cfg.max_pitch_rate = 0.3;

    sim::Simulator a = makeSim(makeState(), std::make_unique<sim::RandomController>(cfg));
    sim::Simulator b = makeSim(makeState(), std::make_unique<sim::RandomController>(cfg));

    for (int i = 0; i < 200; ++i) {
        a.step(0.02);
        b.step(0.02);
    }
    checkNear((a.state().p - b.state().p).norm(), 0.0, 1e-12, "同 seed 随机运动应完全一致");
    checkNear((a.state().q.coeffs() - b.state().q.coeffs()).norm(), 0.0, 1e-12, "同 seed 姿态一致");
}

// ---------------------------------------------------------------------------
// 几何 / 观测时刻表 / 观测
// ---------------------------------------------------------------------------

void testArmorGeometry() {
    sim::RobotGeometry g;
    g.armor_count = 4;
    g.armor_radius = 0.22;
    g.armor_dz = 0.05;

    const sim::Vec3 c(1.0, 2.0, 0.18);
    const sim::Quat body = sim::Quat(Eigen::AngleAxisd(0.3, sim::Vec3::UnitY()));

    const auto armors = sim::makeArmors(g, c, body, 0.7);
    const double expect_r = std::hypot(g.armor_radius, g.armor_dz);

    for (int i = 0; i < 4; ++i) {
        const auto& a = armors[static_cast<std::size_t>(i)];
        checkNear(a.n.norm(), 1.0, 1e-12, "法向必须是单位向量");
        checkNear((a.p - c).norm(), expect_r, 1e-12, "板中心到车体中心的距离固定");
        checkNear(a.up.norm(), 1.0, 1e-12, "up 必须是单位向量");
        checkNear(a.n.dot(a.up), 0.0, 1e-12, "法向与 up 正交");
    }

    check(std::abs(armors[0].n.y()) > 1e-3 || std::abs(armors[0].n.z()) > 1e-3,
          "车体姿态变化应传导到装甲板法向");
}

void testObservationSchedule() {
    sim::SensorConfig cfg;
    cfg.t0 = 0.1;
    cfg.interval = 0.04;
    cfg.jitter = 0.01;
    cfg.seed = 3;

    const auto a = sim::makeObservationTimes(cfg, 500);
    const auto b = sim::makeObservationTimes(cfg, 500);
    check(a == b, "观测时刻表应由 seed 唯一决定");
    checkNear(a[0], cfg.t0, 1e-12, "第一帧时刻应为 t0");

    bool nonuniform = false;
    for (std::size_t k = 1; k < a.size(); ++k) {
        const double d = a[k] - a[k - 1];
        check(d > 0.0, "帧间隔必须为正");
        check(d >= cfg.interval - cfg.jitter - 1e-12 && d <= cfg.interval + cfg.jitter + 1e-12,
              "帧间隔应落在 [interval-jitter, interval+jitter]");
        if (std::abs(d - cfg.interval) > 1e-9) nonuniform = true;
    }
    check(nonuniform, "jitter > 0 时帧间隔应非均匀");

    cfg.intervals = {0.05, 0.02, 0.08};
    const auto c = sim::makeObservationTimes(cfg, 7);
    checkNear(c[1] - c[0], 0.05, 1e-12, "显式间隔 1");
    checkNear(c[2] - c[1], 0.02, 1e-12, "显式间隔 2");
    checkNear(c[3] - c[2], 0.08, 1e-12, "显式间隔 3");
    checkNear(c[4] - c[3], 0.05, 1e-12, "显式间隔循环使用");
}

sim::SensorConfig frontCameraSensor() {
    sim::SensorConfig cfg;
    cfg.seed = 7;
    cfg.interval = 0.04;
    cfg.jitter = 0.01;
    cfg.position_sigma = sim::Vec3(0.01, 0.02, 0.03);
    cfg.normal_sigma = 0.02;
    cfg.dropout = 0.0;
    cfg.camera.enabled = true;
    cfg.camera.position = sim::Vec3(3.0, 0.0, 1.0);
    cfg.camera.target = sim::Vec3(0.0, 0.0, 0.18);
    return cfg;
}

void testSensorStats() {
    const sim::Simulator sim = makeSim();
    const sim::RobotTruth truth = sim.truth();

    const sim::SensorConfig cfg = frontCameraSensor();
    const sim::GaussianSensor sensor(cfg);

    const int N = 20000;
    const auto times = sim::makeObservationTimes(cfg, N);

    sim::Observation last;
    sim::Vec3 sum = sim::Vec3::Zero();
    sim::Vec3 sum2 = sim::Vec3::Zero();
    double nx = 0.0, nx2 = 0.0, ny = 0.0, ny2 = 0.0;
    int valid = 0;

    for (int i = 0; i < N; ++i) {
        sim::ObservationTruth anno;
        const sim::Observation obs =
            sensor.observeAt(truth, times[static_cast<std::size_t>(i)], static_cast<std::uint64_t>(i), &anno);
        if (!obs.valid) continue;
        ++valid;
        last = obs;

        const sim::ArmorTruth& a = truth.armors[static_cast<std::size_t>(anno.armor_index)];
        const sim::Vec3 r = obs.p - a.p;
        sum += r;
        sum2 += r.cwiseProduct(r);

        const sim::Vec3 e = obs.n - a.n;
        const sim::Vec3 t1 = a.up.cross(a.n).normalized();
        const sim::Vec3 t2 = a.n.cross(t1);
        nx += e.dot(t1);
        nx2 += e.dot(t1) * e.dot(t1);
        ny += e.dot(t2);
        ny2 += e.dot(t2) * e.dot(t2);
    }

    check(valid == N, "静态正对相机时应每帧都有效");
    const sim::Vec3 mean = sum / static_cast<double>(N);
    const sim::Vec3 var = sum2 / static_cast<double>(N) - mean.cwiseProduct(mean);
    for (int k = 0; k < 3; ++k) {
        const double sigma = cfg.position_sigma[k];
        checkNear(mean[k], 0.0, 5.0 * sigma / std::sqrt(static_cast<double>(N)), "位置噪声均值应为 0");
        checkNear(std::sqrt(var[k]), sigma, 0.05 * sigma, "位置噪声标准差应与配置一致");
    }

    const double s1 = std::sqrt(nx2 / N - (nx / N) * (nx / N));
    const double s2 = std::sqrt(ny2 / N - (ny / N) * (ny / N));
    checkNear(std::sqrt(0.5 * (s1 * s1 + s2 * s2)), cfg.normal_sigma, 0.12 * cfg.normal_sigma,
              "法向角噪声应与配置一致");
    checkNear(last.n.norm(), 1.0, 1e-9, "观测法向应是单位向量");
}

void testNoiseKeyedByFrame() {
    const sim::Simulator sim = makeSim();
    const sim::RobotTruth truth = sim.truth();
    const sim::GaussianSensor sensor(frontCameraSensor());

    const sim::Observation a = sensor.observeAt(truth, 1.0, 5);
    const sim::Observation b = sensor.observeAt(truth, 99.0, 5);
    checkNear((a.p - b.p).norm(), 0.0, 1e-15, "噪声只由帧号决定，与时间戳无关");
    checkNear((a.n - b.n).norm(), 0.0, 1e-15, "法向噪声只由帧号决定");

    const sim::Observation c = sensor.observeAt(truth, 1.0, 6);
    check((a.p - c.p).norm() > 1e-9, "不同帧号的噪声应不同");
}

void testSensorDeterminism() {
    const sim::Simulator sim = makeSim();
    const sim::RobotTruth truth = sim.truth();

    sim::SensorConfig cfg = frontCameraSensor();
    cfg.dropout = 0.3;
    const sim::GaussianSensor sensor(cfg);

    const auto times = sim::makeObservationTimes(cfg, 300);
    std::vector<sim::Observation> forward;
    forward.reserve(times.size());
    for (std::size_t k = 0; k < times.size(); ++k) {
        forward.push_back(sensor.observeAt(truth, times[k], static_cast<std::uint64_t>(k)));
    }

    std::vector<std::size_t> order(times.size());
    for (std::size_t i = 0; i < order.size(); ++i) order[i] = i;
    std::mt19937_64 rng(5);
    std::shuffle(order.begin(), order.end(), rng);

    bool same = true;
    for (std::size_t idx : order) {
        const sim::Observation got = sensor.observeAt(truth, times[idx], static_cast<std::uint64_t>(idx));
        if (got.valid != forward[idx].valid) same = false;
        if (got.valid && forward[idx].valid && (got.p != forward[idx].p || got.n != forward[idx].n)) same = false;
    }
    check(same, "观测必须与求值顺序无关（无状态随机数）");
}

void testCameraFrame() {
    const sim::Simulator sim = makeSim();
    const sim::RobotTruth truth = sim.truth();

    sim::SensorConfig cfg = frontCameraSensor();
    cfg.position_sigma = sim::Vec3(0.001, 0.001, 0.001);
    cfg.camera.position = sim::Vec3(2.0, -1.0, 1.5);
    cfg.camera.output_in_camera_frame = true;

    const sim::GaussianSensor sensor(cfg);
    sim::ObservationTruth anno;
    const sim::Observation obs = sensor.observeAt(truth, 0.0, 0, &anno);

    check(obs.valid, "相机系输出时观测应有效");
    check(obs.in_camera_frame, "in_camera_frame 标志应置位");

    const sim::Vec3 ref = truth.armors[static_cast<std::size_t>(anno.armor_index)].p;
    const double expect_range = (ref - cfg.camera.position).norm();
    checkNear(obs.p.norm(), expect_range, 1e-2, "相机系下 |p| 应等于到相机的距离");
    check(obs.p.z() > 0.0, "相机系下 z 应大于 0（在相机前方）");
    checkNear(obs.n.norm(), 1.0, 1e-9, "相机系法向仍是单位向量");
}

// ---------------------------------------------------------------------------
// 预设 / 配置
// ---------------------------------------------------------------------------

void testPresetCircle() {
    sim::PresetParams p;
    p.speed = 2.0;
    p.radius = 1.0;
    p.accel = 2.0;
    p.duration = 5.0;

    const auto plan = sim::makePresetControls("circle", p);
    check(plan.size() == 2, "circle 预设应为两段（加速 + 绕圈）");

    sim::Simulator sim = makeSim(makeState(), std::make_unique<sim::PlanController>(plan), 0.001);
    sim.advanceTo(2.0);
    checkNear(sim.state().speed, 2.0, 1e-9, "circle: 加速结束速度 = 2 m/s");
    checkNear(sim.state().a.norm(), 4.0, 1e-6, "circle: 向心加速度 = v²/R");

    sim.advanceTo(2.5);
    const sim::Vec3 A = sim.state().p;
    sim.advanceTo(3.0);
    const sim::Vec3 B = sim.state().p;
    sim.advanceTo(3.5);
    const sim::Vec3 C = sim.state().p;
    const double ab = (B - A).norm();
    const double bc = (C - B).norm();
    const double ca = (A - C).norm();
    const double R = (ab * bc * ca) / (2.0 * (B - A).cross(C - A).norm());
    checkNear(R, 1.0, 1e-6, "circle: 轨迹半径 = radius");
}

void testPresetForever() {
    sim::PresetParams p;
    p.speed = 2.0;
    p.radius = 1.0;
    p.accel = 2.0;
    p.duration = 5.0;
    p.forever = true;

    const auto plan = sim::makePresetControls("circle", p);
    check(plan.size() == 2, "circle forever 仍为两段");
    check(std::isinf(plan.back().duration), "forever: 最后一段时长应为无穷");

    sim::Simulator sim = makeSim(makeState(), std::make_unique<sim::PlanController>(plan), 0.001);
    sim.advanceTo(30.0);
    checkNear(sim.state().speed, 2.0, 1e-9, "forever: 30s 后仍保持 2 m/s");
    checkNear(sim.state().a.norm(), 4.0, 1e-6, "forever: 向心加速度保持");

    sim.advanceTo(30.5);
    const sim::Vec3 A = sim.state().p;
    sim.advanceTo(31.0);
    const sim::Vec3 B = sim.state().p;
    sim.advanceTo(31.5);
    const sim::Vec3 C = sim.state().p;
    const double R = ((B - A).norm() * (C - B).norm() * (A - C).norm()) /
                     (2.0 * (B - A).cross(C - A).norm());
    checkNear(R, 1.0, 1e-6, "forever: 半径始终保持不变");

    // 有限模式仍然会结束
    p.forever = false;
    const auto finite = sim::makePresetControls("circle", p);
    check(!std::isinf(finite.back().duration), "forever=false 时最后一段应有限");
}

void testPresetNames() {
    const auto names = sim::presetNames();
    check(std::find(names.begin(), names.end(), "circle") != names.end(), "预设列表应包含 circle");
    check(std::find(names.begin(), names.end(), "straight") != names.end(), "预设列表应包含 straight");
    check(sim::makePresetControls("no_such_preset", {}).empty(), "未知预设应返回空序列");

    const auto idle = sim::makePresetControls("idle", {});
    check(idle.size() == 1 && idle[0].accel == 0.0 && idle[0].angular_rate.norm() == 0.0,
          "idle 应为一段零控制");
}

void testConfigFiles() {
    const std::string dir = sim::resolvePath("configs/sim/");
    for (const std::string name : {"rm_straight_spin.yaml", "rm_random.yaml", "rm_circle.yaml", "rm_weave.yaml", "rm_loop.yaml"}) {
        auto setup = sim::loadSimSetup(dir + name);
        sim::Simulator sim(setup.geometry, setup.initial, std::move(setup.controller), setup.min_dt);
        const sim::GaussianSensor sensor(setup.sensor);

        const auto times = sim::makeObservationTimes(setup.sensor, 120);
        int valid = 0;
        for (std::size_t k = 0; k < times.size(); ++k) {
            const sim::RobotTruth tr = sim.advanceTo(times[k]);
            sim::ObservationTruth anno;
            const sim::Observation obs =
                sensor.observeAt(tr, times[k], static_cast<std::uint64_t>(k), &anno);
            if (obs.valid) ++valid;
            check(tr.chassis.p.allFinite(), name + ": 真值必须有限");
            check(obs.p.allFinite(), name + ": 观测必须有限");
            checkNear(tr.chassis.t, times[k], 1e-9, name + ": 推进后时间应与观测时刻一致");
        }
        check(valid > 60, name + ": 大部分帧应有有效观测");
    }
}

}  // namespace

int main() {
    testStraightAccel();
    testJerk();
    testStop();
    testCircle();
    testPitch();
    testQuatStaysUnit();
    testSplitEquivalence();
    testMinDtConvergence();
    testPlanDuration();
    testPlanLoop();
    testPlanSeam();
    testExplicitControl();
    testRandomDeterminism();
    testArmorGeometry();
    testObservationSchedule();
    testSensorStats();
    testNoiseKeyedByFrame();
    testSensorDeterminism();
    testCameraFrame();
    testPresetCircle();
    testPresetForever();
    testPresetNames();
    testConfigFiles();

    if (g_fail == 0) {
        std::cout << "all sim contract tests passed\n";
        return 0;
    }
    std::cerr << g_fail << " check(s) failed\n";
    return 1;
}
