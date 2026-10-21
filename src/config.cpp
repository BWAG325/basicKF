#include "sim/config.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "sim/preset.hpp"

namespace sim {

namespace {

[[noreturn]] void fail(const std::string& msg) { throw std::runtime_error("[config] " + msg); }

double getD(const YAML::Node& n, const char* k, double def) {
    if (!n[k]) return def;
    try {
        return n[k].as<double>();
    } catch (const std::exception& e) {
        fail(std::string("字段 ") + k + " 不是数值: " + e.what());
    }
}

int getI(const YAML::Node& n, const char* k, int def) {
    if (!n[k]) return def;
    try {
        return n[k].as<int>();
    } catch (const std::exception& e) {
        fail(std::string("字段 ") + k + " 不是整数: " + e.what());
    }
}

bool getB(const YAML::Node& n, const char* k, bool def) {
    if (!n[k]) return def;
    try {
        return n[k].as<bool>();
    } catch (const std::exception& e) {
        fail(std::string("字段 ") + k + " 不是布尔: " + e.what());
    }
}

std::uint64_t getU64(const YAML::Node& n, const char* k, std::uint64_t def) {
    if (!n[k]) return def;
    try {
        return n[k].as<std::uint64_t>();
    } catch (const std::exception& e) {
        fail(std::string("字段 ") + k + " 不是无符号整数: " + e.what());
    }
}

std::string getS(const YAML::Node& n, const char* k, const std::string& def) {
    if (!n[k]) return def;
    return n[k].as<std::string>();
}

Vec3 getV3(const YAML::Node& n, const char* k, const Vec3& def) {
    if (!n[k]) return def;
    const YAML::Node v = n[k];
    if (!v.IsSequence() || v.size() != 3) fail(std::string("字段 ") + k + " 需要 3 个数");
    return Vec3(v[0].as<double>(), v[1].as<double>(), v[2].as<double>());
}

std::pair<double, double> getRange(const YAML::Node& n, const char* k,
                                   const std::pair<double, double>& def) {
    if (!n[k]) return def;
    const YAML::Node v = n[k];
    if (!v.IsSequence() || v.size() != 2) fail(std::string("字段 ") + k + " 需要 [lo, hi]");
    const double lo = v[0].as<double>();
    const double hi = v[1].as<double>();
    if (hi < lo) fail(std::string("字段 ") + k + " 的 hi < lo");
    return {lo, hi};
}

/// 控制段的角速度：优先 `angular_rate: [x,y,z]`，也接受 turn/pitch/roll_rate 三个标量。
Vec3 getAngularRate(const YAML::Node& seg) {
    Vec3 w = getV3(seg, "angular_rate", Vec3::Zero());
    if (seg["turn_rate"]) w.z() = seg["turn_rate"].as<double>();
    if (seg["pitch_rate"]) w.y() = seg["pitch_rate"].as<double>();
    if (seg["roll_rate"]) w.x() = seg["roll_rate"].as<double>();
    return w;
}

}  // namespace

SimSetup loadSimSetup(const std::string& yaml_path) {
    YAML::Node root;
    try {
        root = YAML::LoadFile(yaml_path);
    } catch (const std::exception& e) {
        fail("读取 " + yaml_path + " 失败: " + e.what());
    }
    if (!root.IsMap()) fail("根节点必须是 map");

    SimSetup setup;

    // ---- robot ----
    const YAML::Node robot = root["robot"];
    RobotGeometry geom;
    if (robot) {
        geom.armor_count = getI(robot, "armors", geom.armor_count);
        geom.armor_radius = getD(robot, "armor_radius", geom.armor_radius);
        geom.armor_dz = getD(robot, "armor_dz", geom.armor_dz);
        geom.armor_width = getD(robot, "armor_width", geom.armor_width);
        geom.armor_height = getD(robot, "armor_height", geom.armor_height);
        geom.body_radius = getD(robot, "body_radius", geom.body_radius);
    }
    setup.geometry = geom;

    // ---- initial ----
    RobotState init;
    init.p = Vec3(0.0, 0.0, 0.18);
    if (const YAML::Node ini = root["initial"]) {
        init.p = getV3(ini, "position", init.p);

        const double yaw = getD(ini, "yaw", 0.0);
        const double pitch = getD(ini, "pitch", 0.0);
        const double roll = getD(ini, "roll", 0.0);
        init.q = (Quat(Eigen::AngleAxisd(yaw, Vec3::UnitZ())) *
                  Quat(Eigen::AngleAxisd(pitch, Vec3::UnitY())) *
                  Quat(Eigen::AngleAxisd(roll, Vec3::UnitX())))
                     .normalized();

        // 速度沿车体 +x：把给的 world 速度投影到车头上
        const Vec3 vel = getV3(ini, "velocity", Vec3::Zero());
        init.speed = std::max(0.0, vel.dot(init.q * Vec3(1.0, 0.0, 0.0)));

        init.omega = getV3(ini, "omega", Vec3::Zero());
        init.spin_phase = getD(ini, "spin_phase", 0.0);
        init.spin_rate = getD(ini, "spin_rate", 0.0);
    }
    setup.initial = init;

    // ---- 控制来源：preset / plan / random ----
    if (root["preset"]) {
        const YAML::Node pr = root["preset"];
        PresetParams p;
        p.speed = getD(pr, "speed", p.speed);
        p.radius = getD(pr, "radius", p.radius);
        p.accel = getD(pr, "accel", p.accel);
        p.duration = getD(pr, "duration", p.duration);

        const std::string name = getS(pr, "name", "straight");
        auto controls = makePresetControls(name, p);
        if (controls.empty()) {
            std::string known;
            for (const auto& n : presetNames()) known += (known.empty() ? "" : ", ") + n;
            fail("preset.name 未知或 duration<=0: " + name + "（可用: " + known + "）");
        }
        setup.controller = std::make_unique<PlanController>(std::move(controls));
    } else if (root["plan"]) {
        std::vector<Control> plan;
        const YAML::Node list = root["plan"];
        if (!list.IsSequence()) fail("plan 必须是列表");
        for (std::size_t i = 0; i < list.size(); ++i) {
            const YAML::Node seg = list[i];
            if (!seg.IsMap()) fail("plan[" + std::to_string(i) + "] 必须是 map");

            Control c;
            c.accel = getD(seg, "accel", c.accel);
            c.jerk = getD(seg, "jerk", c.jerk);
            c.angular_rate = getAngularRate(seg);
            c.spin_rate = getD(seg, "spin_rate", c.spin_rate);
            c.duration = getD(seg, "duration", c.duration);
            if (!(c.duration > 0.0)) fail("plan[" + std::to_string(i) + "].duration 必须 > 0");
            plan.push_back(c);
        }
        setup.controller = std::make_unique<PlanController>(std::move(plan));
    } else if (root["random"]) {
        const YAML::Node rnd = root["random"];
        RandomController::Config cfg;
        cfg.seed = getU64(rnd, "seed", cfg.seed);
        const auto dur = getRange(rnd, "duration", {cfg.min_duration, cfg.max_duration});
        const auto acc = getRange(rnd, "accel", {cfg.min_accel, cfg.max_accel});
        const auto turn = getRange(rnd, "turn_rate", {cfg.min_turn_rate, cfg.max_turn_rate});
        const auto pitch = getRange(rnd, "pitch_rate", {cfg.min_pitch_rate, cfg.max_pitch_rate});
        const auto roll = getRange(rnd, "roll_rate", {cfg.min_roll_rate, cfg.max_roll_rate});
        const auto spin = getRange(rnd, "spin_rate", {cfg.min_spin_rate, cfg.max_spin_rate});
        cfg.min_duration = dur.first;
        cfg.max_duration = dur.second;
        cfg.min_accel = acc.first;
        cfg.max_accel = acc.second;
        cfg.min_turn_rate = turn.first;
        cfg.max_turn_rate = turn.second;
        cfg.min_pitch_rate = pitch.first;
        cfg.max_pitch_rate = pitch.second;
        cfg.min_roll_rate = roll.first;
        cfg.max_roll_rate = roll.second;
        cfg.min_spin_rate = spin.first;
        cfg.max_spin_rate = spin.second;
        setup.controller = std::make_unique<RandomController>(cfg);
    }

    // ---- sim ----
    if (root["sim"]) setup.min_dt = getD(root["sim"], "min_dt", setup.min_dt);
    if (!(setup.min_dt > 0.0)) fail("sim.min_dt 必须 > 0");

    // ---- sensor ----
    SensorConfig sensor;
    if (const YAML::Node sen = root["sensor"]) {
        sensor.seed = getU64(sen, "seed", sensor.seed);
        sensor.position_sigma = getV3(sen, "position_sigma", sensor.position_sigma);
        sensor.normal_sigma = getD(sen, "normal_sigma", getD(sen, "yaw_sigma", sensor.normal_sigma));
        sensor.dropout = getD(sen, "dropout", sensor.dropout);
        sensor.max_range = getD(sen, "max_range", sensor.max_range);
        sensor.facing_min_cos = getD(sen, "facing_min_cos", sensor.facing_min_cos);

        if (const YAML::Node sch = sen["schedule"]) {
            sensor.t0 = getD(sch, "t0", sensor.t0);
            sensor.interval = getD(sch, "interval", sensor.interval);
            sensor.jitter = getD(sch, "jitter", sensor.jitter);
            if (sch["intervals"]) {
                if (!sch["intervals"].IsSequence()) fail("sensor.schedule.intervals 必须是列表");
                for (std::size_t i = 0; i < sch["intervals"].size(); ++i) {
                    sensor.intervals.push_back(sch["intervals"][i].as<double>());
                }
            }
        }
        if (!(sensor.interval > 0.0)) fail("sensor.schedule.interval 必须 > 0");
        if (sensor.jitter < 0.0) fail("sensor.schedule.jitter 必须 >= 0");

        if (const YAML::Node cam = sen["camera"]) {
            sensor.camera.enabled = getB(cam, "enabled", true);
            sensor.camera.position = getV3(cam, "position", sensor.camera.position);
            sensor.camera.target = getV3(cam, "target", sensor.camera.target);
            sensor.camera.fov_half_deg = getD(cam, "fov_half_deg", sensor.camera.fov_half_deg);
            sensor.camera.output_in_camera_frame = getB(cam, "output_in_camera_frame", false);
        }
    }
    setup.sensor = sensor;

    return setup;
}

}  // namespace sim
