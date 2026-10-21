#include "sim/config.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "sim/primitives.hpp"

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

Vec3 getV3(const YAML::Node& n, const char* k, const Vec3& def) {
    if (!n[k]) return def;
    const YAML::Node v = n[k];
    if (!v.IsSequence() || v.size() != 3) fail(std::string("字段 ") + k + " 需要 3 个数");
    return Vec3(v[0].as<double>(), v[1].as<double>(), v[2].as<double>());
}

/// heading 省略或写 "auto" ⇒ 继承段首速度方向。
double getHeading(const YAML::Node& seg) {
    if (!seg["heading"]) return kHeadingInherit;
    const std::string s = seg["heading"].as<std::string>();
    if (s == "auto") return kHeadingInherit;
    try {
        return std::stod(s);
    } catch (...) {
        fail("heading 既不是 auto 也不是数值: " + s);
    }
}

/// speed 省略或写 "auto" ⇒ 继承段首水平速率（保证 C1 连续）。
double getSpeed(const YAML::Node& seg) {
    if (!seg["speed"]) return kSpeedInherit;
    const std::string s = seg["speed"].as<std::string>();
    if (s == "auto") return kSpeedInherit;
    try {
        return std::stod(s);
    } catch (...) {
        fail("speed 既不是 auto 也不是数值: " + s);
    }
}

/// 省略 ⇒ 继承段首角速度。
double getRate(const YAML::Node& seg, const char* key) {
    if (!seg[key]) return kRateInherit;
    try {
        return seg[key].as<double>();
    } catch (...) {
        fail(std::string("字段 ") + key + " 不是数值");
    }
}

std::string getS(const YAML::Node& n, const char* k, const std::string& def) {
    if (!n[k]) return def;
    return n[k].as<std::string>();
}

template <class Fn>
void forEach(const YAML::Node& node, const char* what, Fn&& fn) {
    if (!node) return;
    if (!node.IsSequence()) fail(std::string(what) + " 必须是列表");
    for (std::size_t i = 0; i < node.size(); ++i) {
        if (!node[i].IsMap()) fail(std::string(what) + "[" + std::to_string(i) + "] 必须是 map");
        fn(node[i], i);
    }
}

void warnJunctions(const char* tag, const std::vector<JunctionReport>& rs) {
    for (const auto& r : rs) {
        if (!r.warn) continue;
        std::cerr << "[sim][warn] " << tag << " 段 " << r.segment << " (t=" << r.t
                  << "s) 未声明的不连续:"
                  << " dp=" << r.entry_gap.dp << " dv=" << r.entry_gap.dv
                  << " dyaw=" << r.entry_gap.dyaw << " domega=" << r.entry_gap.domega
                  << " | 段内自造: dp=" << r.internal_gap.dp << " dv=" << r.internal_gap.dv
                  << " dyaw=" << r.internal_gap.dyaw << " domega=" << r.internal_gap.domega << "\n";
    }
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

    Scene scene;

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

        const std::string mode = getS(robot, "spin_mode", "body_rotation");
        if (mode == "body_rotation") {
            scene.setSpinSemantics(SpinSemantics::BodyRotation);
        } else if (mode == "armor_orbit") {
            scene.setSpinSemantics(SpinSemantics::ArmorOrbit);
        } else {
            fail("spin_mode 只能是 body_rotation / armor_orbit，收到: " + mode);
        }
    }
    scene.setGeometry(geom);

    // ---- initial ----
    TranslationState init_tr;
    YawState init_yaw;
    PhaseState init_spin;
    init_tr.p = Vec3(0.0, 0.0, 0.18);
    const YAML::Node init = root["initial"];
    if (init) {
        init_tr.p = getV3(init, "position", init_tr.p);
        init_tr.v = getV3(init, "velocity", init_tr.v);
        init_yaw.yaw = getD(init, "yaw", init_yaw.yaw);
        init_yaw.rate = getD(init, "yaw_rate", init_yaw.rate);
        init_spin.phase = getD(init, "spin_phase", init_spin.phase);
        init_spin.omega = getD(init, "spin_rate", init_spin.omega);
    }
    scene.setInitial(init_tr, init_yaw, init_spin);

    // ---- translation ----
    forEach(root["translation"], "translation", [&](const YAML::Node& seg, std::size_t i) {
        const std::string type = getS(seg, "type", "hold");
        const double dur = getD(seg, "duration", 0.0);

        TranslationState jump;
        if (seg["jump_velocity"]) jump.v = getV3(seg, "jump_velocity", Vec3::Zero());

        std::unique_ptr<ITranslationPrimitive> prim;
        if (type == "hold") {
            prim = std::make_unique<Hold>(dur);
        } else if (type == "line") {
            prim = std::make_unique<Line>(dur, getSpeed(seg), getHeading(seg), getD(seg, "vz", 0.0));
        } else if (type == "arc") {
            prim = std::make_unique<Arc>(dur, getSpeed(seg), getD(seg, "omega", 0.0),
                                         getHeading(seg), getD(seg, "vz", 0.0));
        } else if (type == "accel") {
            prim = std::make_unique<Accel>(dur, getD(seg, "accel", 0.0), getHeading(seg), getD(seg, "vz", 0.0));
        } else if (type == "brake") {
            prim = std::make_unique<Brake>(dur, getD(seg, "decel", 2.0));
        } else {
            fail("translation[" + std::to_string(i) + "] 未知 type: " + type);
        }
        scene.translation().add(std::move(prim), jump);
    });

    // ---- yaw ----
    forEach(root["yaw"], "yaw", [&](const YAML::Node& seg, std::size_t i) {
        const std::string type = getS(seg, "type", "hold");
        const double dur = getD(seg, "duration", 0.0);

        std::unique_ptr<IYawPrimitive> prim;
        if (type == "hold") {
            prim = std::make_unique<YawHold>(dur);
        } else if (type == "rate") {
            prim = std::make_unique<YawRate>(dur, getRate(seg, "rate"));
        } else if (type == "ramp") {
            prim = std::make_unique<YawRamp>(dur, getRate(seg, "from"), getD(seg, "to", 0.0));
        } else {
            fail("yaw[" + std::to_string(i) + "] 未知 type: " + type);
        }
        scene.yaw().add(std::move(prim));
    });

    // ---- spin ----
    forEach(root["spin"], "spin", [&](const YAML::Node& seg, std::size_t i) {
        const std::string type = getS(seg, "type", "off");
        const double dur = getD(seg, "duration", 0.0);

        std::unique_ptr<ISpinPrimitive> prim;
        if (type == "off") {
            prim = std::make_unique<SpinOff>(dur);
        } else if (type == "rate") {
            prim = std::make_unique<SpinRate>(dur, getRate(seg, "rate"));
        } else if (type == "ramp") {
            prim = std::make_unique<SpinRamp>(dur, getRate(seg, "from"), getD(seg, "to", 0.0));
        } else {
            fail("spin[" + std::to_string(i) + "] 未知 type: " + type);
        }
        scene.spin().add(std::move(prim));
    });

    if (!(scene.duration() > 0.0)) fail("场景总时长为 0，检查各段 duration");

    // 三条时间线长度不一致时取最短，并提示（多半是配置笔误）。
    std::vector<std::pair<const char*, double>> durs;
    if (!scene.translation().empty()) durs.emplace_back("translation", scene.translation().duration());
    if (!scene.yaw().empty()) durs.emplace_back("yaw", scene.yaw().duration());
    if (!scene.spin().empty()) durs.emplace_back("spin", scene.spin().duration());
    for (const auto& [name, d] : durs) {
        if (std::abs(d - scene.duration()) > 1e-9) {
            std::cerr << "[sim][warn] " << name << " 总时长 " << d << "s 与最短的 " << scene.duration()
                      << "s 不一致，超出部分会被截断\n";
        }
    }

    const auto validation = scene.validate();
    warnJunctions("translation", validation.translation);
    warnJunctions("yaw", validation.yaw);
    warnJunctions("spin", validation.spin);

    // ---- sensor ----
    SensorConfig sensor;
    const YAML::Node sen = root["sensor"];
    if (sen) {
        sensor.dt = getD(sen, "dt", sensor.dt);
        sensor.seed = getU64(sen, "seed", sensor.seed);
        sensor.position_sigma = getV3(sen, "position_sigma", sensor.position_sigma);
        sensor.yaw_sigma = getD(sen, "yaw_sigma", sensor.yaw_sigma);
        sensor.dropout = getD(sen, "dropout", sensor.dropout);
        sensor.max_range = getD(sen, "max_range", sensor.max_range);
        sensor.facing_min_cos = getD(sen, "facing_min_cos", sensor.facing_min_cos);

        const YAML::Node cam = sen["camera"];
        if (cam) {
            sensor.camera.enabled = getB(cam, "enabled", true);
            sensor.camera.position = getV3(cam, "position", sensor.camera.position);
            sensor.camera.target = getV3(cam, "target", sensor.camera.target);
            sensor.camera.fov_half_deg = getD(cam, "fov_half_deg", sensor.camera.fov_half_deg);
            sensor.camera.output_in_camera_frame = getB(cam, "output_in_camera_frame", false);
        }
    }
    if (sensor.dt <= 0.0) fail("sensor.dt 必须 > 0");

    return SimSetup{std::move(scene), sensor, yaml_path};
}

}  // namespace sim
