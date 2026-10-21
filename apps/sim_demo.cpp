#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>

#include "sim/config.hpp"
#include "sim/sensor.hpp"

namespace {

void usage() {
    std::cerr << "用法:\n"
                 "  sim_demo --config <yaml> [--t <sec>]\n"
                 "  sim_demo --config <yaml> --dump <t0> <t1> <dt>\n\n"
                 "  --t      打印单个时刻的真值与观测\n"
                 "  --dump   按固定步长输出 CSV 到 stdout\n";
}

void printTruth(const sim::RobotTruth& tr) {
    std::cout << std::fixed << std::setprecision(5);
    std::cout << "  chassis : p=(" << tr.chassis.p.x() << ", " << tr.chassis.p.y() << ", " << tr.chassis.p.z()
              << ")  v=(" << tr.chassis.v.x() << ", " << tr.chassis.v.y() << ", " << tr.chassis.v.z()
              << ")  a=(" << tr.chassis.a.x() << ", " << tr.chassis.a.y() << ", " << tr.chassis.a.z() << ")\n"
              << "            yaw=" << tr.chassis.yaw << " rad  yaw_rate=" << tr.chassis.yaw_rate
              << "  yaw_acc=" << tr.chassis.yaw_acc << "\n"
              << "  spin    : phase=" << tr.spin_phase << " rad (raw " << tr.spin_phase_raw
              << ")  rate=" << tr.spin_rate << " rad/s\n";
    for (int i = 0; i < tr.armor_count; ++i) {
        const auto& a = tr.armors[static_cast<std::size_t>(i)];
        std::cout << "  armor[" << i << "]: p=(" << a.p.x() << ", " << a.p.y() << ", " << a.p.z()
                  << ")  n=(" << a.n.x() << ", " << a.n.y() << ", " << a.n.z()
                  << ")  yaw=" << a.yaw << "\n";
    }
}

void printObservation(const sim::Observation& obs, const sim::ObservationTruth& anno) {
    std::cout << std::fixed << std::setprecision(5);
    std::cout << "  obs     : valid=" << (obs.valid ? 1 : 0) << " reason=" << sim::toString(obs.reason)
              << (obs.in_camera_frame ? " frame=camera" : " frame=world") << "\n";
    if (!obs.valid) return;
    std::cout << "            p=(" << obs.p.x() << ", " << obs.p.y() << ", " << obs.p.z()
              << ")  yaw=" << obs.yaw << " rad\n"
              << "            truth_armor=" << anno.armor_index << "  facing=" << anno.facing_cos
              << "  range=" << anno.range << "\n"
              << "            sigma_pos=(" << std::sqrt(obs.R_pos(0, 0)) << ", " << std::sqrt(obs.R_pos(1, 1))
              << ", " << std::sqrt(obs.R_pos(2, 2)) << ")  sigma_yaw=" << std::sqrt(obs.R_yaw) << "\n";
}

}  // namespace

int main(int argc, char** argv) {
    std::string config_path;
    double t_query = 0.0;
    bool dump = false;
    double t0 = 0.0;
    double t1 = 0.0;
    double dt = 0.0;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--config" && i + 1 < argc) {
            config_path = argv[++i];
        } else if (a == "--t" && i + 1 < argc) {
            t_query = std::stod(argv[++i]);
        } else if (a == "--dump" && i + 3 < argc) {
            dump = true;
            t0 = std::stod(argv[++i]);
            t1 = std::stod(argv[++i]);
            dt = std::stod(argv[++i]);
        } else if (a == "-h" || a == "--help") {
            usage();
            return 0;
        } else {
            std::cerr << "未知参数: " << a << "\n";
            usage();
            return 2;
        }
    }

    if (config_path.empty()) {
        usage();
        return 2;
    }
    if (dump && !(dt > 0.0)) {
        std::cerr << "--dump 的步长必须 > 0\n";
        return 2;
    }

    try {
        auto setup = sim::loadSimSetup(config_path);
        const sim::GaussianSensor sensor(setup.sensor);
        const sim::Scene& scene = setup.scene;

        if (dump) {
            std::cout << "t,p_x,p_y,p_z,vx,vy,vz,yaw,spin_phase,spin_rate,"
                         "obs_valid,obs_reason,obs_x,obs_y,obs_z,obs_yaw,"
                         "truth_armor,facing,range\n";
            for (double t = t0; t <= t1 + 1e-12; t += dt) {
                const sim::RobotTruth tr = scene.truthAt(t);
                sim::ObservationTruth anno;
                const sim::Observation obs = sensor.observeAt(tr, t, &anno);
                std::cout << std::fixed << std::setprecision(6) << t << ',' << tr.chassis.p.x() << ','
                          << tr.chassis.p.y() << ',' << tr.chassis.p.z() << ',' << tr.chassis.v.x() << ','
                          << tr.chassis.v.y() << ',' << tr.chassis.v.z() << ',' << tr.chassis.yaw << ','
                          << tr.spin_phase << ',' << tr.spin_rate << ',' << (obs.valid ? 1 : 0) << ','
                          << sim::toString(obs.reason) << ',' << obs.p.x() << ',' << obs.p.y() << ',' << obs.p.z()
                          << ',' << obs.yaw << ',' << anno.armor_index << ',' << anno.facing_cos << ','
                          << anno.range << '\n';
            }
            return 0;
        }

        std::cout << "config   : " << setup.source_path << "\n"
                  << "duration : " << std::fixed << std::setprecision(3) << scene.duration() << " s\n"
                  << "armors   : " << scene.geometry().armor_count
                  << "  radius=" << scene.geometry().armor_radius
                  << "  dz=" << scene.geometry().armor_dz << "\n"
                  << "spin_mode: "
                  << (scene.spinSemantics() == sim::SpinSemantics::BodyRotation ? "body_rotation" : "armor_orbit")
                  << "\n"
                  << "sample   : dt=" << setup.sensor.dt << " s  seed=" << setup.sensor.seed
                  << "  dropout=" << setup.sensor.dropout << "\n\n";

        const double t = std::min(std::max(t_query, 0.0), scene.duration());
        const sim::RobotTruth tr = scene.truthAt(t);
        sim::ObservationTruth anno;
        const sim::Observation obs = sensor.observeAt(tr, t, &anno);

        std::cout << "t = " << t << " s\n";
        printTruth(tr);
        printObservation(obs, anno);
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
}
