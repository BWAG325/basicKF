#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "sim/config.hpp"
#include "sim/core/paths.hpp"
#include "sim/sensing/sensor.hpp"
#include "sim/simulator.hpp"

// 与 auto_aim_/tests 一致的参数传入方式：cv::CommandLineParser + keys 默认值。
// CLion 里直接运行本目标（不填 Program arguments）即可。
const std::string keys =
  "{help h usage ? |                                   | 输出命令行参数说明}"
  "{config-path c  | configs/sim/rm_straight_spin.yaml | yaml配置文件路径 }"
  "{t              | 2.2                               | 推进到该时刻并打印一帧（<0 关闭）}"
  "{duration       | 0                                 | >0 时按观测时刻表输出 CSV}"
  "{min-dt         | -1                                | 积分步长上限（<0 用 yaml）}";

namespace {

void printTruth(const sim::RobotTruth& tr) {
    std::cout << std::fixed << std::setprecision(5);
    std::cout << "  chassis : p=(" << tr.chassis.p.x() << ", " << tr.chassis.p.y() << ", " << tr.chassis.p.z()
              << ")  v=(" << tr.chassis.v.x() << ", " << tr.chassis.v.y() << ", " << tr.chassis.v.z()
              << ")  |v|=" << tr.chassis.v.norm() << "\n"
              << "            yaw=" << tr.chassis.yaw << "  pitch=" << tr.chassis.pitch
              << "  roll=" << tr.chassis.roll << " rad\n"
              << "            omega=(" << tr.chassis.omega.x() << ", " << tr.chassis.omega.y() << ", "
              << tr.chassis.omega.z() << ") rad/s\n"
              << "            q=(" << tr.chassis.q.w() << ", " << tr.chassis.q.x() << ", " << tr.chassis.q.y()
              << ", " << tr.chassis.q.z() << ")\n";
    std::cout << "  spin    : phase=" << tr.spin_phase << " rad (raw " << tr.spin_phase_raw
              << ")  rate=" << tr.spin_rate << " rad/s\n";
    for (int i = 0; i < tr.armor_count; ++i) {
        const auto& a = tr.armors[static_cast<std::size_t>(i)];
        std::cout << "  armor[" << i << "]: p=(" << a.p.x() << ", " << a.p.y() << ", " << a.p.z()
                  << ")  n=(" << a.n.x() << ", " << a.n.y() << ", " << a.n.z()
                  << ")  up=(" << a.up.x() << ", " << a.up.y() << ", " << a.up.z() << ")\n";
    }
}

void printObservation(const sim::Observation& obs, const sim::ObservationTruth& anno) {
    std::cout << std::fixed << std::setprecision(5);
    std::cout << "  obs     : valid=" << (obs.valid ? 1 : 0) << " reason=" << sim::toString(obs.reason)
              << (obs.in_camera_frame ? " frame=camera" : " frame=world") << "\n";
    if (!obs.valid) return;
    std::cout << "            p=(" << obs.p.x() << ", " << obs.p.y() << ", " << obs.p.z()
              << ")  n=(" << obs.n.x() << ", " << obs.n.y() << ", " << obs.n.z() << ")\n"
              << "            truth_armor=" << anno.armor_index << "  facing=" << anno.facing_cos
              << "  range=" << anno.range << "\n";
}

std::size_t frameIndexAt(const std::vector<double>& times, double t) {
    const auto it = std::upper_bound(times.begin(), times.end(), t);
    if (it == times.begin()) return 0;
    return static_cast<std::size_t>(it - times.begin() - 1);
}

double minInterval(const sim::SensorConfig& cfg) {
    if (!cfg.intervals.empty()) return *std::min_element(cfg.intervals.begin(), cfg.intervals.end());
    return std::max(1e-3, cfg.interval - cfg.jitter);
}

/// CLion 的工作目录可能是工程根目录，也可能是 build 目录：两种都试，最后退到源码目录。

}  // namespace

int main(int argc, char* argv[]) {
    cv::CommandLineParser cli(argc, argv, keys);
    if (cli.has("help")) {
        cli.printMessage();
        std::cout << "\n注意：OpenCV 参数用等号，例如\n"
                     "  sim_demo -c=configs/sim/rm_random.yaml --t=1.0\n"
                     "  sim_demo --duration=2.0\n";
        return 0;
    }

    const std::string config_path = sim::resolvePath(cli.get<std::string>("config-path"));
    const double t_print = cli.get<double>("t");
    const double duration = cli.get<double>("duration");
    const double min_dt_arg = cli.get<double>("min-dt");

    if (!std::filesystem::exists(config_path)) {
        std::cerr << "error: 找不到配置文件: " << config_path << "\n"
                  << "提示：OpenCV 参数要写成 key=value，例如\n"
                  << "  sim_demo -c=configs/sim/rm_random.yaml --t=1.0\n";
        return 1;
    }

    try {
        auto setup = sim::loadSimSetup(config_path);
        const double dt_res = (min_dt_arg > 0.0) ? min_dt_arg : setup.min_dt;

        sim::Simulator sim(setup.geometry, setup.initial, std::move(setup.controller), dt_res);
        const sim::GaussianSensor sensor(setup.sensor);

        if (duration > 0.0) {
            const std::size_t frames = static_cast<std::size_t>(duration / minInterval(setup.sensor)) + 8;
            const std::vector<double> times = sim::makeObservationTimes(setup.sensor, frames);

            std::cout << "t,frame_dt,p_x,p_y,p_z,vx,vy,vz,yaw,pitch,roll,spin_phase,spin_rate,"
                         "obs_valid,obs_reason,obs_x,obs_y,obs_z,obs_nx,obs_ny,obs_nz,"
                         "truth_armor,facing,range\n";

            for (std::size_t k = 0; k < times.size(); ++k) {
                const double t = times[k];
                if (t > duration + 1e-12) break;

                const sim::RobotTruth tr = sim.advanceTo(t);
                sim::ObservationTruth anno;
                const sim::Observation obs = sensor.observeAt(tr, t, k, &anno);

                const double frame_dt = (k == 0) ? 0.0 : times[k] - times[k - 1];
                std::cout << std::fixed << std::setprecision(6) << t << ',' << frame_dt << ','
                          << tr.chassis.p.x() << ',' << tr.chassis.p.y() << ',' << tr.chassis.p.z() << ','
                          << tr.chassis.v.x() << ',' << tr.chassis.v.y() << ',' << tr.chassis.v.z() << ','
                          << tr.chassis.yaw << ',' << tr.chassis.pitch << ',' << tr.chassis.roll << ','
                          << tr.spin_phase << ',' << tr.spin_rate << ',' << (obs.valid ? 1 : 0) << ','
                          << sim::toString(obs.reason) << ',' << obs.p.x() << ',' << obs.p.y() << ','
                          << obs.p.z() << ',' << obs.n.x() << ',' << obs.n.y() << ',' << obs.n.z() << ','
                          << anno.armor_index << ',' << anno.facing_cos << ',' << anno.range << '\n';
            }
            return 0;
        }

        if (t_print < 0.0) {
            cli.printMessage();
            return 0;
        }

        const std::size_t frames = static_cast<std::size_t>(t_print / minInterval(setup.sensor)) + 8;
        const std::vector<double> times = sim::makeObservationTimes(setup.sensor, frames);

        sim.advanceTo(t_print);
        const sim::RobotTruth tr = sim.truth();
        const std::size_t k = frameIndexAt(times, t_print);

        sim::ObservationTruth anno;
        const sim::Observation obs = sensor.observeAt(tr, t_print, k, &anno);

        std::cout << "config   : " << config_path << "\n"
                  << "min_dt   : " << std::fixed << std::setprecision(6) << dt_res << " s\n"
                  << "frame    : #" << k << "  (schedule dt="
                  << (k + 1 < times.size() ? times[k + 1] - times[k] : 0.0) << " s)\n\n"
                  << "t = " << std::fixed << std::setprecision(5) << sim.time() << " s\n";
        printTruth(tr);
        printObservation(obs, anno);
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
}
