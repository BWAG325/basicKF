#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "kf/models/cv.hpp"
#include "sim/config.hpp"
#include "sim/sensing/sensor.hpp"
#include "sim/simulator.hpp"

// 跑一个最简单的 CV 卡尔曼：模拟器出真值/观测，滤波器只吃观测位置。
const std::string keys =
  "{help h usage ? |                                 | 输出命令行参数说明}"
  "{config-path c  | configs/sim/rm_linear.yaml      | yaml配置文件路径}"
  "{duration       | 6.0                               | 运行时长 [s]}"
  "{sigma-accel    | 0.3                               | 过程噪声：加速度 std [m/s²]}"
  "{sigma-pos      | -1                                | 观测噪声 std [m]（<0 用观测自带的 R）}"
  "{warmup         | 20                                | 统计误差前跳过的帧数}"
  "{table t        | false                             | 逐帧打印}"
  "{csv            | false                             | 输出 CSV 到 stdout}";

namespace {

double minInterval(const sim::SensorConfig& cfg) {
    if (!cfg.intervals.empty()) return *std::min_element(cfg.intervals.begin(), cfg.intervals.end());
    return std::max(1e-3, cfg.interval - cfg.jitter);
}

std::string resolveConfigPath(const std::string& in) {
    namespace fs = std::filesystem;
    const std::vector<std::string> candidates = {
        in,
        "../" + in,
        "../../" + in,
#ifdef SIM_SOURCE_DIR
        std::string(SIM_SOURCE_DIR) + "/" + in,
#endif
    };
    for (const auto& c : candidates) {
        if (fs::exists(c)) return c;
    }
    return in;
}

}  // namespace

int main(int argc, char* argv[]) {
    cv::CommandLineParser cli(argc, argv, keys);
    if (cli.has("help")) {
        cli.printMessage();
        return 0;
    }

    const std::string config_path = resolveConfigPath(cli.get<std::string>("config-path"));
    if (!std::filesystem::exists(config_path)) {
        std::cerr << "error: 找不到配置文件: " << config_path << "\n";
        return 1;
    }

    const double duration = cli.get<double>("duration");
    const double sigma_accel = cli.get<double>("sigma-accel");
    const double sigma_pos = cli.get<double>("sigma-pos");
    const int warmup = std::max(0, cli.get<int>("warmup"));
    const bool print_table = cli.get<bool>("table");
    const bool print_csv = cli.get<bool>("csv");

    try {
        auto setup = sim::loadSimSetup(config_path);
        sim::Simulator sim(setup.geometry, setup.initial, std::move(setup.controller), setup.min_dt);
        const sim::GaussianSensor sensor(setup.sensor);

        const std::size_t frame_count =
            static_cast<std::size_t>(duration / minInterval(setup.sensor)) + 16;
        const std::vector<double> times = sim::makeObservationTimes(setup.sensor, frame_count);

        kf::CvKalman filter;
        bool initialized = false;
        bool have_first = false;
        sim::Vec3 first_obs = sim::Vec3::Zero();
        double first_t = 0.0;
        double last_t = 0.0;

        int frames = 0;
        int valid = 0;
        int dropped = 0;
        double err_sum2 = 0.0;
        double nis_sum = 0.0;
        int err_n = 0;
        int nis_n = 0;

        if (print_csv) {
            std::cout << "t,truth_x,truth_y,truth_z,obs_x,obs_y,obs_z,"
                         "est_x,est_y,est_z,err,nis,"
                         "truth_vx,truth_vy,truth_vz,est_vx,est_vy,est_vz\n";
        } else if (print_table) {
            std::cout << "   t     truth(x,y,z)          obs(x,y,z)            "
                         "est(x,y,z)            err[cm]  NIS\n";
        }

        for (std::size_t k = 0; k < times.size(); ++k) {
            if (times[k] > duration + 1e-12) break;

            const sim::RobotTruth truth = sim.advanceTo(times[k]);
            sim::ObservationTruth anno;
            const sim::Observation obs = sensor.observeAt(truth, times[k], k, &anno);
            ++frames;

            if (!obs.valid) {
                ++dropped;
                continue;
            }
            ++valid;

            const sim::Vec3 plate_truth = truth.armors[static_cast<std::size_t>(anno.armor_index)].p;
            const kf::Mat3 R = (sigma_pos >= 0.0) ? kf::Mat3::Identity() * sigma_pos * sigma_pos
                                                  : obs.R_pos;

            // 用前两帧差分出初速，避免“从 0 开始追 2 m/s”的起步超调
            if (!initialized) {
                if (!have_first) {
                    first_obs = obs.p;
                    first_t = times[k];
                    have_first = true;
                    continue;
                }
                const double dt0 = times[k] - first_t;
                const kf::Vec3 v0 = (obs.p - first_obs) / dt0;

                filter = kf::CvKalman(obs.p, v0, 0.05, 1.5);
                filter.setAccelNoise(sigma_accel);
                filter.setMeasurementNoise(R);
                initialized = true;
                last_t = times[k];
                continue;
            }

            const double dt = times[k] - last_t;
            last_t = times[k];

            filter.predict(dt);
            filter.update(obs.p, R);

            const auto estimate = filter.estimate();
            const kf::Vec3 est = estimate.position;
            const double err = (est - plate_truth).norm();
            const double nis = filter.nis();

            if (valid > warmup) {
                err_sum2 += err * err;
                ++err_n;
                nis_sum += nis;
                ++nis_n;
            }

            if (print_csv) {
                std::cout << std::fixed << std::setprecision(6) << times[k] << ',' << plate_truth.x() << ','
                          << plate_truth.y() << ',' << plate_truth.z() << ',' << obs.p.x() << ',' << obs.p.y()
                          << ',' << obs.p.z() << ',' << est.x() << ',' << est.y() << ',' << est.z() << ','
                          << err << ',' << nis << ',' << truth.chassis.v.x() << ',' << truth.chassis.v.y() << ','
                          << truth.chassis.v.z() << ',' << estimate.velocity.x() << ','
                          << estimate.velocity.y() << ',' << estimate.velocity.z() << '\n';
            } else if (print_table) {
                std::cout << std::fixed << std::setprecision(3) << std::setw(6) << times[k] << "  ("
                          << std::setw(6) << plate_truth.x() << ',' << std::setw(6) << plate_truth.y() << ','
                          << std::setw(6) << plate_truth.z() << ")  (" << std::setw(6) << obs.p.x() << ','
                          << std::setw(6) << obs.p.y() << ',' << std::setw(6) << obs.p.z() << ")  ("
                          << std::setw(6) << est.x() << ',' << std::setw(6) << est.y() << ',' << std::setw(6)
                          << est.z() << ")  " << std::setw(6) << err * 100.0 << "  " << std::setw(6) << nis << '\n';
            }
        }

        if (print_csv) return 0;

        const double rmse = err_n > 0 ? std::sqrt(err_sum2 / err_n) : 0.0;
        const double mean_nis = nis_n > 0 ? nis_sum / nis_n : 0.0;

        std::cout << "\nconfig    : " << config_path << "\n"
                  << "duration  : " << duration << " s   frames: " << frames << " (valid " << valid
                  << ", dropped " << dropped << ")\n"
                  << "sigma_a   : " << sigma_accel << " m/s²    sigma_pos: "
                  << (sigma_pos >= 0.0 ? std::to_string(sigma_pos) + " m (覆盖)" : "用观测 R")
                  << "\n"
                  << std::fixed << std::setprecision(2) << "pos RMSE  : " << rmse * 100.0 << " cm   (跳过前 " << warmup
                  << " 帧)\n"
                  << "mean NIS  : " << mean_nis << "   (3 维观测，理论 ≈ 3)\n";

        return 0;
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
}
