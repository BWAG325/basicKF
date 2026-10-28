// 在 kf_compare 的折返/小陀螺场景上，对同一 CV 滤波器做 RTS，比较 r/l/h。
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "kf/models/cv.hpp"
#include "sim/config.hpp"
#include "sim/core/paths.hpp"
#include "sim/sensing/sensor.hpp"
#include "sim/simulator.hpp"

int main(int argc, char** argv)
{
    if (argc < 3) {
        std::cerr << "usage: rts_compare <sim.yaml> <out.csv>\n";
        return 1;
    }
    const std::string path = argv[1];
    const std::string out = argv[2];
    const double duration = 16.0;
    const double sigma = 0.02;
    auto setup = sim::loadSimSetup(path);
    sim::Simulator sim(setup.geometry, setup.initial, std::move(setup.controller), setup.min_dt);
    sim::GaussianSensor sensor(setup.sensor);
    const int armor_num = setup.geometry.armor_count;
    int locked = sensor.pickArmor(sim.truth());
    if (locked < 0) locked = 0;

    const double min_dt = setup.sensor.intervals.empty()
                            ? std::max(1e-3, setup.sensor.interval - setup.sensor.jitter)
                            : *std::min_element(setup.sensor.intervals.begin(), setup.sensor.intervals.end());
    const auto times = sim::makeObservationTimes(
        setup.sensor, static_cast<std::size_t>(duration / min_dt) + 32);

    struct Sample {
        double t;
        Eigen::Vector3d truth;
        Eigen::Vector3d obs;
    };
    std::vector<Sample> ss;
    for (std::size_t k = 0; k < times.size() && times[k] <= duration; ++k) {
        sim.advanceTo(times[k]);
        auto o = sensor.observeArmorAt(sim.truth(), times[k], k, locked, nullptr);
        if (!o.valid) continue;
        ss.push_back({times[k], sim.truth().armors[static_cast<std::size_t>(locked)].p, o.p});
    }
    if (ss.size() < 4) return 1;
    const double dt0 = std::max(1e-6, ss[1].t - ss[0].t);
    const Eigen::Vector3d v0 = (ss[1].obs - ss[0].obs) / dt0;

    kf::CvParams p;
    p.q_xyz = 100.0;
    p.q_yaw = 75.0;
    p.armor_radius = setup.geometry.armor_radius;
    p.armor_num = armor_num;
    p.armor_id = locked;
    p.R = Eigen::Matrix3d::Identity() * sigma * sigma;
    kf::CvKalman f(p);
    f.init(ss[1].obs, v0);
    f.startHistory();
    double last_t = ss[1].t;
    for (std::size_t i = 2; i < ss.size(); ++i) {
        const double dt = std::max(1e-6, ss[i].t - last_t);
        last_t = ss[i].t;
        f.predict(dt);
        f.update(ss[i].obs);
    }
    const auto xs = f.smooth();
    // xs: 对应 ss[1..N-1] 的滤波/平滑状态（最后一个为最后滤波值）
    const double rt = setup.geometry.armor_a > 0 ? setup.geometry.armor_a
                                                  : setup.geometry.armor_radius;
    const double lt = (setup.geometry.armor_b > 0 ? setup.geometry.armor_b : rt) - rt;
    const double ht = -setup.geometry.armor_dh;

    std::ofstream csv(out);
    csv << "t,r_ref,l_ref,h_ref,r_causal,l_causal,h_causal,r_smooth,l_smooth,h_smooth\n";
    for (std::size_t i = 0; i < xs.size(); ++i) {
        const auto& x = xs[i];
        // 因果值没有单独保存，历史里取同一时刻的滤波 state
        const auto& xf = (i < f.history().size()) ? f.history()[i].xf : f.state();
        csv << ss[i + 1].t << ',' << rt << ',' << lt << ',' << ht << ',' << xf[8] << ',' << xf[9]
            << ',' << xf[10] << ',' << x[8] << ',' << x[9] << ',' << x[10] << '\n';
    }
    std::cout << "wrote " << out << " frames=" << xs.size() << '\n';
    std::cout << "final causal r/l/h = " << f.state()[8] << " " << f.state()[9] << " "
              << f.state()[10] << "\n";
    std::cout << "final RTS    r/l/h = " << xs.back()[8] << " " << xs.back()[9] << " "
              << xs.back()[10] << "\n";
    return 0;
}
