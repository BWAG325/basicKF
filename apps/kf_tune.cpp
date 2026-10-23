#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "kf/imm.hpp"
#include "kf/models/ca.hpp"
#include "kf/models/cj.hpp"
#include "kf/models/cv.hpp"
#include "kf/models/cv9.hpp"
#include "kf/models/ct.hpp"
#include "kf/models/spin.hpp"
#include "sim/config.hpp"
#include "sim/core/paths.hpp"
#include "sim/sensing/sensor.hpp"
#include "sim/simulator.hpp"

// 统一参数调参：同一套参数在全部工况上评估，选平均（或最差）RMSE 最小的一组。
const std::string keys =
  "{help h usage ? |                              | 输出命令行参数说明}"
  "{config-path c  | configs/kf/tune.yaml         | 调参配置}";

namespace {

struct Sample {
    double t = 0.0;
    Eigen::Vector3d truth = Eigen::Vector3d::Zero();
    Eigen::Vector3d obs = Eigen::Vector3d::Zero();
};

struct Scenario {
    std::string name;
    std::vector<Sample> samples;
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * 0.0004;
};

enum class Kind { CV, CA, CJ, CTRV, CTRA, CTRJ, SPIN, SPINA, IMM_CC, IMM_CV9CA, IMM_CACA };

const char* kindName(Kind k) {
    switch (k) {
        case Kind::CV: return "CV";
        case Kind::CA: return "CA";
        case Kind::CJ: return "CJ";
        case Kind::CTRV: return "CTRV";
        case Kind::CTRA: return "CTRA";
        case Kind::CTRJ: return "CTRJ";
        case Kind::SPIN: return "SPIN";
        case Kind::SPINA: return "SPIN+A";
        case Kind::IMM_CC: return "IMM CV+CV";
        case Kind::IMM_CV9CA: return "IMM CV9+CA";
        case Kind::IMM_CACA: return "IMM CA+CA";
    }
    return "?";
}

struct Params {
    double cv = 1.0;
    double ca = 50.0;
    double cj = 300.0;
    double cv_slow = 0.5;
    double cv_fast = 10.0;
    double cv9 = 0.5;
    double ca_slow = 5.0;
    double ca_fast = 50.0;
    double ct_v = 2.0;
    double ct_a = 5.0;
    double ct_j = 50.0;
    double ct_w = 2.0;
    double ct_alpha = 5.0;
    double ct_z = 0.01;
    double spin_v = 2.0;
    double spin_w = 2.0;
    double spin_alpha = 5.0;
};

struct Runner {
    bool is_imm = false;
    std::unique_ptr<kf::IFilter> single;
    std::unique_ptr<kf::Imm> imm;

    void predict(double dt) { is_imm ? imm->predict(dt) : single->predict(dt); }
    void update(const Eigen::Vector3d& z) { is_imm ? imm->update(z) : single->update(z); }
    Eigen::Vector3d position() const {
        if (is_imm) return imm->state().head<3>();
        return single->position();
    }
};

std::unique_ptr<kf::IFilter> leafCv(double sigma, const Eigen::Vector3d& z,
                                   const Eigen::Vector3d& v, const Eigen::Matrix3d& R) {
    auto f = std::make_unique<kf::CvKalman>();
    f->init(z, v, 0.05, 1.5);
    f->setAccelNoise(sigma);
    f->setMeasurementNoise(R);
    return f;
}

std::unique_ptr<kf::IFilter> leafCv9(double sigma, const Eigen::Vector3d& z,
                                    const Eigen::Vector3d& v, const Eigen::Matrix3d& R) {
    auto f = std::make_unique<kf::Cv9Kalman>();
    f->init(z, v, 0.05, 1.5);
    f->setAccelNoise(sigma);
    f->setMeasurementNoise(R);
    return f;
}

std::unique_ptr<kf::IFilter> leafCa(double jerk, const Eigen::Vector3d& z,
                                   const Eigen::Vector3d& v, const Eigen::Matrix3d& R) {
    auto f = std::make_unique<kf::CaKalman>();
    f->init(z, v, Eigen::Vector3d::Zero(), 0.05, 1.5, 2.0);
    f->setJerkNoise(jerk);
    f->setMeasurementNoise(R);
    return f;
}

std::unique_ptr<kf::IFilter> leafCj(double snap, const Eigen::Vector3d& z,
                                   const Eigen::Vector3d& v, const Eigen::Matrix3d& R) {
    auto f = std::make_unique<kf::CjKalman>();
    f->init(z, v, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0.05, 1.5, 2.0, 5.0);
    f->setSnapNoise(snap);
    f->setMeasurementNoise(R);
    return f;
}

Runner makeRunner(Kind kind, const Params& p, const Scenario& sc) {
    const Eigen::Vector3d z = sc.samples[1].obs;
    const double dt0 = std::max(1e-6, sc.samples[1].t - sc.samples[0].t);
    const Eigen::Vector3d v = (sc.samples[1].obs - sc.samples[0].obs) / dt0;
    const Eigen::Matrix3d& R = sc.R;

    Runner r;
    if (kind == Kind::CV) {
        r.single = leafCv(p.cv, z, v, R);
        return r;
    }
    if (kind == Kind::CA) {
        r.single = leafCa(p.ca, z, v, R);
        return r;
    }
    if (kind == Kind::CJ) {
        r.single = leafCj(p.cj, z, v, R);
        return r;
    }

    if (kind == Kind::SPIN || kind == Kind::SPINA) {
        kf::SpinParams sp;
        sp.armor_radius = 0.22;
        sp.has_alpha = (kind == Kind::SPINA);
        sp.sigma_v = p.spin_v;
        sp.sigma_w = p.spin_w;
        sp.sigma_alpha = p.spin_alpha;
        sp.sigma_z = 0.01;
        sp.R = R;

        auto f = std::make_unique<kf::SpinKalman>(sp);
        f->init(z, v);
        f->setMeasurementNoise(R);
        r.single = std::move(f);
        return r;
    }

    if (kind == Kind::CTRV || kind == Kind::CTRA || kind == Kind::CTRJ) {
        kf::CtParams cp;
        cp.has_a = (kind != Kind::CTRV);
        cp.has_j = (kind == Kind::CTRJ);
        cp.has_alpha = (kind != Kind::CTRV);
        cp.sigma_v = p.ct_v;
        cp.sigma_a = p.ct_a;
        cp.sigma_j = p.ct_j;
        cp.sigma_w = p.ct_w;
        cp.sigma_alpha = p.ct_alpha;
        cp.sigma_z = p.ct_z;
        cp.R = R;

        auto f = std::make_unique<kf::CtKalman>(cp);
        f->init(z, v);
        f->setMeasurementNoise(R);
        r.single = std::move(f);
        return r;
    }

    std::vector<kf::Imm::Model> models;
    if (kind == Kind::IMM_CC) {
        models.push_back({"slow", leafCv(p.cv_slow, z, v, R)});
        models.push_back({"fast", leafCv(p.cv_fast, z, v, R)});
    } else if (kind == Kind::IMM_CV9CA) {
        models.push_back({"cv9", leafCv9(p.cv9, z, v, R)});
        models.push_back({"ca", leafCa(p.ca_fast, z, v, R)});
    } else {
        models.push_back({"ca-slow", leafCa(p.ca_slow, z, v, R)});
        models.push_back({"ca-fast", leafCa(p.ca_fast, z, v, R)});
    }

    Eigen::Matrix2d Pi;
    Pi << 0.95, 0.05, 0.05, 0.95;
    r.imm = std::make_unique<kf::Imm>(std::move(models), Pi, Eigen::Vector2d(0.5, 0.5));
    r.is_imm = true;
    return r;
}

double runScenario(Kind kind, const Params& p, const Scenario& sc, int warmup) {
    Runner r = makeRunner(kind, p, sc);
    double se = 0.0;
    int n = 0;
    double last_t = sc.samples[1].t;

    for (std::size_t i = 2; i < sc.samples.size(); ++i) {
        const double dt = std::max(1e-6, sc.samples[i].t - last_t);
        last_t = sc.samples[i].t;
        r.predict(dt);
        r.update(sc.samples[i].obs);

        if (static_cast<int>(i) < warmup + 2) continue;
        const Eigen::Vector3d e = r.position() - sc.samples[i].truth;
        se += e.squaredNorm();
        ++n;
    }
    return std::sqrt(se / std::max(1, n)) * 100.0;
}

struct Score {
    Params params;
    std::vector<double> per;
    double objective = std::numeric_limits<double>::infinity();
    double mean = 0.0;
    double worst = 0.0;
};

Score evaluate(Kind kind, const Params& p, const std::vector<Scenario>& scenarios, int warmup,
               bool use_max) {
    Score s;
    s.params = p;
    s.per.reserve(scenarios.size());
    for (const Scenario& sc : scenarios) s.per.push_back(runScenario(kind, p, sc, warmup));

    double sum = 0.0;
    double worst = 0.0;
    for (double v : s.per) {
        sum += v;
        worst = std::max(worst, v);
    }
    s.mean = sum / static_cast<double>(s.per.size());
    s.worst = worst;
    s.objective = use_max ? worst : s.mean;
    return s;
}

std::vector<double> vec(const YAML::Node& n, std::vector<double> def) {
    if (!n) return def;
    std::vector<double> out;
    for (const auto& v : n) out.push_back(v.as<double>());
    return out.empty() ? def : out;
}

std::string fmt(double v) {
    std::ostringstream os;
    os << std::setprecision(3) << v;
    return os.str();
}

std::string jsonEscape(const std::string& s) {
    std::string out;
    for (char c : s) {
        if (c == '"' || c == '\\') out.push_back('\\');
        out.push_back(c);
    }
    return out;
}

}  // namespace

int main(int argc, char* argv[]) {
    cv::CommandLineParser cli(argc, argv, keys);
    if (cli.has("help")) {
        cli.printMessage();
        return 0;
    }

    const std::string cfg_path = sim::resolvePath(cli.get<std::string>("config-path"));
    try {
        const YAML::Node cfg = YAML::LoadFile(cfg_path);
        const int warmup = cfg["warmup"] ? cfg["warmup"].as<int>() : 25;
        const double meas_sigma = cfg["measurement_sigma"] ? cfg["measurement_sigma"].as<double>() : 0.02;
        const bool use_max = cfg["objective"] && cfg["objective"].as<std::string>() == "max";
        const std::string json_path = sim::resolvePath(
            cfg["output"] && cfg["output"]["json"] ? cfg["output"]["json"].as<std::string>()
                                                   : "build/kf/tune_result.json");

        // ---- 场景：每个只采一次观测 ----
        std::vector<Scenario> scenarios;
        for (const auto& n : cfg["scenarios"]) {
            Scenario sc;
            sc.name = n["name"] ? n["name"].as<std::string>() : "scenario";
            const double duration = n["duration"] ? n["duration"].as<double>() : 16.0;
            const std::string sim_path = sim::resolvePath(n["path"].as<std::string>());

            auto setup = sim::loadSimSetup(sim_path);
            if (cfg["sensor_sigma"]) {
                setup.sensor.position_sigma = Eigen::Vector3d::Constant(cfg["sensor_sigma"].as<double>());
            }
            sim::Simulator sim(setup.geometry, setup.initial, std::move(setup.controller), setup.min_dt);
            const sim::GaussianSensor sensor(setup.sensor);
            int locked = sensor.pickArmor(sim.truth());
            if (locked < 0) locked = 0;

            const double min_interval = [&] {
                if (!setup.sensor.intervals.empty())
                    return *std::min_element(setup.sensor.intervals.begin(), setup.sensor.intervals.end());
                return std::max(1e-3, setup.sensor.interval - setup.sensor.jitter);
            }();
            const std::size_t count = static_cast<std::size_t>(duration / min_interval) + 16;
            const std::vector<double> times = sim::makeObservationTimes(setup.sensor, count);

            for (std::size_t k = 0; k < times.size(); ++k) {
                if (times[k] > duration + 1e-9) break;
                sim.advanceTo(times[k]);
                sim::ObservationTruth anno;
                const sim::Observation obs = sensor.observeArmorAt(sim.truth(), times[k], k, locked, &anno);
                if (!obs.valid) continue;
                sc.samples.push_back(Sample{times[k],
                                            sim.truth().armors[static_cast<std::size_t>(locked)].p,
                                            obs.p});
            }
            sc.R = Eigen::Matrix3d::Identity() * meas_sigma * meas_sigma;
            if (sc.samples.size() < 3) throw std::runtime_error("场景观测太少: " + sc.name);
            scenarios.push_back(std::move(sc));
        }

        const YAML::Node g = cfg["grids"];
        const auto cv_grid = vec(g ? g["cv"] : YAML::Node(), {0.1, 0.2, 0.5, 1, 2, 5, 10});
        const auto ca_grid = vec(g ? g["ca_jerk"] : YAML::Node(), {5, 10, 20, 50, 100, 200});
        const auto cj_grid = vec(g ? g["cj_snap"] : YAML::Node(), {50, 100, 200, 500, 1000, 2000});
        const auto slow_grid = vec(g ? g["cv_slow"] : YAML::Node(), {0.1, 0.2, 0.5, 1.0});
        const auto fast_grid = vec(g ? g["cv_fast"] : YAML::Node(), {2, 5, 10, 20, 50});
        const auto cv9_grid = vec(g ? g["cv9"] : YAML::Node(), {0.1, 0.2, 0.5});
        const auto ca_slow_grid = vec(g ? g["ca_slow"] : YAML::Node(), {1, 2, 5});
        const auto ca_fast_grid = vec(g ? g["ca_fast"] : YAML::Node(), {20, 50, 100});

        struct Out {
            Kind kind;
            Score best;
            std::vector<double> per_best;
            std::string params_label;
        };
        std::vector<Out> outs;

        auto record = [&](Kind k, const Score& best, const std::string& label,
                          const std::vector<double>& per_best) {
            outs.push_back(Out{k, best, per_best, label});
        };

        // 逐场景最优（用于看“统一参数”损失多少）
        auto per_scene_best = [&](const std::vector<Score>& all) {
            std::vector<double> best(scenarios.size(), std::numeric_limits<double>::infinity());
            for (const Score& s : all)
                for (std::size_t i = 0; i < best.size(); ++i) best[i] = std::min(best[i], s.per[i]);
            return best;
        };

        // ---- CV ----
        {
            std::vector<Score> all;
            Score best;
            for (double s : cv_grid) {
                Params p;
                p.cv = s;
                Score sc = evaluate(Kind::CV, p, scenarios, warmup, use_max);
                all.push_back(sc);
                if (sc.objective < best.objective) best = sc;
            }
            record(Kind::CV, best, "sigma=" + fmt(best.params.cv), per_scene_best(all));
        }

        // ---- CA ----
        {
            std::vector<Score> all;
            Score best;
            for (double j : ca_grid) {
                Params p;
                p.ca = j;
                Score sc = evaluate(Kind::CA, p, scenarios, warmup, use_max);
                all.push_back(sc);
                if (sc.objective < best.objective) best = sc;
            }
            record(Kind::CA, best, "jerk=" + fmt(best.params.ca), per_scene_best(all));
        }

        // ---- CJ ----
        {
            std::vector<Score> all;
            Score best;
            for (double s : cj_grid) {
                Params p;
                p.cj = s;
                Score sc = evaluate(Kind::CJ, p, scenarios, warmup, use_max);
                all.push_back(sc);
                if (sc.objective < best.objective) best = sc;
            }
            record(Kind::CJ, best, "snap=" + fmt(best.params.cj), per_scene_best(all));
        }

        const auto ct_v_grid = vec(g ? g["ct_v"] : YAML::Node(), {0.5, 1, 2, 5});
        const auto ct_w_grid = vec(g ? g["ct_w"] : YAML::Node(), {0.5, 1, 2, 5});
        const auto ct_a_grid = vec(g ? g["ct_a"] : YAML::Node(), {2, 5, 10});
        const auto ct_alpha_grid = vec(g ? g["ct_alpha"] : YAML::Node(), {2, 5, 10});
        const auto ct_j_grid = vec(g ? g["ct_j"] : YAML::Node(), {20, 50, 100});

        const auto spin_v_grid = vec(g ? g["spin_v"] : YAML::Node(), {0.5, 1, 2, 5});
        const auto spin_w_grid = vec(g ? g["spin_w"] : YAML::Node(), {0.5, 1, 2, 5});
        const auto spin_alpha_grid = vec(g ? g["spin_alpha"] : YAML::Node(), {2, 5, 10});

        // ---- SPIN ----
        {
            std::vector<Score> all;
            Score best;
            for (double sv : spin_v_grid)
                for (double sw : spin_w_grid) {
                    Params p;
                    p.spin_v = sv;
                    p.spin_w = sw;
                    Score sc = evaluate(Kind::SPIN, p, scenarios, warmup, use_max);
                    all.push_back(sc);
                    if (sc.objective < best.objective) best = sc;
                }
            record(Kind::SPIN, best,
                   "spin_v=" + fmt(best.params.spin_v) + ", spin_w=" + fmt(best.params.spin_w),
                   per_scene_best(all));
        }

        // ---- SPIN+A ----
        {
            std::vector<Score> all;
            Score best;
            for (double sw : spin_w_grid)
                for (double salpha : spin_alpha_grid) {
                    Params p;
                    p.spin_w = sw;
                    p.spin_alpha = salpha;
                    Score sc = evaluate(Kind::SPINA, p, scenarios, warmup, use_max);
                    all.push_back(sc);
                    if (sc.objective < best.objective) best = sc;
                }
            record(Kind::SPINA, best,
                   "spin_w=" + fmt(best.params.spin_w) + ", spin_alpha=" + fmt(best.params.spin_alpha),
                   per_scene_best(all));
        }

        // ---- CTRV ----
        {
            std::vector<Score> all;
            Score best;
            for (double sv : ct_v_grid)
                for (double sw : ct_w_grid) {
                    Params p;
                    p.ct_v = sv;
                    p.ct_w = sw;
                    Score sc = evaluate(Kind::CTRV, p, scenarios, warmup, use_max);
                    all.push_back(sc);
                    if (sc.objective < best.objective) best = sc;
                }
            record(Kind::CTRV, best,
                   "sigma_v=" + fmt(best.params.ct_v) + ", sigma_w=" + fmt(best.params.ct_w),
                   per_scene_best(all));
        }

        // ---- CTRA ----
        {
            std::vector<Score> all;
            Score best;
            for (double sa : ct_a_grid)
                for (double salpha : ct_alpha_grid) {
                    Params p;
                    p.ct_a = sa;
                    p.ct_alpha = salpha;
                    Score sc = evaluate(Kind::CTRA, p, scenarios, warmup, use_max);
                    all.push_back(sc);
                    if (sc.objective < best.objective) best = sc;
                }
            record(Kind::CTRA, best,
                   "sigma_a=" + fmt(best.params.ct_a) + ", sigma_alpha=" + fmt(best.params.ct_alpha),
                   per_scene_best(all));
        }

        // ---- CTRJ ----
        {
            std::vector<Score> all;
            Score best;
            for (double sj : ct_j_grid)
                for (double salpha : ct_alpha_grid) {
                    Params p;
                    p.ct_j = sj;
                    p.ct_alpha = salpha;
                    Score sc = evaluate(Kind::CTRJ, p, scenarios, warmup, use_max);
                    all.push_back(sc);
                    if (sc.objective < best.objective) best = sc;
                }
            record(Kind::CTRJ, best,
                   "sigma_j=" + fmt(best.params.ct_j) + ", sigma_alpha=" + fmt(best.params.ct_alpha),
                   per_scene_best(all));
        }

        // ---- IMM CV+CV ----
        {
            std::vector<Score> all;
            Score best;
            for (double slow : slow_grid)
                for (double fast : fast_grid) {
                    Params p;
                    p.cv_slow = slow;
                    p.cv_fast = fast;
                    Score sc = evaluate(Kind::IMM_CC, p, scenarios, warmup, use_max);
                    all.push_back(sc);
                    if (sc.objective < best.objective) best = sc;
                }
            record(Kind::IMM_CC, best,
                   "sigma_slow=" + fmt(best.params.cv_slow) +
                       ", sigma_fast=" + fmt(best.params.cv_fast),
                   per_scene_best(all));
        }

        // ---- IMM CV9+CA ----
        {
            std::vector<Score> all;
            Score best;
            for (double s9 : cv9_grid)
                for (double j : ca_fast_grid) {
                    Params p;
                    p.cv9 = s9;
                    p.ca_fast = j;
                    Score sc = evaluate(Kind::IMM_CV9CA, p, scenarios, warmup, use_max);
                    all.push_back(sc);
                    if (sc.objective < best.objective) best = sc;
                }
            record(Kind::IMM_CV9CA, best,
                   "cv9_sigma=" + fmt(best.params.cv9) +
                       ", ca_jerk=" + fmt(best.params.ca_fast),
                   per_scene_best(all));
        }

        // ---- IMM CA+CA ----
        {
            std::vector<Score> all;
            Score best;
            for (double slow : ca_slow_grid)
                for (double fast : ca_fast_grid) {
                    Params p;
                    p.ca_slow = slow;
                    p.ca_fast = fast;
                    Score sc = evaluate(Kind::IMM_CACA, p, scenarios, warmup, use_max);
                    all.push_back(sc);
                    if (sc.objective < best.objective) best = sc;
                }
            record(Kind::IMM_CACA, best,
                   "jerk_slow=" + fmt(best.params.ca_slow) +
                       ", jerk_fast=" + fmt(best.params.ca_fast),
                   per_scene_best(all));
        }

        // ---- 输出 ----
        std::filesystem::create_directories(std::filesystem::path(json_path).parent_path());
        std::ofstream js(json_path);
        js << "{\n  \"objective\": \"" << (use_max ? "max" : "mean") << "\",\n"
           << "  \"warmup\": " << warmup << ",\n  \"measurement_sigma\": " << meas_sigma << ",\n"
           << "  \"scenarios\": [";
        for (std::size_t i = 0; i < scenarios.size(); ++i) {
            js << (i ? ", " : "") << "\"" << jsonEscape(scenarios[i].name) << "\"";
        }
        js << "],\n  \"results\": [\n";

        for (std::size_t k = 0; k < outs.size(); ++k) {
            const Out& o = outs[k];
            js << "    {\"model\": \"" << jsonEscape(kindName(o.kind)) << "\", \"params\": \""
               << jsonEscape(o.params_label) << "\", \"mean\": " << o.best.mean
               << ", \"worst\": " << o.best.worst << ", \"per_scenario\": [";
            for (std::size_t i = 0; i < o.best.per.size(); ++i) js << (i ? ", " : "") << o.best.per[i];
            js << "], \"per_scenario_best\": [";
            for (std::size_t i = 0; i < o.per_best.size(); ++i) js << (i ? ", " : "") << o.per_best[i];
            js << "]}" << (k + 1 == outs.size() ? "\n" : ",\n");
        }
        js << "  ]\n}\n";

        std::cout << std::fixed << std::setprecision(2);
        std::cout << "scenarios: ";
        for (const auto& sc : scenarios) std::cout << sc.name << " ";
        std::cout << "\nobjective: " << (use_max ? "max" : "mean") << "\n\n";

        std::cout << std::left << std::setw(14) << "model" << std::setw(48) << "best common params"
                  << std::setw(9) << "mean" << std::setw(9) << "worst" << "per-scenario\n";
        for (const Out& o : outs) {
            std::cout << std::left << std::setw(14) << kindName(o.kind) << std::setw(48)
                      << o.params_label << std::setw(9) << o.best.mean << std::setw(9) << o.best.worst;
            for (double v : o.best.per) std::cout << v << " ";
            std::cout << '\n';
        }

        std::cout << "\nper-scenario best (headroom):\n";
        for (const Out& o : outs) {
            std::cout << std::left << std::setw(14) << kindName(o.kind);
            for (double v : o.per_best) std::cout << v << " ";
            std::cout << '\n';
        }
        std::cout << "\njson: " << json_path << "\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
}
