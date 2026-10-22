// IMM 的数学检查：单模型退化、同模型对称、混合初值公式。
#include <cmath>
#include <cstdio>
#include <memory>
#include <vector>

#include "kf/imm.hpp"
#include "kf/models/ca.hpp"
#include "kf/models/cv.hpp"
#include "kf/models/cv9.hpp"

namespace {

int failures = 0;

void check(bool ok, const char* what) {
    if (!ok) {
        std::printf("FAIL: %s\n", what);
        ++failures;
    }
}

bool close(double a, double b, double eps = 1e-9) { return std::abs(a - b) <= eps; }

std::unique_ptr<kf::CvKalman> makeCv(const Eigen::Vector3d& p, const Eigen::Vector3d& v, double sigma) {
    auto f = std::make_unique<kf::CvKalman>();
    f->init(p, v, 0.05, 1.5);
    f->setAccelNoise(sigma);
    f->setMeasurementNoise(0.02);
    return f;
}

// 单模型：概率恒为 1，状态和单独跑该滤波器一致。
void testSingleModel() {
    auto f = makeCv(Eigen::Vector3d::Zero(), Eigen::Vector3d(2, 0, 0), 0.5);
    kf::CvKalman ref = *f;

    std::vector<kf::Imm::Model> models;
    models.push_back({"only", std::move(f)});
    kf::Imm imm(std::move(models), Eigen::MatrixXd::Identity(1, 1), Eigen::VectorXd::Ones(1));

    for (int i = 1; i <= 20; ++i) {
        const double dt = 0.04;
        Eigen::Vector3d z(2.0 * i * dt, 0.0, 0.0);

        imm.predict(dt);
        ref.predict(dt);

        imm.update(z);
        ref.update(z);
    }

    check(close(imm.probabilities()[0], 1.0, 1e-12), "单模型概率应为 1");
    check((imm.state() - ref.state()).norm() < 1e-9, "单模型状态应与直接滤波一致");
}

// 两个完全相同的模型：似然相同，概率应保持 0.5/0.5，组合状态等于任一模型。
void testSymmetricModels() {
    auto a = makeCv(Eigen::Vector3d::Zero(), Eigen::Vector3d(2, 0, 0), 0.5);
    auto b = makeCv(Eigen::Vector3d::Zero(), Eigen::Vector3d(2, 0, 0), 0.5);

    const Eigen::Matrix2d Pi = (Eigen::Matrix2d() << 0.9, 0.1, 0.1, 0.9).finished();
    std::vector<kf::Imm::Model> models;
    models.push_back({"a", std::move(a)});
    models.push_back({"b", std::move(b)});
    kf::Imm imm(std::move(models), Pi, Eigen::Vector2d(0.5, 0.5));

    for (int i = 1; i <= 30; ++i) {
        const double dt = 0.04;
        Eigen::Vector3d z(2.0 * i * dt, 0.0, 0.0);

        imm.predict(dt);
        imm.update(z);
    }

    check(close(imm.probabilities()[0], 0.5, 1e-9), "同模型概率应保持 0.5");
    check(close(imm.probabilities()[1], 0.5, 1e-9), "同模型概率应保持 0.5");
    check((imm.modelState(0) - imm.modelState(1)).norm() < 1e-9, "同模型状态应一致");
    check((imm.state() - imm.modelState(0)).norm() < 1e-9, "组合状态应等于模型状态");
}

// 混合公式：dt=0 时只做混合，验证 x⁰_j = Σ_i (p_ij μ_i / c_j) x_i。
void testMixingFormula() {
    const Eigen::Vector3d xa(0.0, 0.0, 0.0);
    const Eigen::Vector3d xb(10.0, 0.0, 0.0);

    auto a = makeCv(xa, Eigen::Vector3d::Zero(), 0.5);
    auto b = makeCv(xb, Eigen::Vector3d::Zero(), 0.5);
    a->setState(Eigen::VectorXd::Zero(6), Eigen::MatrixXd::Zero(6, 6));
    b->setState((Eigen::VectorXd(6) << 10, 0, 0, 0, 0, 0).finished(), Eigen::MatrixXd::Zero(6, 6));

    const Eigen::Matrix2d Pi = (Eigen::Matrix2d() << 0.9, 0.1, 0.2, 0.8).finished();
    const Eigen::Vector2d mu0(0.6, 0.4);

    std::vector<kf::Imm::Model> models;
    models.push_back({"a", std::move(a)});
    models.push_back({"b", std::move(b)});
    kf::Imm imm(std::move(models), Pi, mu0);
    imm.predict(0.0);

    const double c0 = 0.9 * 0.6 + 0.2 * 0.4;  // 0.62
    const double c1 = 0.1 * 0.6 + 0.8 * 0.4;  // 0.38

    const double x00 = (0.9 * 0.6 * xa.x() + 0.2 * 0.4 * xb.x()) / c0;
    const double x01 = (0.1 * 0.6 * xa.x() + 0.8 * 0.4 * xb.x()) / c1;

    check(close(imm.modelState(0)[0], x00, 1e-9), "混合初值 x⁰_0 公式");
    check(close(imm.modelState(1)[0], x01, 1e-9), "混合初值 x⁰_1 公式");
    check(close(imm.probabilities()[0], 0.6, 1e-12), "predict 不应改动上一拍后验概率");
}

// 异构模型：6 维 CV + 9 维 CA，公共空间应为 9 维，投影回各自维度。
void testHeterogeneous() {
    auto cv = makeCv(Eigen::Vector3d::Zero(), Eigen::Vector3d(2, 0, 0), 0.5);
    auto ca = std::make_unique<kf::CaKalman>();
    ca->init(Eigen::Vector3d::Zero(), Eigen::Vector3d(2, 0, 0), Eigen::Vector3d::Zero(), 0.05, 1.5, 2.0);
    ca->setJerkNoise(10.0);
    ca->setMeasurementNoise(0.02);

    const Eigen::Matrix2d Pi = (Eigen::Matrix2d() << 0.9, 0.1, 0.1, 0.9).finished();
    std::vector<kf::Imm::Model> models;
    models.push_back({"cv", std::move(cv)});
    models.push_back({"ca", std::move(ca)});
    kf::Imm imm(std::move(models), Pi, Eigen::Vector2d(0.5, 0.5));

    check(imm.commonDim() == 9, "异构 IMM 公共维度应为 9");

    for (int i = 1; i <= 10; ++i) {
        const double dt = 0.04;
        const Eigen::Vector3d z(2.0 * i * dt, 0.0, 0.0);
        imm.predict(dt);
        imm.update(z);
    }

    check(imm.modelState(0).size() == 6, "CV 模型仍是 6 维");
    check(imm.modelState(1).size() == 9, "CA 模型仍是 9 维");
    check(imm.state().size() == 9, "组合输出在公共空间应为 9 维");
    check((imm.state().head<3>() - Eigen::Vector3d(2.0 * 10 * 0.04, 0, 0)).norm() < 0.05,
          "异构 IMM 位置应收敛到观测");
    check(std::abs(imm.probabilities().sum() - 1.0) < 1e-9, "异构 IMM 概率应归一化");
}

// CV9 + CA：公共空间 9 维；混合后 CV9 的虚拟 a 仍必须为 0。
void testCv9Ca() {
    auto cv9 = std::make_unique<kf::Cv9Kalman>();
    cv9->init(Eigen::Vector3d::Zero(), Eigen::Vector3d(2, 0, 0), 0.05, 1.5);
    cv9->setAccelNoise(0.2);
    cv9->setMeasurementNoise(0.02);

    auto ca = std::make_unique<kf::CaKalman>();
    ca->init(Eigen::Vector3d::Zero(), Eigen::Vector3d(2, 0, 0), Eigen::Vector3d::Zero(), 0.05, 1.5, 2.0);
    ca->setJerkNoise(20.0);
    ca->setMeasurementNoise(0.02);

    const Eigen::Matrix2d Pi = (Eigen::Matrix2d() << 0.95, 0.05, 0.05, 0.95).finished();
    std::vector<kf::Imm::Model> models;
    models.push_back({"cv9", std::move(cv9)});
    models.push_back({"ca", std::move(ca)});
    kf::Imm imm(std::move(models), Pi, Eigen::Vector2d(0.5, 0.5));

    check(imm.commonDim() == 9, "CV9+CA 公共维度应为 9");

    for (int i = 1; i <= 20; ++i) {
        const double dt = 0.04;
        const Eigen::Vector3d z(2.0 * i * dt, 0.0, 0.0);
        imm.predict(dt);
        imm.update(z);
        check(imm.modelState(0).tail<3>().norm() < 1e-12, "混合后 CV9 的虚拟加速度应保持 0");
    }
}

}  // namespace

int main() {
    testSingleModel();
    testSymmetricModels();
    testMixingFormula();
    testHeterogeneous();
    testCv9Ca();

    if (failures == 0) std::printf("all IMM tests passed\n");
    return failures == 0 ? 0 : 1;
}
