#pragma once

#include <memory>
#include <vector>

#include "sim/primitive.hpp"
#include "sim/primitives.hpp"
#include "sim/robot_model.hpp"
#include "sim/types.hpp"

namespace sim {

/// 模拟器对外的唯一真值接口：任意 t 求值，纯函数、const、可重入。
class IScene {
public:
    virtual ~IScene() = default;
    [[nodiscard]] virtual double duration() const = 0;
    [[nodiscard]] virtual RobotTruth truthAt(double t) const = 0;
};

/// 自转语义：
///   BodyRotation —— 整车绕 z 轴转（RM 小陀螺的真实情况），chassis.yaw 含自转；
///   ArmorOrbit   —— 车体朝向独立，装甲环单独绕中心公转，chassis.yaw 不含自转。
/// 两种语义下装甲环总转角都等于 yaw + phase，所以观测/几何完全一致，
/// 只有 chassis.yaw / yaw_rate 的语义不同。
enum class SpinSemantics { BodyRotation, ArmorOrbit };

/// 场景 = 三条互相独立的分段时间线 + 几何。
/// **没有任何“初始化完成”这一步**：add 只是登记，truthAt(t) 现场折叠计算。
class Scene final : public IScene {
public:
    Scene() = default;
    Scene(const Scene&) = delete;
    Scene& operator=(const Scene&) = delete;
    Scene(Scene&&) noexcept = default;
    Scene& operator=(Scene&&) noexcept = default;
    ~Scene() override = default;

    void setGeometry(const RobotGeometry& g) { geom_ = g; }
    [[nodiscard]] const RobotGeometry& geometry() const noexcept { return geom_; }

    void setSpinSemantics(SpinSemantics s) noexcept { semantics_ = s; }
    [[nodiscard]] SpinSemantics spinSemantics() const noexcept { return semantics_; }

    /// 初值只登记到各条时间线上，不推导任何后续状态。
    void setInitial(const TranslationState& tr, const YawState& y, const PhaseState& s) noexcept {
        trans_.setInitial(tr);
        yaw_.setInitial(y);
        spin_.setInitial(s);
    }

    [[nodiscard]] Timeline<ITranslationPrimitive, TranslationState>& translation() noexcept { return trans_; }
    [[nodiscard]] Timeline<IYawPrimitive, YawState>& yaw() noexcept { return yaw_; }
    [[nodiscard]] Timeline<ISpinPrimitive, PhaseState>& spin() noexcept { return spin_; }

    [[nodiscard]] const Timeline<ITranslationPrimitive, TranslationState>& translation() const noexcept { return trans_; }
    [[nodiscard]] const Timeline<IYawPrimitive, YawState>& yaw() const noexcept { return yaw_; }
    [[nodiscard]] const Timeline<ISpinPrimitive, PhaseState>& spin() const noexcept { return spin_; }

    /// 三条线里最短的那条（空线不参与）。
    [[nodiscard]] double duration() const override;

    [[nodiscard]] RobotTruth truthAt(double t) const override;

    struct Validation {
        std::vector<JunctionReport> translation;
        std::vector<JunctionReport> yaw;
        std::vector<JunctionReport> spin;
        [[nodiscard]] bool anyWarn() const;
    };
    [[nodiscard]] Validation validate(double eps = 1e-9) const;

private:
    RobotGeometry geom_{};
    SpinSemantics semantics_ = SpinSemantics::BodyRotation;

    Timeline<ITranslationPrimitive, TranslationState> trans_;
    Timeline<IYawPrimitive, YawState> yaw_;
    Timeline<ISpinPrimitive, PhaseState> spin_;
};

}  // namespace sim
