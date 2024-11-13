#ifndef SHOOTER_STATE_KEY_CONTROL_HPP
#define SHOOTER_STATE_KEY_CONTROL_HPP

#include "StateMachine.hpp"

#include "GMMotorHandler.hpp"
#include "Dr16.hpp"
#include "GM6020.hpp"
#include "GM3508.hpp"
#include "GM2006.hpp"

#include "State.hpp"

/**
 * @class ShooterStaterRemoteControl
 * @brief 射击键鼠操控模式。
 * @note 射击键鼠控制状态机继承状态机基类。
 */
class ShooterStateKeyControl : public State
{
public:
    ShooterStateKeyControl(GM3508 *LeftFricMotor,
                              GM3508 *RightFricMotor,
                              GM2006 *TriggerMotor) : LeftFricMotor(LeftFricMotor),
                                                      RightFricMotor(RightFricMotor),
                                                      TriggerMotor(TriggerMotor) {};

    /**
     * @brief 析构函数。
     */
    ~ShooterStateKeyControl() {};

    /**
     * @brief 左摩擦轮电机
     */
    GM3508 *LeftFricMotor;

    /**
     * @brief 右摩擦轮电机
     */
    GM3508 *RightFricMotor;

    /**
     * @brief 播弹盘电机
     */
    GM2006 *TriggerMotor;

    void Init() override;

    void Enter() override;

    void Execute() override;

    void Exit() override;
};

#endif
