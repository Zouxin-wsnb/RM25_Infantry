#ifndef SHOOTER_STATE_RELAX_HPP
#define SHOOTER_STATE_RELAX_HPP

#include "StateMachine.hpp"

#include "GMMotorHandler.hpp"
#include "Dr16.hpp"
#include "GM6020.hpp"
#include "GM3508.hpp"
#include "GM2006.hpp"

/**
 * @class ShooterStateRelax
 * @brief 云台放松状态机，电机不动。
 * @note 云台放松状态机继承自状态机基类。
 */
class ShooterStateRelax : public State
{
public:
    ShooterStateRelax(GM3508 *LeftFricMotor,
                      GM3508 *RightFricMotor,
                      GM2006 *TriggerMotor) : LeftFricMotor(LeftFricMotor),
                                              RightFricMotor(RightFricMotor),
                                              TriggerMotor(TriggerMotor) {};

    /**
     * @brief 析构函数。
     */
    ~ShooterStateRelax() {};

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
