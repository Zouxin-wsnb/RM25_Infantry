#ifndef SHOOTER_CONTROLLER_HPP
#define SHOOTER_CONTROLLER_HPP

#include "main.h"

#include "StateMachine.hpp"

#include "GMMotorHandler.hpp"
#include "Dr16.hpp"
#include "GM6020.hpp"
#include "GM3508.hpp"
#include "GM2006.hpp"
#include "Monitor.hpp"

#include "ShooterStateRelax.hpp"
#include "ShooterStateWarm.hpp"
#include "ShooterStateFire.hpp"
#include "ShooterStateKeyControl.hpp"

/**
 * @class ShooterController
 * @brief 射击控制类。
 * @note 该类继承自Controller类。
 * @todo 考虑是否需要像云台控制类或者云台状态类一样，将射击控制类的状态进行拆分成不同的类
 */
class ShooterController : public StateMachine
{
public:
    /**
     * @brief 左摩擦轮电机
     */
    GM3508 LeftFricMotor;

    /**
     * @brief 右摩擦轮电机
     */
    GM3508 RightFricMotor;

    /**
     * @brief 播弹盘电机
     */
    GM2006 TriggerMotor;

    ShooterStateFire FireState;
    ShooterStateRelax RelaxState;
    ShooterStateWarm WarmState;
    ShooterStateKeyControl KeyControlState;

    ShooterController() : FireState(&LeftFricMotor, &RightFricMotor, &TriggerMotor),
                          RelaxState(&LeftFricMotor, &RightFricMotor, &TriggerMotor),
                          WarmState(&LeftFricMotor, &RightFricMotor, &TriggerMotor),
                          KeyControlState(&LeftFricMotor, &RightFricMotor, &TriggerMotor)
    {
        SetCurrentState(&RelaxState);
        SetDefaultState(&RelaxState);
    }

    /**
     * @brief 析构函数。
     */
    ~ShooterController() {};

    /**
     * @brief 初始化函数。
     */
    void Init() override;

    /**
     * @brief 运行函数。
     */
    void Run() override;

    /**
     * @brief 处理输入函数。
     * @note 该函数会根据遥控器的输入，设置底盘的状态。
     */
    void HandleInput() override;

    static ShooterController *Instance()
    {
        static ShooterController instance;
        return &instance;
    }
};

#endif
