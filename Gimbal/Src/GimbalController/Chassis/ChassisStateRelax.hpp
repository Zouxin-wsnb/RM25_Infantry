#ifndef CHASSIS_STATE_RELAX_HPP
#define CHASSIS_STATE_RELAX_HPP

#include "StateMachine.hpp"

#include "BoardConnectivity.hpp"
#include "GMMotor.hpp"

/**
 * @class ChassisStateRelax
 * @brief 底盘放松状态机。
 */
class ChassisStateRelax : public State
{
public:
    float Vx = 0; ///< 底盘X轴速度
    float Vy = 0; ///< 底盘Y轴速度
    float Vw = 0; ///< 底盘旋转速度
    BoardMsg xyMsg;
    BoardMsg wAndAngleMsg;
    /**
     * @brief 初始化函数，设置电机数据为0
     */
    void Init() override;

    /**
     * @brief 进入函数
     */
    void Enter() override;

    /**
     * @brief 执行函数，设置电机速度为0
     */
    void Execute() override;

    /**
     * @brief 退出函数
     */
    void Exit() override;
    /**
     * @brief 构造函数
     */
    ChassisStateRelax() {}

    ~ChassisStateRelax()
    {
    }
};

#endif // CHASSIS_CONTROLLER_HPP
