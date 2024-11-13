#ifndef CHASSIS_STATE_FPS_CONTROL_HPP
#define CHASSIS_STATE_FPS_CONTROL_HPP

#include "GMMotor.hpp"

#include "StateMachine.hpp"
#include "Dr16.hpp"
#include "BoardConnectivity.hpp"

#include "PID.hpp"
#include "FirstOrderFilter.hpp"
#include "KalmanFilter.hpp"

/**
 * @class ChassisStateRemoteControl
 * @brief 底盘遥控状态机。
 */
class ChassisStateFPS : public State
{
public:
    float Vx;               ///< 横向移动速度
    float Vy;               ///< 前后移动速度
    float Vw;               ///< 旋转速度
    PID ChassisYawSpeedPid; ///< 底盘跟随云台Yaw轴速度PID
    float relativeAngle;    ///< 云台相对于底盘的角度，用于底盘跟随云台
    BoardMsg xyMsg;         ///< xy方向速度
    BoardMsg wAndAngleMsg;  ///< 角速度和云台底盘的相对角度

    // 对速度进行卡尔曼滤波
    KalmanFilter VxFilter;
    KalmanFilter VyFilter;
    KalmanFilter VwFilter;

    /**
     * @brief 初始化函数，设置电机数据为0
     */
    void Init() override;

    /**
     * @brief 进入函数，接受遥控器指令
     */
    void Enter() override;

    /**
     * @brief 执行函数，将速度指令放入BoardConnectivity的发送缓冲区
     */
    void Execute() override;

    /**
     * @brief 退出函数
     */
    void Exit() override;

    ChassisStateFPS() {};

    ~ChassisStateFPS() {};
};

#endif
