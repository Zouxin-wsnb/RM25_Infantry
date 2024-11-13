#ifndef CHASSIS_STATE_REMOTE_CONTROL_HPP
#define CHASSIS_STATE_REMOTE_CONTROL_HPP

#include "Dr16.hpp"
#include "BoardConnectivity.hpp"
#include "GMMotor.hpp"

#include "StateMachine.hpp"

#include "FirstOrderFilter.hpp"
/**
 * @class ChassisStateRemoteControl
 * @brief 底盘遥控状态机，这个状态一般不用于比赛，主要用于调试。
 * @todo 添加键盘控制逻辑
 */
class ChassisStateRemoteControl : public State
{
public:
    /*用于速度解算的变量，单位为 m/s。这些变量也是直接用于操控底盘移动的变量。*/
    float Vx;                  ///< 横向移动速度
    float Vy;                  ///< 前后移动速度
    float Vw;                  ///< 旋转速度
    FirstOrderFilter VxFilter; ///< 横向速度滤波器
    FirstOrderFilter VyFilter; ///< 纵向速度滤波器
    FirstOrderFilter VwFilter; ///< 旋转速度滤波器
    BoardMsg xyMsg;
    BoardMsg wAndAngleMsg;
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

    /**
     * @brief 构造函数
     */
    ChassisStateRemoteControl() {};

    ~ChassisStateRemoteControl() 
    {
    }
};

#endif // CHASSIS_CONTROLLER_HPP
