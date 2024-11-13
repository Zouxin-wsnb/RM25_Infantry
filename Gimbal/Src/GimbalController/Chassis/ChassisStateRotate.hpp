#ifndef CHASSIS_STATE_ROTATE_HPP
#define CHASSIS_STATE_ROTATE_HPP

#include "BoardConnectivity.hpp"
#include "Dr16.hpp"
#include "GMMotor.hpp"

#include "StateMachine.hpp"

/**
 * @class ChassisStateRotate
 * @brief 底盘小陀螺状态机。
 * @todo 添加键盘控制逻辑
 * @todo 添加小陀螺随机速度
 */
class ChassisStateRotate : public State
{
public:
    /*用于速度解算的变量，单位为 m/s。这些变量也是直接用于操控底盘移动的变量。*/
    float Vx;              ///< 横向移动速度
    float Vy;              ///< 前后移动速度
    float Vw;              ///< 旋转速度
    float RelativeAngle;   ///< 相对角度
    BoardMsg xyMsg;        // xy方向速度
    BoardMsg wAndAngleMsg; // 角速度和云台底盘的相对角度
    /**
     * @brief 初始化函数，设置电机数据为0, w设定为一个固定值
     */
    void Init() override;

    /**
     * @brief 进入函数，接收底盘速度指令，进行速度解算
     */
    void Enter() override;

    /**
     * @brief 执行函数，将速度指令转化为电机速度
     */
    void Execute() override;

    /**
     * @brief 退出函数，设置电机速度
     */
    void Exit() override;

    ChassisStateRotate() {}

    ~ChassisStateRotate() {}
};

#endif // CHASSIS_CONTROLLER_HPP
