#ifndef CHASSIS_STATE_REMOTE_CONTROL_HPP
#define CHASSIS_STATE_REMOTE_CONTROL_HPP

#include "main.h"
#include "arm_math.h"
#include "string.h"

#include "GM3508.hpp"
#include "GMMotorHandler.hpp"
#include "BoardConnectivity.hpp"
#include "SuperCap.hpp"

#include "StateMachine.hpp"

#include "FirstOrderFilter.hpp"
#include "KalmanFilter.hpp"
#include "Math.hpp"
#include "Pid.hpp"
#include "Referee.hpp"

#define InitialPowerBuffer 60
#define square(x) ((x) * (x))
//#define PowerLimitNormal
// #define PowerLimitWithBuffer
// #define PowerLimitWithSuperCap

/**
 * @class ChassisController
 * @brief 底盘控制类，提供底盘的基本控制功能。
 * 用于底盘的C板，接收指令并控制底盘电机。
 * @todo 添加键盘控制逻辑
 */
class ChassisController : public StateMachine
{
public:
    /*电机实例*/
    GM3508 R_Front; ///< 右前轮
    GM3508 L_Front; ///< 左前轮
    GM3508 R_Rear;  ///< 右后轮
    GM3508 L_Rear;  ///< 左后轮

    /*用于速度解算的变量，单位为 m/s。这些变量也是直接用于操控底盘移动的变量。*/
    float Vx;            ///< 绝对横向移动速度
    float Vy;            ///< 绝对前后移动速度
    float Vw;            ///< 绝对旋转速度
    float RelativeAngle; ///< 云台和底盘的相对角度

    /*速度解算后四个轮子的速度*/
    float RF_speedSet; ///< 右前轮速度设定
    float LF_speedSet; ///< 左前轮速度设定
    float RR_speedSet; ///< 右后轮速度设定
    float LR_speedSet; ///< 左后轮速度设定

    /*与速度解算相关的一些常量*/
    const float distanceToFrontAxle = 1.0f; ///< 中心到前轴距离
    const float distanceToRearAxle = 1.0f;  ///< 中心到后轴距离
    const float distanceToRightSide = 1.0f; ///< 中心到右侧距离
    const float distanceToLeftSide = 1.0f;  ///< 中心到左侧距离
    const float wheelRadius = 0.15f;        ///< 轮子半径

    PID GM3508SpeedPid; ///< 速度环PID

    SuperCapSetUnion SCSetUnion; ///< 用于发送给超级电容的数据
    SuperCapFdbUnion SCFdbUnion; ///< 用于接收超级电容的数据

    // FirstOrderFilter VxFilter; ///< 横向速度滤波器
    // FirstOrderFilter VyFilter; ///< 纵向速度滤波器
    // FirstOrderFilter VwFilter; ///< 旋转速度滤波器

    KalmanFilter VxFilter; ///< 横向速度滤波器
    KalmanFilter VyFilter; ///< 纵向速度滤波器
    KalmanFilter VwFilter; ///< 旋转速度滤波器

    BoardMsg ChassisMsg; ///< 用于发送给云台的数据
    /**
     * @brief 初始化函数，用于挂载四个底盘电机，并设置PID参数
     */
    void Init() override;

    /**
     * @brief 运行函数，依次执行enter、Execute、exit函数
     */
    void Run() override;
    /**
     * @brief 构造函数，将位于controller的四个底盘电机指针传入
     */
    void HandleInput() override;
    /**
     * @brief 功率限制函数，用于限制底盘功率
     */
    void PowerLimit();
    /**
     * @brief 根据缓存能量，限制底盘功率
     */
    void PowerLimitWithPowerBuffer();
    /**
     * @brief 根据超级电容，限制底盘功率
     */
    void PowerLimitWithSuperCap();

    ChassisController() {};
    ~ChassisController() {};

    static ChassisController *Instance()
    {
        static ChassisController instance;
        return &instance;
    }
};

#endif // CHASSIS_CONTROLLER_HPP
