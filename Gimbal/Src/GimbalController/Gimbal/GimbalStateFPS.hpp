#ifndef GIMBAL_STATE_FPS_HPP
#define GIMBAL_STATE_FPS_HPP

#include "StateMachine.hpp"

#include "bsp_USB.hpp"

#include "GMMotorHandler.hpp"
#include "BoardConnectivity.hpp"

#include "Dr16.hpp"
#include "GM6020.hpp"
#include "GM3508.hpp"
#include "GM2006.hpp"
#include "PID.hpp"

#include "IMU_Config.hpp"
#ifdef IMU_LIB_EKF_
#include "AHRS-EKF.hpp"
#elif defined(IMU_LIB_MOTION_FX_)
#include "AHRS-MFX.hpp"
#elif defined(IMU_LIB_ICM42688_)
#include "AHRS-ICM.hpp"
#else
#include "AHRS-MFX.hpp" // 默认实现
#endif

#include "Math.hpp"
#include "string.h"
#include "FirstOrderFilter.hpp"
#include "KalmanFilter.hpp"
#include "crc.hpp"

/**
 * @struct USBReceivePacket
 * @brief USB接收数据包结构体。
 * 一共12个字节。由于USB虚拟串口的特性，发送和接受的数据的大小应该为4的倍数。
 */
struct USBReceivePacket
{
    uint8_t header; // 发送数据包的头

    uint8_t tracking : 1; // 跟踪的颜色
    uint8_t id : 3;       // 识别的id
    uint8_t fire : 1;     // 是否开火
    uint8_t reserved : 3; // 保留位

    float pitch; // 俯仰角
    float yaw;   // 偏航角

    uint16_t checksum; // 校验和
} __attribute__((packed));

/**
 * @class GimbalStateRelax
 * @brief 底盘跟随状态机
 * @note 云台放松状态机继承自状态机基类。
 */
class GimbalStateFPS : public State
{
public:
    /**
     * @brief 构造函数。
     * @param YawMotor 云台Yaw轴电机。
     * @param PitchMotor 云台Pitch轴电机。
     */
    GimbalStateFPS(GM6020 *YawMotor,
                   GM6020 *PitchMotor, USBReceivePacket *ReceivePacket) : YawMotor(YawMotor),
                                                                          PitchMotor(PitchMotor),
                                                                          ReceivePacket(ReceivePacket) {};

    /**
     * @brief 析构函数。
     */
    ~GimbalStateFPS() {};

    /**
     * @brief 云台Yaw轴电机。
     */
    GM6020 *YawMotor;

    /**
     * @brief 云台Pitch轴电机。
     */
    GM6020 *PitchMotor;

    KalmanFilter YawPosSetFilter;
    KalmanFilter YawSpdSetFilter;
    /**
     * @brief Pitch轴电机位置设置的一阶滤波器。
     */
    FirstOrderFilter PitchSetFilter;

    /**
     * @brief Yaw轴电机初始位置。
     * @note 弧度制，[0, 2π]
     * 这个值会在init函数中被赋值，在新车下载代码时，需要先用relaxmode进行debug查看。
     */
    float YawOffset;

    /**
     * @brief Pitch轴电机初始位置。
     * @note 弧度制，[0, 2π]
     * 这个值会在init函数中被赋值，需要根据实际情况修改这个值。会在init函数中被赋值，在新车下载代码时，需要先用relaxmode进行debug查看。
     */
    float PitchOffset;

    /**
     * @brief Yaw轴电机的位置或者速度设定。
     * 并且还需要根据实际情况添加限幅
     */
    float YawSet;

    /**
     * @brief Pitch轴电机的位置或者速度设定。
     * 并且还需要根据实际情况添加限幅
     */
    float PitchSet;

    /**
     * @brief 云台上限
     */
    float PitchUpperLimit;

    /**
     * @brief 云台下限
     */
    float PitchLowerLimit;

    /**
     * @brief 云台Yaw轴电机的前馈控制量。从底盘状态机中获取，目前是直接从chassiscontroller中获取。
     * @todo 从底盘C板发送回来的数据中获取。
     */
    float ForwardVw;

    /**
     * @brief 上位机发送的数据包。
     */
    USBReceivePacket *ReceivePacket;

    /**
     * @brief 初始化各种参数。
     */
    void Init() override;

    /**
     * @brief 进入放松状态。
     * 将电机设置为RELAX_MODE。
     */
    void Enter() override;

    /**
     * @brief 执行放松状态。
     * setOutput()函数设置电机的输出。
     * 电机输出为0。
     */

    void Execute() override;

    /**
     * @brief 退出放松状态。
     */
    void Exit() override;

    /**
     * @brief 自瞄程序。
     * 需要注意对yaw轴数据的处理。
     */
    void AutoAim();

    /**
     * @brief 解析视觉数据。
     */
    void DecodeVisionData();
};

#endif
