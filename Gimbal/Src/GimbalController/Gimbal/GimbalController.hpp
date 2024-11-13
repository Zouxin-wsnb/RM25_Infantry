#ifndef GIMBALCONTROLLER_HPP
#define GIMBALCONTROLLER_HPP

#include "main.h"

#include "bsp_USB.hpp"

#include "StateMachine.hpp"

#include "GMMotorHandler.hpp"
#include "Dr16.hpp"
#include "GM6020.hpp"
#include "GM3508.hpp"
#include "GM2006.hpp"
#include "Monitor.hpp"

#include "GimbalStateRemoteControl.hpp"
#include "GimbalStateSearch.hpp"
#include "GimbalStateRelax.hpp"
#include "GimbalStateFPS.hpp"

#include "crc.hpp"
/**
 * @struct USBSendPacket
 * @brief USB发送数据包结构体。
 * 一共20个字节。由于USB虚拟串口的特性，发送和接受的数据的大小应该为4的倍数。
 */
struct USBSendPacket
{
    uint8_t header;           // 发送数据包的头
    uint8_t detect_color : 1; // 检测到的颜色
    bool reset_tracker : 1;   // 是否重置追踪
    uint8_t reserved : 6;     // 保留位
    float q1;                 // 四元数
    float q2;
    float q3;
    float q4;
    uint16_t checksum; // 校验和
} __attribute__((packed));

/**
 * @class GibmalController
 * @brief 云台控制类，主要操控云台的电机。
 */
class GimbalController : public StateMachine
{
public:
    /**
     * @brief 云台Yaw轴电机。
     * 电机实例，将会传入状态机中
     */
    GM6020 YawMotor;

    // 两个方向的偏移量，用于在小陀螺模式和底盘跟随云台模式之间切换时防止云台电机旋转180度时的突变
    static const float OffsetValue1;
    static const float OffsetValue2;
    /**
     * @brief 云台Pitch轴电机。
     * 电机实例，将会传入状态机中
     */
    GM6020 PitchMotor;

    /*-----------------------------状态-----------------------------*/

    /**
     * @brief 放松状态
     */
    GimbalStateRelax RelaxState;

    /**
     * @brief 自瞄状态
     */
    GimbalStateSearch SearchState;

    /**
     * @brief 遥控器遥控状态
     */
    GimbalStateRemoteControl RemoteControlState;

    /**
     * @brief 底盘跟随云台状态
     */
    GimbalStateFPS FPSState;

    /**
     * @brief USB发送数据包。
     */
    USBSendPacket SendPacket;

    /**
     * @brief USB接收数据包。
     */
    USBReceivePacket ReceivePacket;

    /**
     * @brief 构造函数
     * 默认模式是放松模式。
     * 将电机注册传入不同的状态机。
     */
    GimbalController() : RelaxState(&YawMotor, &PitchMotor),
                         RemoteControlState(&YawMotor, &PitchMotor),
                         SearchState(&YawMotor, &PitchMotor),
                         FPSState(&YawMotor, &PitchMotor, &ReceivePacket)
    {
        SetCurrentState(&RelaxState);
        SetDefaultState(&RelaxState);
    }

    /**
     * @brief 析构函数。
     */
    ~GimbalController() {};

    /**
     * @brief 初始化函数。
     * 初始化所有的底盘电机，注册电机到MotorHandler。
     * 电机的id在这里设置
     */
    void Init() override;

    /**
     * @brief 运行函数。
     * 根据遥控器的状态，选择不同的状态。
     */
    void HandleInput() override;

    /**
     * @brief 更新USB数据。
     * 主要更新四元数数据。
     */
    void UpdateUSBData();

    /**
     * @brief 解析视觉数据。
     */
    void DecodeVisionData();

    void Run() override;
    /**
     * @brief 获取云台控制类的单例。
     * @return GimbalController* 返回云台控制类的单例。
     */
    static GimbalController *Instance()
    {
        static GimbalController instance;
        return &instance;
    }
};

#endif
