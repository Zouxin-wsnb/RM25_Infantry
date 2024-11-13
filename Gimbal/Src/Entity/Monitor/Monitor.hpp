#ifndef MONITOR_HPP
#define MONITOR_HPP

// 1. C++标准库文件
#include <stdlib.h> // for malloc and free
#include <string.h> // for memcpy
#include <stdio.h>

// 2. STM32库文件
#include "main.h"
#include "arm_math.h"

// 3. BSP头文件
#include "bsp_usart.hpp"
#include "bsp_DWT.hpp"

#define MONITOR_LOG_MAX_SIZE 128 // 日志最大长度

/**
 * @class Monitor
 * @brief 监测类，用于监测机器人各个部分的状态
 * 所有的monitor函数将会在各个任务中调用，用于更新监测的标志位
 * run函数将会在主循环中调用，检查各个标志位，当标志位出现异常，或者更新时间超过阈值时，将会发出警告
 * @todo 关于视觉信息，裁判系统和超电部分的监测
 */
class Monitor
{
public:
    /**
     * @enum LOGLEVEL
     * 日志级别
     * INFO 信息
     * WARNING 警告
     * ERROR 错误
     */
    enum LOGLEVEL
    {
        INFO = 0,
        WARNING,
        ERROR
    };

    /**
     * @enum Monitor_Status
     * 表示通讯是否成功
     */
    enum Monitor_Status
    {
        Monitor_LOG_OK = 0,
        Monitor_LOG_BUSY,
        Monitor_LOG_ERROR
    };

    /**
     * @enum Probe_Status
     * 表示探针状态
     * @param Probe_ERROR 异常
     * @param Probe_NORMAL 正常
     */
    enum Probe_Status
    {
        Probe_ERROR = 0,
        Probe_NORMAL = 1
    };

    /**
     * @struct Probe
     * @brief 监测结构体,作用相当于探针
     * 用于记录监测的状态和更新时间
     * @param Update_Time_s 更新时间
     * @param Is_Normal 是否正常
     * @param Is_Loged 是否已经发出记录
     */
    struct Probe
    {
        float Update_Time_s; // 更新时间
        Probe_Status Status; // 是否正常
        bool Is_Loged;       // 是否已经记录
    };

    /**
     * @brief 构造函数
     */
    Monitor();

    /**
     * @brief 析构函数
     */
    ~Monitor();

    /**
     * @brief 初始化日志系统，设置监测参数。进行一次握手
     * 初始化串口DMA接收和每个探针的状态
     */
    void Monitor_Init();

    /**
     * @brief 设置机器人初始化完成标志位
     * 机器人完成所有初始化后，才能进行监测
     */
    void Monitor_RobotFinishInit();

    /**
     * @brief 写入日志信息
     * @param level 日志级别
     * @param message 日志信息
     * @param len 日志信息长度
     * @note 记得使用时在message后面加上"\r\n"，否则日志不会换行
     * @return Monitor_Status 通讯是否成功
     */
    Monitor_Status Log_Messages(LOGLEVEL level, uint8_t *message);

    /**
     * @brief 设置can通讯频率正常值
     */
    void Set_Normal_CanFreq(uint16_t freq);

    /**
     * @brief 设置usb接收频率正常值
     */
    void Set_Normal_USBRecvFreq(uint16_t freq);

    /**
     * @brief 设置电机温度正常值
     */
    void Set_Normal_MotorTemp(uint8_t temp);

    /**
     * @brief 监测底盘
     * 包括电机温度和CAN通信频率
     */
    void MotorMes_Monitor(uint8_t id);

    /**
     * @brief 监测视觉上位机
     * 包括USB接收频率
     */
    void Vision_USBRecvFreq_Monitor();

    /**
     * @brief 监测IMU
     * 包括IMU数据是否正常
     */
    void IMU_Monitor(float roll, float pitch, float yaw);

    /**
     * @brief 监测遥控器数据
     * 当遥控器数据一直为0时，发出warning
     */
    void RemoteController_Monitor();

    /**
     * @brief 监测函数
     * 检查各个监测标志位，当标志位出现异常，或者更新时间超过阈值时，发出警告
     */
    void Monitor_Run();

    Probe_Status Monitor_QueryProbeStatus(Probe *probe);

    /**
     * @brief 获取Monitor实例
     */
    static Monitor *Instance()
    {
        static Monitor monitor;
        return &monitor;
    }

    bool Is_Robot_Init; // 机器人是否完成所有初始化

protected:
    Probe MotorMes_Probe[8]; // 电机数据包监测

    Probe RemoteController_Probe; // 监测遥控器数据时间

    Probe IMU_Probe; // 监测IMU数据

    /*---------------------正常值---------------------*/
    uint16_t Normal_CanFreq;     // CAN通信频率正常值
    uint16_t Normal_USBRecvFreq; // USB接收频率正常值
    uint8_t Normal_MotorTemp;    // 电机温度正常值

    uint8_t Monitor_Counter; // 监测计数器
    float Monitor_Time;      // 监测时间
};

#endif // MONITOR_HPP
