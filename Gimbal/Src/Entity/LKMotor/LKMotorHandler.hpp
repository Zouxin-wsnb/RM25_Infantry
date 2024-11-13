#ifndef LKMOTORHANDLER_HPP
#define LKMOTORHANDLER_HPP

#include "main.h"

#include "bsp_can.h"
#include "LKMotor.hpp"
#include "PID.hpp"
#include "Math.hpp"

#define LK_CAN_ID 0x140 ///< 控制ID范围在0x140-0x140+32的所有电机

/**
 * @class LKMotorHandler
 * @brief LK电机处理类，用于处理电机数据
 */
class LKMotorHandler
{
public:
    /**
     *@brief 使用指针数组, 2个CAN口，每个CAN口最多8个电机
     */
    LKMotor *LKMotorList[2][8];

    LKMotorHandler();
    ~LKMotorHandler();

    // 电机数据转换因子
    const float RawPosToRad = 0.0007669903939f;   // （7.6699039394282061485904379474597e-4）位置值转换为弧度的转换因子，编码器为十三位，2^13 = 8192, 2 * PI / 8192 (rad)，这样，当编码器的值增加或减少 1 时，它表示电机轴旋转了 2 * PI / 8192 (rad)
    const float RawRpmToRadps = 0.1047197551196f; // 0.1047197551f;  // RPM 到弧度每秒的转换因子, 2 * PI / 60 (s)
    const float RawPos2Rad = 0.00009587379924285f;  /* 2Pi / 65536*/
    const float RawDps2Rps = 0.0174532925199433f;   /* 2Pi / 360 */
    
    /**
     * @brief CAN1接收数据，经过了初步的处理，索引对应电机ID
     */
    LKMotor::motor_measure_t can1_receive_data[8];
    /**
     * @brief CAN2接收数据，经过了初步的处理，索引对应电机ID
     */
    LKMotor::motor_measure_t can2_receive_data[8];

    uint8_t can1_send_data_0[8]; // CAN1电机控制数据，
    uint8_t can1_send_data_1[8]; // CAN1电机控制数据，

    uint8_t can2_send_data_0[8]; // CAN2电机控制数据，
    uint8_t can2_send_data_1[8]; // CAN2电机控制数据，

    /**
     * @brief 注册电机，将电机指针存入MotorList中
     * @param motor 电机指针
     * @param hcan CAN句柄
     * @param canId 电机ID
     */
    void registerMotor(LKMotor *LKmotor, CAN_HandleTypeDef *hcan, uint16_t canId); // 使用指针作为参数

    /**
     * @brief 处理来自can回调中的原始数据
     * @param rx_data 接收到的原始数据的数组
     * @param hcan CAN句柄
     * @param index 电机索引
     * @note 该函数用于处理CAN接收到的原始数据，将其转换为电机测量数据，储存在can_receive_data中。在最后调用updateFeedback()函数更新电机数据。
     */
    void processRawData(CAN_HandleTypeDef *hcan, uint8_t *rx_data, int index);

    /**
     * @brief 发送控制数据
     * @param hcan1 CAN1句柄
     * @param hcan2 CAN2句柄
     */
    void sendControlData(CAN_HandleTypeDef *hcan1, CAN_HandleTypeDef *hcan2);

    /**
     * @brief 处理并更新电机反馈数据
     * @param hcan1 CAN1句柄
     * @param hcan2 CAN2句柄
     */
    void updateFeedback();

    static LKMotorHandler *instance()
    {
        static LKMotorHandler instance;
        return &instance;
    }
};

#endif // LKMOTORHANDLER_HPP
