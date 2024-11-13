#include "LKMotorHandler.hpp"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/**
 * @brief 构造函数，将所有值初始化
 */
LKMotorHandler::LKMotorHandler()
{
    for (int i = 0; i < 8; i++)
        LKMotorList[0][i] = nullptr;
    for (int i = 0; i < 8; i++)
        LKMotorList[1][i] = nullptr;

    for (int i = 0; i < 8; i++)
        can1_send_data_0[i] = 0;
    for (int i = 0; i < 8; i++)
        can1_send_data_1[i] = 0;
    for (int i = 0; i < 8; i++)
        can2_send_data_0[i] = 0;
    for (int i = 0; i < 8; i++)
        can2_send_data_1[i] = 0;
}

LKMotorHandler::~LKMotorHandler()
{
}

/**
 * @brief 注册电机，将电机指针存入MotorList中
 * @param motor 电机指针
 * @param hcan CAN句柄
 * @param canId 电机ID，范围在0x140-0x140+32的所有电机
 */
void LKMotorHandler::registerMotor(LKMotor *LKmotor, CAN_HandleTypeDef *hcan, uint16_t canId)
{
    LKmotor->canId = canId;
    LKmotor->hcan = hcan;

    if (canId >= 0x140 && canId <= 0x160)
    {
        if (LKmotor->hcan == &hcan1)
            LKMotorList[0][canId - 0x140] = LKmotor;
        else if (LKmotor->hcan == &hcan2)
            LKMotorList[1][canId - 0x140] = LKmotor;
    }
}

void LKMotorHandler::processRawData(CAN_HandleTypeDef *hcan, uint8_t *rx_data, int index)
{
    if (hcan == &hcan1)
    {
        for (int i = 0; i < 8; i++)
        {
            can1_receive_data[index].last_ecd = can1_receive_data[index].ecd;                 ///< 上一次的编码器值
            can1_receive_data[index].ecd = (uint16_t)(rx_data[7] << 8 | rx_data[6]);          ///< 编码器值
            can1_receive_data[index].speed_dps = (int16_t)(rx_data[5] << 8 | rx_data[4]);     ///< 速度值，单位dps
            can1_receive_data[index].given_current = (int16_t)(rx_data[3] << 8 | rx_data[2]); ///< 电流值，或者说是转矩值
            can1_receive_data[index].temperate = rx_data[1];                                  ///< 温度值
        }
    }
    else if (hcan == &hcan2)
    {
        for (int i = 0; i < 8; i++)
        {
            can2_receive_data[index].last_ecd = can2_receive_data[index].ecd;                 ///< 上一次的编码器值
            can2_receive_data[index].ecd = (uint16_t)(rx_data[7] << 8 | rx_data[6]);          ///< 编码器值
            can2_receive_data[index].speed_dps = (int16_t)(rx_data[5] << 8 | rx_data[4]);     ///< 速度值，单位dps
            can2_receive_data[index].given_current = (int16_t)(rx_data[3] << 8 | rx_data[2]); ///< 电流值，或者说是转矩值
            can2_receive_data[index].temperate = rx_data[1];                                  ///< 温度值
        }
    }
}

/**
 * @brief 发送控制数据
 * @param hcan1 CAN1句柄
 * @param hcan2 CAN2句柄
 */
void LKMotorHandler::sendControlData(CAN_HandleTypeDef *hcan1, CAN_HandleTypeDef *hcan2)
{
    // 循环遍历所有的电机，将电机的控制数据存入can_send_data中
    for (int i = 0; i < 8; i++)
    {
        // 处理挂载在CAN1的电机
        if (LKMotorList[0][i] != nullptr)
        {
            //控制标识符为0x140
            if (LKMotorList[0][i]->canId >= 0x140 && LKMotorList[0][i]->canId <= 0x160)
            {
                int index = (LKMotorList[0][i]->canId - 0x200) * 2;
                can1_send_data_0[index - 2] = LKMotorList[0][i]->currentSet >> 8;
                can1_send_data_0[index - 1] = LKMotorList[0][i]->currentSet;
            }

        }
        // 处理挂载在CAN2的电机
        if (LKMotorList[1][i] != nullptr)
        {
            if (LKMotorList[1][i]->canId >= 0x140 && LKMotorList[1][i]->canId <= 0x160)
            {
                int index = (LKMotorList[1][i]->canId - 0x200) * 2;
                can2_send_data_0[index - 2] = LKMotorList[1][i]->currentSet >> 8;
                can2_send_data_0[index - 1] = LKMotorList[1][i]->currentSet;
            }

        }
    }
    // 使用bsp_can中的函数发送数据
    can_sendData(hcan1, 0x280, can1_send_data_0, 8);
    can_sendData(hcan2, 0x280, can2_send_data_0, 8);
}

/**
 * @brief 处理并更新电机反馈数据
 * 将电机的反馈数据存入Motorlist中存在的电机的MotorFeedback中
 * @param hcan1 CAN1句柄
 * @param hcan2 CAN2句柄
 */
void LKMotorHandler::updateFeedback()
{
    for (int i = 0; i < 8; i++)
    {
        // 处理CAN1的电机
        if (LKMotorList[0][i] != nullptr)
        {
            LKMotorList[0][i]->motorFeedback.lastPositionFdb = LKMotorList[0][i]->motorFeedback.positionFdb;
            LKMotorList[0][i]->motorFeedback.lastSpeedFdb = LKMotorList[0][i]->motorFeedback.speedFdb;
            // 位置反馈，转换为弧度，并限幅至-PI到PI
            LKMotorList[0][i]->motorFeedback.positionFdb = Math::LoopFloatConstrain((float)(can1_receive_data[i].ecd - LKMotorList[0][i]->offset) * RawPosToRad, -Math::Pi, Math::Pi);
            LKMotorList[0][i]->motorFeedback.speedFdb = can1_receive_data[i].speed_dps * LKMotorHandler::RawDps2Rps;
            LKMotorList[0][i]->motorFeedback.currentFdb = can1_receive_data[i].given_current;

            LKMotorList[0][i]->motorFeedback.torqueFdb = (float)LKMotorList[0][i]->motorFeedback.currentFdb / 392.78f;

            LKMotorList[0][i]->motorFeedback.temperatureFdb = (float)can1_receive_data[i].temperate;
        }
        // 处理CAN2的电机
        if (LKMotorList[1][i] != nullptr)
        {
            LKMotorList[1][i]->motorFeedback.lastPositionFdb = LKMotorList[1][i]->motorFeedback.positionFdb;
            LKMotorList[1][i]->motorFeedback.lastSpeedFdb = LKMotorList[1][i]->motorFeedback.speedFdb;
            // 位置反馈，转换为弧度，并限幅至-PI到PI
            LKMotorList[1][i]->motorFeedback.positionFdb = Math::LoopFloatConstrain((float)(can1_receive_data[i].ecd - LKMotorList[1][i]->offset) * RawPosToRad, -Math::Pi, Math::Pi);
            LKMotorList[1][i]->motorFeedback.speedFdb = can1_receive_data[i].speed_dps * LKMotorHandler::RawRpmToRadps;
            LKMotorList[1][i]->motorFeedback.currentFdb = can1_receive_data[i].given_current;

            LKMotorList[1][i]->motorFeedback.torqueFdb = (float)LKMotorList[1][i]->motorFeedback.currentFdb / 392.78f;
            LKMotorList[1][i]->motorFeedback.temperatureFdb = (float)can1_receive_data[i].temperate;
        }
    }
}
