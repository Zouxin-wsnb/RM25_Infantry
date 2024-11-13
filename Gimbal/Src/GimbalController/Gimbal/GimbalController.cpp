#include "GimbalController.hpp"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

// 两个方向的偏移量，用于防止切换状态时云台电机旋转180度时的突变
const float GimbalController::OffsetValue1 = 4.56359291 - Math::Pi;
const float GimbalController::OffsetValue2 = 4.56359291 - Math::PiX2;

void GimbalController::Init()
{
    RelaxState.Init();
    RemoteControlState.Init();
    SearchState.Init();
    FPSState.Init();

    SetDefaultState(&RelaxState);
    SetCurrentState(&RelaxState);
    YawMotor.controlMode = GM6020::RELAX_MODE;
    YawMotor.Offset = 4.56359291 - Math::Pi;
    GMMotorHandler::instance()->registerMotor(&YawMotor, &hcan1, 0x205); // 挂载云台电机

    PitchMotor.controlMode = GM6020::RELAX_MODE;
    PitchMotor.Offset = 0.0f;
    GMMotorHandler::instance()->registerMotor(&PitchMotor, &hcan1, 0x206); // 挂载云台电机

    SendPacket.header = 0x5A;
    SendPacket.detect_color = 0;
    SendPacket.reset_tracker = 0;
    SendPacket.q1 = 0.0f;
    SendPacket.q2 = 0.0f;
    SendPacket.q3 = 0.0f;
    SendPacket.q4 = 0.0f;
    SendPacket.checksum = 0;

    // 日志输出
    Monitor::Instance()->Log_Messages(Monitor::INFO, (uint8_t *)("GimbalController Init Success!\r\n"));
}

void GimbalController::HandleInput()
{
    // 遥控器拨杆控制状态
//    if (Dr16::Instance()->QuerySwStatus(Dr16::LEFT_SWITCH) == Dr16::RC_SW_DOWN) // 如果遥控器左边的拨杆向下
//    {
//        ChangeState(&RelaxState);
//    }
//    else if (Dr16::Instance()->QuerySwStatus(Dr16::LEFT_SWITCH) == Dr16::RC_SW_MID) // 如果遥控器左边的拨杆中间
//    {
//        ChangeState(&FPSState); // 遥控状态
//    }
//    else if (Dr16::Instance()->QuerySwStatus(Dr16::LEFT_SWITCH) == Dr16::RC_SW_UP) // 如果遥控器左边的拨杆向上
//    {
//        ChangeState(&FPSState); // 搜索状态
//    }
//    else // 默认状态
//    {
//        ChangeState(&RelaxState);
//    }

    // 键盘控制状态
    if (Dr16::Instance()->QueryPcKeyStatus(Dr16::PC_KEY_R) == Dr16::PC_KEY_DOWN)
        ChangeState(&RelaxState);
    else if (Dr16::Instance()->QueryPcKeyStatus(Dr16::PC_KEY_F) == Dr16::PC_KEY_DOWN)
        ChangeState(&FPSState);
    else if (Dr16::Instance()->QueryPcKeyStatus(Dr16::PC_KEY_V) == Dr16::PC_KEY_DOWN)
        ChangeState(&SearchState);

    // 放松模式保护
    if (Dr16::Instance()->QuerySwStatus(Dr16::LEFT_SWITCH) == Dr16::RC_SW_DOWN) // 如果遥控器左边的拨杆向下
    {
        ChangeState(&RelaxState);
    }

    if (Dr16::Instance()->QuerySwStatus(Dr16::LEFT_SWITCH_CHANGE) == Dr16::RC_SWITCH_U2M) // 如果从小陀螺模式切换回底盘跟随云台模式
    {
        float distance1 = fabs(YawMotor.motorFeedback.positionFdb - OffsetValue1);
        float distance2 = fabs(YawMotor.motorFeedback.positionFdb - OffsetValue2);
        if (distance1 < distance2)
        {
            YawMotor.Offset = OffsetValue1;
        }
        else
        {
            YawMotor.Offset = OffsetValue2;
        }
    }

    UpdateUSBData();
    DecodeVisionData();
}

void GimbalController::Run()
{
    HandleInput();
    Update();
}
uint8_t counter = 0;

void GimbalController::UpdateUSBData()
{
    counter++;
    if (counter % 5 == 0)
    {
        SendPacket.header = 0x5A;
        SendPacket.detect_color = 1;
        SendPacket.q1 = AHRS::Instance()->INS.q[0];
        SendPacket.q2 = AHRS::Instance()->INS.q[1];
        SendPacket.q3 = AHRS::Instance()->INS.q[2];
        SendPacket.q4 = AHRS::Instance()->INS.q[3];
        Append_CRC16_Check_Sum((uint8_t *)&SendPacket, 20);
        USB_Transmit((uint8_t *)&SendPacket, sizeof(SendPacket));
    }
}

void GimbalController::DecodeVisionData()
{
    uint8_t buffer[sizeof(ReceivePacket)];
    USB_Receive(buffer, sizeof(buffer));

    // 数据的处理需要根据视觉发送的格式更改
    ReceivePacket.header = buffer[0]; // 解析header
    // 解析 tracking, id, reserved 组合字段
    ReceivePacket.tracking = (buffer[1] >> 7) & 0x01; // 取出tracking位 (最高位)
    ReceivePacket.id = (buffer[1] >> 4) & 0x07;       // 取出id (3位)
    ReceivePacket.fire = (buffer[1] >> 3) & 0x01;     // 取出fire位 (1位)
    ReceivePacket.reserved = buffer[1] & 0x07;        // 取出reserved位 (3位)
    // 手动解析float类型的pitch和yaw
    uint8_t *pitchPtr = (uint8_t *)&ReceivePacket.pitch;
    uint8_t *yawPtr = (uint8_t *)&ReceivePacket.yaw;
    for (int i = 0; i < sizeof(float); i++)
    {
        pitchPtr[i] = buffer[2 + i];
        yawPtr[i] = buffer[6 + i];
    }
    // 解析checksum
    ReceivePacket.checksum = (uint16_t)(buffer[10] | (buffer[11] << 8));
}
