#include "ChassisStateRotate.hpp"
#include "GimbalController.hpp"

void ChassisStateRotate::Init()
{
    // 初始化速度为0
    Vx = 0.0f;
    Vy = 0.0f;
    Vw = 4.0f;

    xyMsg.type = BOARD_CONNECTIVITY_CAN_2;
    xyMsg.id = 0xB1;
    xyMsg.len = 8;

    wAndAngleMsg.type = BOARD_CONNECTIVITY_CAN_2;
    wAndAngleMsg.id = 0xB2;
    wAndAngleMsg.len = 8;
}

void ChassisStateRotate::Enter()
{
    // 初始化速度为0
    Vx = 0.0f;
    Vy = 0.0f;
    Vw = 4.0f;
}

void ChassisStateRotate::Execute()
{ 
    // 将遥控器的数据转化为底盘速度，在这里乘以一个常数，是为了调整速度的大小。单位应该为m/s。
    if (GimbalController::OffsetValue1 == GimbalController::Instance()->YawMotor.Offset)
    {
        Vx = Dr16::Instance()->GetLeftY() * 5;
        Vy = Dr16::Instance()->GetLeftX() * 5;
    }
    else if (GimbalController::OffsetValue2 == GimbalController::Instance()->YawMotor.Offset)
    {
        Vx = -Dr16::Instance()->GetLeftY() * 5;
        Vy = -Dr16::Instance()->GetLeftX() * 5;
    }
    Vw = 2.50f; // 单位为rad/s，因此不需要乘以常数，或者乘以一个较小的常数。不要超过Pi，因为在PID算法里面，总是计算最短距离。
    RelativeAngle = GimbalController::Instance()->YawMotor.motorFeedback.positionFdb - GimbalController::Instance()->YawMotor.Offset;

    // 发送数据给底盘
    memcpy(xyMsg.data, &Vx, 4);
    memcpy(xyMsg.data + 4, &Vy, 4);
    BoardConnectivity::Instance()->Add2Memory(xyMsg);

    memcpy(wAndAngleMsg.data, &Vw, 4);
    memcpy(wAndAngleMsg.data + 4, &RelativeAngle, 4);
    BoardConnectivity::Instance()->Add2Memory(wAndAngleMsg);
}

void ChassisStateRotate::Exit()
{
}
