#include "ChassisStateRemoteControl.hpp"
#include "GimbalController.hpp"

void ChassisStateRemoteControl::Init()
{
    // 初始化速度为0
    Vx = 0.0f;
    Vy = 0.0f;
    Vw = 0.0f;

    // 设置滤波器参数
    VxFilter.SetTau(0.16f);
    VxFilter.SetUpdatePeriod(5);

    VyFilter.SetTau(0.12f);
    VyFilter.SetUpdatePeriod(5);

    VwFilter.SetTau(0.04f);
    VwFilter.SetUpdatePeriod(5);

    xyMsg.type = BOARD_CONNECTIVITY_CAN_2;
    xyMsg.id = 0xB1;
    xyMsg.len = 8;

    wAndAngleMsg.type = BOARD_CONNECTIVITY_CAN_2;
    wAndAngleMsg.id = 0xB2;
    wAndAngleMsg.len = 8;
}

void ChassisStateRemoteControl::Enter()
{
}

#ifdef KeyBoardControl
void ChassisStateRemoteControl::Execute()
{
    // 将键盘数据转化为底盘速度，乘常数控制其响应速度
    Vx *= 0.2f;
    Vy *= 0.2f;
    if (Dr16::Instance()->QueryPcKeyStatus(Dr16::PC_KEY_S) == Dr16::PC_KEY_DOWN)
        Vy += 0.01f;
    else if (Dr16::Instance()->QueryPcKeyStatus(Dr16::PC_KEY_W) == Dr16::PC_KEY_DOWN)
        Vy -= 0.01f;
    else if (Dr16::Instance()->QueryPcKeyStatus(Dr16::PC_KEY_A) == Dr16::PC_KEY_DOWN)
        Vx -= 0.01f;
    else if (Dr16::Instance()->QueryPcKeyStatus(Dr16::PC_KEY_D) == Dr16::PC_KEY_DOWN)
        Vx += 0.01f;
    Vx *= 5.0f;
    Vy *= 5.0f;

    Math::FloatConstrain(Vx, -1.0f, 1.0f);
    Math::FloatConstrain(Vy, -1.0f, 1.0f);

    // 对速度进行滤波
    VxFilter.SetInput(Vx);
    VxFilter.Update();
    Vx = VxFilter.GetResult();

    // 对速度进行滤波
    VyFilter.SetInput(Vy);
    VyFilter.Update();
    Vy = VyFilter.GetResult();

    // 发送数据给底盘
    memcpy(xyMsg.data, &Vx, 4);
    memcpy(xyMsg.data + 4, &Vy, 4);
    BoardConnectivity::Instance()->Add2Memory(xyMsg);

    memcpy(wAndAngleMsg.data, &Vw, 4);
    memcpy(wAndAngleMsg.data + 4, &Vw, 4);
    BoardConnectivity::Instance()->Add2Memory(wAndAngleMsg);
}
#else
#define RemoteControl
void ChassisStateRemoteControl::Execute()
{
    // 将遥控器的数据转化为底盘速度，在这里乘以一个常数，是为了调整速度的大小。单位应该为m/s。
    Vx = Dr16::Instance()->GetLeftX() * 5;
    Vy = Dr16::Instance()->GetLeftY() * 5;
    Vw = Dr16::Instance()->GetRightX() * 3; // 单位为rad/s

    // 对速度进行滤波
    VxFilter.SetInput(Vx);
    VxFilter.Update();
    Vx = VxFilter.GetResult();

    // 对速度进行滤波
    VyFilter.SetInput(Vy);
    VyFilter.Update();
    Vy = VyFilter.GetResult();

    // 发送数据给底盘
    memcpy(xyMsg.data, &Vx, 4);
    memcpy(xyMsg.data + 4, &Vy, 4);
    BoardConnectivity::Instance()->Add2Memory(xyMsg);

    memcpy(wAndAngleMsg.data, &Vw, 4);
    memcpy(wAndAngleMsg.data + 4, &Vw, 4);
    BoardConnectivity::Instance()->Add2Memory(wAndAngleMsg);
}
#endif

void ChassisStateRemoteControl::Exit()
{
}
