#include "ChassisStateFPS.hpp"
#include "GimbalController.hpp"

float debug_chassis_follow = 0.0f;
float KM_R = 0.6f;
float KM_Q = 0.001f;
float scale = 5.0f;

/**
 * @brief 初始化函数，设置各种数据
 */
void ChassisStateFPS::Init()
{
    Vx = 0.0f;
    Vy = 0.0f;
    Vw = 0.0f;

    // 设置PID参数
    ChassisYawSpeedPid.kp = 2.5f;
    ChassisYawSpeedPid.ki = 0.0f;
    ChassisYawSpeedPid.kd = 9.0f;
    ChassisYawSpeedPid.maxIOut = 0.0f;
    ChassisYawSpeedPid.maxOut = 15.0f;

    VxFilter.Clear();
    VyFilter.Clear();
    VwFilter.Clear();
}

void ChassisStateFPS::Enter()
{
    // 发送数据给底盘
    xyMsg.type = BOARD_CONNECTIVITY_CAN_2;
    xyMsg.id = 0xB1;
    xyMsg.len = 8;
    BoardConnectivity::Instance()->Add2Memory(xyMsg);

    wAndAngleMsg.type = BOARD_CONNECTIVITY_CAN_2;
    wAndAngleMsg.id = 0xB2;
    wAndAngleMsg.len = 8;
    BoardConnectivity::Instance()->Add2Memory(wAndAngleMsg);
}

void ChassisStateFPS::Execute()
{
    VxFilter.SetQ(KM_Q);
    VyFilter.SetQ(KM_Q);
    VwFilter.SetQ(KM_Q);
    VxFilter.SetR(KM_R);
    VyFilter.SetR(KM_R);
    VwFilter.SetR(KM_R);

    // 将遥控器的数据转化为底盘速度，在这里乘以一个常数，是为了调整速度的大小。单位为m/s。
    // 将键盘数据转化为底盘速度，乘常数控制其响应速度
    Vx /= scale;
    Vy /= scale;
    if (GimbalController::OffsetValue1 == GimbalController::Instance()->YawMotor.Offset)
    {
        //        Vx = Dr16::Instance()->GetLeftY() * 5;
        //        Vy = Dr16::Instance()->GetLeftX() * 5;
        if (Dr16::Instance()->QueryPcKeyStatus(Dr16::PC_KEY_S) == Dr16::PC_KEY_DOWN)
            Vy += 0.01f;
        else if (Dr16::Instance()->QueryPcKeyStatus(Dr16::PC_KEY_W) == Dr16::PC_KEY_DOWN)
            Vy -= 0.01f;
        else if (Dr16::Instance()->QueryPcKeyStatus(Dr16::PC_KEY_A) == Dr16::PC_KEY_DOWN)
            Vx -= 0.01f;
        else if (Dr16::Instance()->QueryPcKeyStatus(Dr16::PC_KEY_D) == Dr16::PC_KEY_DOWN)
            Vx += 0.01f;
        else
        {
            Vx = 0;
            Vy = 0;
        }
    }
    else if (GimbalController::OffsetValue2 == GimbalController::Instance()->YawMotor.Offset)
    {
        //        Vx = -Dr16::Instance()->GetLeftY() * 5;
        //        Vy = -Dr16::Instance()->GetLeftX() * 5;
        if (Dr16::Instance()->QueryPcKeyStatus(Dr16::PC_KEY_S) == Dr16::PC_KEY_DOWN)
            Vy -= 0.01f;
        else if (Dr16::Instance()->QueryPcKeyStatus(Dr16::PC_KEY_W) == Dr16::PC_KEY_DOWN)
            Vy += 0.01f;
        else if (Dr16::Instance()->QueryPcKeyStatus(Dr16::PC_KEY_A) == Dr16::PC_KEY_DOWN)
            Vx += 0.01f;
        else if (Dr16::Instance()->QueryPcKeyStatus(Dr16::PC_KEY_D) == Dr16::PC_KEY_DOWN)
            Vx -= 0.01f;
        else
        {
            Vx = 0;
            Vy = 0;
        }
    }

    Vx *= scale;
    Vy *= scale;

    Math::FloatConstrain(Vx, -1.0f, 1.0f);
    Math::FloatConstrain(Vy, -1.0f, 1.0f);
    Math::FloatConstrain(Vw, -1.0f, 1.0f);

    relativeAngle = GimbalController::Instance()->YawMotor.motorFeedback.positionFdb - GimbalController::Instance()->YawMotor.Offset;
    // 防止角度突变
    if (relativeAngle >= Math::Pi)
    {
        relativeAngle -= Math::PiX2;
    }
    else if (relativeAngle <= -Math::Pi)
    {
        relativeAngle += Math::PiX2;
    }

    // 底盘跟随云台PID计算
    ChassisYawSpeedPid.ref = relativeAngle;
    ChassisYawSpeedPid.fdb = 0;
    ChassisYawSpeedPid.UpdateResult();
    Vw = ChassisYawSpeedPid.result;

    Vx = VxFilter.Update(Vx);
    Vy = VyFilter.Update(Vy);
    Vw = VwFilter.Update(Vw);

    // xy方向速度
    memcpy(xyMsg.data, &Vx, 4);
    memcpy(xyMsg.data + 4, &Vy, 4);
    BoardConnectivity::Instance()->Add2Memory(xyMsg);

    // 角速度和云台底盘的相对角度
    memcpy(wAndAngleMsg.data, &Vw, 4);
    float TempRelativeAngle = 0; // 在底盘跟随模式下，需要将相对角度置0
    memcpy(wAndAngleMsg.data + 4, &TempRelativeAngle, 4);
    BoardConnectivity::Instance()->Add2Memory(wAndAngleMsg);
}

void ChassisStateFPS::Exit()
{
}
