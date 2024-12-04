#include "GimbalStateFPS.hpp"
#include "GimbalController.hpp"
#include "Chassis/ChassisController.hpp"

void GimbalStateFPS::Init()
{
    PitchLowerLimit = -25 * Math::DegreeToRad; // Pitch轴电机最小角度 //-35
    PitchUpperLimit = 32 * Math::DegreeToRad;  // Pitch轴电机最大角度 //22

    PitchSetFilter.SetTau(0.05f);      //  设置滤波时间常数
    PitchSetFilter.SetUpdatePeriod(1); // 设置更新周期

    YawPosSetFilter.SetQ(0.001);
    YawPosSetFilter.SetR(0.543);

    YawSpdSetFilter.SetQ(0.001);
    YawSpdSetFilter.SetR(0.800);

    ReceivePacket->header = 0x00; // 初始化视觉数据包
    ReceivePacket->tracking = 1;
    ReceivePacket->fire = false;
    ReceivePacket->id = 3;
    ReceivePacket->pitch = 0.0f;
    ReceivePacket->yaw = 0.0f;

    ForwardVw = 0.0f; // 初始化底盘状态
}

void GimbalStateFPS::Enter()
{
    // 初始化云台电机位置
    YawSet = AHRS::Instance()->INS.YawTotalAngle * Math::DegreeToRad; // 将初始位置设置为位姿解算的的Yaw
    PitchSet = (AHRS::Instance()->INS.Pitch * Math::DegreeToRad);     //< Pitch轴电机初始化位置为当前位置

    /*-----------------------------------------云台Yaw轴电机-----------------------------------------*/
    YawMotor->controlMode = GM6020::SPD_MODE; // 单环控制？
    // 位置环PID参数
    YawMotor->positionPid.mode = PID_POSITION | PID_Integral_Limit | PID_Derivative_On_Measurement;
    YawMotor->positionPid.kp = 20.0f;
    YawMotor->positionPid.ki = 0.015f;
    YawMotor->positionPid.kd = 1500.0f;
    YawMotor->positionPid.ScalarA = 6.0f;
    YawMotor->positionPid.ScalarB = 0.01f;
    YawMotor->positionPid.maxIOut = 1.0f;
    YawMotor->positionPid.maxOut = 25000.0f;
    // 速度环PID参数
    YawMotor->speedPid.mode = PID_POSITION;
    YawMotor->speedPid.kp = 3000.0f;
    YawMotor->speedPid.ki = 1.0f;
    YawMotor->speedPid.kd = 500.0f;
    YawMotor->speedPid.maxIOut = 3000.0f;
    YawMotor->speedPid.maxOut = 25000.0f;
    /*----------------------------------------云台Pitch轴电机----------------------------------------*/
    PitchMotor->controlMode = GM6020::SPD_MODE;
    // 位置环PID参数
    PitchMotor->positionPid.mode = PID_POSITION;
    PitchMotor->positionPid.kp = 25.0f;
    PitchMotor->positionPid.ki = 0.02f;
    PitchMotor->positionPid.kd = 0.1f;
    PitchMotor->positionPid.maxIOut = 3.0f;
    PitchMotor->positionPid.maxOut = 25000.0f;
    // 速度环PID参数
    PitchMotor->speedPid.mode = PID_POSITION;
    PitchMotor->speedPid.kp = 3000.0f;
    PitchMotor->speedPid.ki = 0.0f;
    PitchMotor->speedPid.kd = 5.0f;
    PitchMotor->speedPid.maxIOut = 0.0f;
    PitchMotor->speedPid.maxOut = 25000.0f;
}

#ifdef KeyBoardControl
void GimbalStateFPS::Execute()
{
    // 接受遥控器输入
    YawSet += Dr16::Instance()->GetRightX() * 0.008f; // 不需要进行限幅，因为使用了totalAngle
    PitchSet += Dr16::Instance()->GetRightY() * 0.005f;

    YawSet += Dr16::Instance()->GetMouseX() * 0.00003f;
    PitchSet -= Dr16::Instance()->GetMouseY() * 0.00003f;

    PitchSet = Math::FloatConstrain(PitchSet, PitchLowerLimit, PitchUpperLimit); // 将Pitch轴电机的角度范围限幅到[PitchLowerLimit, PitchUpperLimit]

    // 根据底盘状态设置Yaw轴电机的前馈
    if (ChassisController::Instance()->GetCurrentState() == &ChassisController::Instance()->chassisStateFPS)
    {
        ForwardVw = ChassisController::Instance()->chassisStateFPS.Vw;
    }
    else if (ChassisController::Instance()->GetCurrentState() == &ChassisController::Instance()->chassisStateRotate)
    {
        ForwardVw = ChassisController::Instance()->chassisStateRotate.Vw;
    }
    else
    {
        ForwardVw = 0.0f;
    }

    AutoAim(); // 自瞄程序

    YawMotor->positionPid.ref = YawPosSetFilter.Update(YawSet);
    YawMotor->positionPid.fdb = (AHRS::Instance()->INS.YawTotalAngle * Math::DegreeToRad);

    YawMotor->positionPid.UpdateResult();

    // 设置电机输出
    YawMotor->speedSet = -(float)(YawMotor->positionPid.result + (2.5f * ForwardVw)); // 根据速度PID结果设置电流 + 前馈控制
    YawMotor->setOutput();                                                            // 设置电机输出

    /*-----------------------------------------PITCH-----------------------------------------*/
    // Pirtch位置设置一阶滤波
    PitchSetFilter.SetInput(PitchSet);
    PitchSetFilter.Update();

    // Pitch PID计算
    PitchMotor->positionPid.ref = PitchSetFilter.GetResult();
    PitchMotor->positionPid.fdb = AHRS::Instance()->INS.Pitch * Math::DegreeToRad;
    PitchMotor->positionPid.UpdateResult();

    // 设置电机输出
    PitchMotor->speedSet = -PitchMotor->positionPid.result;
    PitchMotor->setOutput();
}
#else
#define RemoteControl
void GimbalStateFPS::Execute()
{
    // 接受遥控器输入
    YawSet += Dr16::Instance()->GetRightX() * 0.008f; // 不需要进行限幅，因为使用了totalAngle

    PitchSet += Dr16::Instance()->GetRightY() * 0.005f;
    PitchSet = Math::FloatConstrain(PitchSet, PitchLowerLimit, PitchUpperLimit); // 将Pitch轴电机的角度范围限幅到[PitchLowerLimit, PitchUpperLimit]

    // 根据底盘状态设置Yaw轴电机的前馈
    if (ChassisController::Instance()->GetCurrentState() == &ChassisController::Instance()->chassisStateFPS)
    {
        ForwardVw = ChassisController::Instance()->chassisStateFPS.Vw;
    }
    else if (ChassisController::Instance()->GetCurrentState() == &ChassisController::Instance()->chassisStateRotate)
    {
        ForwardVw = ChassisController::Instance()->chassisStateRotate.Vw;
    }
    else
    {
        ForwardVw = 0.0f;
    }

    AutoAim(); // 自瞄程序

    YawMotor->positionPid.ref = YawPosSetFilter.Update(YawSet);
    YawMotor->positionPid.fdb = (AHRS::Instance()->INS.YawTotalAngle * Math::DegreeToRad);

    YawMotor->positionPid.UpdateResult();

    // 设置电机输出
    YawMotor->speedSet = -(float)(YawMotor->positionPid.result + (2.5f * ForwardVw)); // 根据速度PID结果设置电流 + 前馈控制
    YawMotor->setOutput();                                                            // 设置电机输出

    /*-----------------------------------------PITCH-----------------------------------------*/
    // Pirtch位置设置一阶滤波
    PitchSetFilter.SetInput(PitchSet);
    PitchSetFilter.Update();

    // Pitch PID计算
    PitchMotor->positionPid.ref = PitchSetFilter.GetResult();
    PitchMotor->positionPid.fdb = AHRS::Instance()->INS.Pitch * Math::DegreeToRad;
    PitchMotor->positionPid.UpdateResult();

    // 设置电机输出
    PitchMotor->speedSet = -PitchMotor->positionPid.result;
    PitchMotor->setOutput();
}
#endif

void GimbalStateFPS::Exit()
{
    // 可以在这里执行一些清理工作，比如PID和滤波器的清零
}

void GimbalStateFPS::AutoAim()
{
    // 检查数据包是否正确
    if (ReceivePacket->header == 0xA5)
    {
        // 检查是CRC校验是否正确
        //        if (Verify_CRC16_Check_Sum((uint8_t *)&ReceivePacket, (uint32_t)12) == false)
        //        {
        //            ReceivePacket->header = 0x00;
        //            Monitor::Instance()->Log_Messages(Monitor::ERROR, (uint8_t *)"GimbalStateFPS::AutoAim: CRC16 Check Failed\n\r");
        //            return; // 如果CRC校验失败，直接返回
        //        }
        // 检查数据是否在浮点数范围内
        if (isnan(ReceivePacket->pitch) || isnan(ReceivePacket->yaw))
        {
            ReceivePacket->header = 0x00;
            Monitor::Instance()->Log_Messages(Monitor::ERROR, (uint8_t *)"GimbalStateFPS::AutoAim: data nan error \n\r");
            return; // 如果数据包中的pitch或yaw是nan，直接返回
        }
        // 更新Pitch和Yaw的值
        PitchSet = ReceivePacket->pitch;
        YawSet = Math::PiX2 * AHRS::Instance()->INS.YawRoundCount - ReceivePacket->yaw;
    }
    ReceivePacket->header = 0x00; // 清空数据包
}
