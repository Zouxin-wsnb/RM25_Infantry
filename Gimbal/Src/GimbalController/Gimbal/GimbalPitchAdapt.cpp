#include "GimbalPitchAdapt.hpp"

void GimbalStatePitchAdapt::Init()
{
    // 新机器人下代码之后需要使用debug查看电机位置并修改下面的值
    PitchLowerLimit = 0.460961252; // Pitch轴电机最小角度
    PitchUpperLimit = 5.97485542;  // Pitch轴电机最大角度
		AdaptFinished = false;
}

void GimbalStatePitchAdapt::Enter()
{
	// 云台Pitch轴电机
    PitchMotor->controlMode = GM6020::POS_MODE;
    // 速度环PID参数
    PitchMotor->speedPid.kp = 600.0f;
    PitchMotor->speedPid.ki = 0.0f;
    PitchMotor->speedPid.kd = 0.0f;
    PitchMotor->speedPid.maxIOut = 3.0f;
    PitchMotor->speedPid.maxOut = 25000.0f;
    // 位置环PID参数
    PitchMotor->positionPid.kp = 100.0f;
    PitchMotor->positionPid.ki = 0.0f;
    PitchMotor->positionPid.kd = 0.0f;
    PitchMotor->positionPid.maxIOut = 3.0f;
    PitchMotor->positionPid.maxOut = 25000.0f;

    PitchSet = PitchMotor->motorFeedback.positionFdb;
}

void GimbalStatePitchAdapt::Execute()
{
    PitchSet = 3.095f;
    PitchMotor->positionSet = PitchSet;
    // 外环控制，位置环控制
    PitchMotor->positionPid.ref = PitchMotor->positionSet;
    PitchMotor->positionPid.fdb = PitchMotor->motorFeedback.positionFdb;
    if (PitchMotor->positionPid.ref - PitchMotor->positionPid.fdb >= Math::Pi)
    {
        PitchMotor->positionPid.ref -= Math::PiX2;
    }
    else if (PitchMotor->positionPid.ref - PitchMotor->positionPid.fdb <= -Math::Pi)
    {
        PitchMotor->positionPid.ref += Math::PiX2;
    }

    PitchMotor->positionPid.UpdateResult();

    // 内环控制，速度环控制
    PitchMotor->speedPid.ref = PitchMotor->positionPid.result;
    PitchMotor->speedPid.fdb = PitchMotor->motorFeedback.speedFdb;
    PitchMotor->speedPid.UpdateResult();

    PitchMotor->currentSet = PitchMotor->speedPid.result; // 根据速度PID结果设置电流
}

void GimbalStatePitchAdapt::Exit()
{

}