#include "GimbalStateRelax.hpp"

void GimbalStateRelax::Init()
{
    // 初始化放松状态
    YawMotor->controlMode = GM6020::RELAX_MODE;
    YawMotor->speedPid.Clear();
    YawMotor->positionPid.Clear();

    PitchMotor->controlMode = GM6020::RELAX_MODE;
    PitchMotor->speedPid.Clear();
    PitchMotor->positionPid.Clear();
}

void GimbalStateRelax::Enter()
{
    // 初始化放松状态
    YawMotor->controlMode = GM6020::RELAX_MODE;
    PitchMotor->controlMode = GM6020::RELAX_MODE;
}

void GimbalStateRelax::Execute()
{
    YawMotor->setOutput();
    PitchMotor->setOutput();
}

void GimbalStateRelax::Exit()
{
    // 退出放松状态
}

