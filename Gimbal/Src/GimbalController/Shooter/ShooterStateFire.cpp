#include "ShooterStateFire.hpp"

void ShooterStateFire::Init() {}

void ShooterStateFire::Enter()
{
    LeftFricMotor->speedSet = -10 * 19;
    RightFricMotor->speedSet = 10 * 19;
    TriggerMotor->speedSet = 0;
}

void ShooterStateFire::Execute()
{
    LeftFricMotor->speedSet = -800;
    RightFricMotor->speedSet = 800;
    TriggerMotor->speedSet = -2*36;

    LeftFricMotor->setOutput();
    RightFricMotor->setOutput();
    TriggerMotor->setOutput();
}

void ShooterStateFire::Exit() {}
