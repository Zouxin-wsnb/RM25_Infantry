#include "ShooterStateRelax.hpp"

void ShooterStateRelax::Init() {}

void ShooterStateRelax::Enter()
{
    LeftFricMotor->speedSet = 0;
    RightFricMotor->speedSet = 0;
    TriggerMotor->speedSet = 0;
}

void ShooterStateRelax::Execute()
{
    LeftFricMotor->speedSet = 0;
    RightFricMotor->speedSet = 0;
    TriggerMotor->speedSet = 0;

    LeftFricMotor->setOutput();
    RightFricMotor->setOutput();
    TriggerMotor->setOutput();
}

void ShooterStateRelax::Exit() {}
