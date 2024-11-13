#include "ShooterStateWarm.hpp"
void ShooterStateWarm::Init() {}

void ShooterStateWarm::Enter() {}

void ShooterStateWarm::Execute()
{
    LeftFricMotor->speedSet = -10 * 19;
    RightFricMotor->speedSet = 10 * 19;
    TriggerMotor->speedSet = 0;

    LeftFricMotor->setOutput();
    RightFricMotor->setOutput();
    TriggerMotor->setOutput();
}

void ShooterStateWarm::Exit() {}
