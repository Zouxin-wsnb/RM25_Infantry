#include "ShooterStateKeyControl.hpp"
int FricSpeed = 800;
int TriggerSpeed = 300;
void ShooterStateKeyControl::Init() {}

void ShooterStateKeyControl::Enter()
{
    LeftFricMotor->speedSet = -10 * 19;
    RightFricMotor->speedSet = 10 * 19;
    TriggerMotor->speedSet = 0;
}

void ShooterStateKeyControl::Execute()
{
    LeftFricMotor->speedSet = -FricSpeed;
    RightFricMotor->speedSet = FricSpeed;

    // if(Dr16::Instance()->QueryPcKeyState(Dr16::PC_KEY_Z,Dr16::KEY_DOWN))
    // {
    // 	TriggerMotor->speedSet = 20;
    // }
    // else if(Dr16::Instance()->QueryPcKeyState(Dr16::PC_KEY_X,Dr16::KEY_DOWN))
    // {
    // 	DEBUG_trigger_spd_set = 35;
    // }

    // 右键正转左键反转
    if (Dr16::Instance()->R_Press())
    {
        TriggerMotor->speedSet = -TriggerSpeed;
        // 堵转电流过大自动反转，大堵转时没有效果
        // if (TriggerMotor->motorFeedback.currentFdb <= -6000)
        //     TriggerMotor->speedSet = TriggerSpeed;
    }
    else if (Dr16::Instance()->L_Press())
    {
        TriggerMotor->speedSet = TriggerSpeed;
    }
    else
    {
        TriggerMotor->speedSet = 0;
    }

    LeftFricMotor->setOutput();
    RightFricMotor->setOutput();
    TriggerMotor->setOutput();
}

void ShooterStateKeyControl::Exit() {}
