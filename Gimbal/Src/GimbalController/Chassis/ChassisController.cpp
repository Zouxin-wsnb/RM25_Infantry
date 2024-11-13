#include "ChassisController.hpp"

void ChassisController::Init()
{
    SetCurrentState(&chassisStateRelax); // 设置当前状态为放松状态
    SetDefaultState(&chassisStateRelax); // 设置默认状态为放松状态

    chassisStateRelax.Init();         // 初始化放松状态
    chassisStateRemoteControl.Init(); // 初始化遥控状态
    chassisStateRotate.Init();        // 初始化状态
    chassisStateFPS.Init();
    // 日志输出
    Monitor::Instance()->Log_Messages(Monitor::INFO, (uint8_t *)("ChassisController Init Success!\r\n"));
}

void ChassisController::HandleInput()
{
    // 遥控器拨杆切换状态
//    if (Dr16::Instance()->QuerySwStatus(Dr16::LEFT_SWITCH) == Dr16::RC_SW_DOWN) // 如果遥控器左边的拨杆向下
//    {
//        ChangeState(&chassisStateRelax);
//    }
//    else if (Dr16::Instance()->QuerySwStatus(Dr16::LEFT_SWITCH) == Dr16::RC_SW_MID) // 如果遥控器左边的拨杆中间
//    {
//        ChangeState(&chassisStateFPS);
//    }
//    else if (Dr16::Instance()->QuerySwStatus(Dr16::LEFT_SWITCH) == Dr16::RC_SW_UP) // 如果遥控器左边的拨杆向上
//    {
//        ChangeState(&chassisStateRotate);
//    }
//    else
//    {
//        ChangeState(&chassisStateRelax);
//    }

    // 键盘切换状态
    if (Dr16::Instance()->QueryPcKeyStatus(Dr16::PC_KEY_R) == Dr16::PC_KEY_DOWN)
        ChangeState(&chassisStateRelax);
    else if (Dr16::Instance()->QueryPcKeyStatus(Dr16::PC_KEY_F) == Dr16::PC_KEY_DOWN)
        ChangeState(&chassisStateFPS);
    else if (Dr16::Instance()->QueryPcKeyStatus(Dr16::PC_KEY_G) == Dr16::PC_KEY_DOWN)
        ChangeState(&chassisStateRotate);

    // 放松模式保护
    if (Dr16::Instance()->QuerySwStatus(Dr16::LEFT_SWITCH) == Dr16::RC_SW_DOWN) // 如果遥控器左边的拨杆向下
    {
        ChangeState(&chassisStateRelax);
    }
}

void ChassisController::Run()
{
    HandleInput();
    Update();
}