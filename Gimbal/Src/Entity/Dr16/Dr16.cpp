#include "Dr16.hpp"

float Dr16::MapAvix(int16_t ch)
{
    return (float)ch / (float)RC_CH_OFFSET_MAX;
}

Dr16::Dr16()
{
}

Dr16::~Dr16()
{
}

void Dr16::Dr16_Init()
{
    rc_raw = get_remote_control_raw(); // 获取遥控器原始数据指针

    remote_control_init(); // 初始化遥控器

    Monitor::Instance()->Log_Messages(Monitor::INFO, (uint8_t *)("Dr16 Init Success!\r\n")); // 串口输出初始化成功信息
}

float Dr16::GetRightX()
{
    return MapAvix(rc_raw->rc.ch[0]);
}

float Dr16::GetRightY()
{
    return MapAvix(rc_raw->rc.ch[1]);
}

float Dr16::GetLeftX()
{
    return MapAvix(rc_raw->rc.ch[2]);
}

float Dr16::GetLeftY()
{
    return MapAvix(rc_raw->rc.ch[3]);
}

Dr16::PC_KEY_STATE Dr16::QueryPcKeyStatus(PC_KEY_TYPE key)
{
    uint16_t mask = (uint16_t)key;
    uint16_t current = CurrentKeyState & mask;
    uint16_t previous = PreviousKeyState & mask;

    if (current && previous)
    {
        return PC_KEY_DOWN;
    }
    else if (!current && !previous)
    {
        return PC_KEY_UP;
    }
    else if (previous && !current)
    {
        return PC_KEY_FALL;
    }
    else
    {
        return PC_KEY_RISE;
    }
}

Dr16::RC_SWITCH_STATE Dr16::QuerySwStatus(RC_SWITCH_TYPE sw)
{
    switch (sw)
    {
    case LEFT_SWITCH:
        return Left_CurrentSw;
    case RIGHT_SWITCH:
        return Right_CurrentSw;
    case LEFT_SWITCH_CHANGE:
        return Left_SwChange;
    case RIGHT_SWITCH_CHANGE:
        return Right_SwChange;
    default:
        return RC_SW_DOWN; // 默认给出开关下的状态，这个可以考虑在赛场上采用底盘跟随的方式，调试时采用对应relax模式
    }
}

void Dr16::UpdateRcStatus()
{
    // 如果左侧开关状态发生变化，记录变化前的状态
    if (Left_CurrentSw != rc_raw->rc.s[1])
        Left_PreviousSw = Left_CurrentSw;
    if (Right_CurrentSw != rc_raw->rc.s[0])
        Right_PreviousSw = Right_CurrentSw;

    // 更新左右侧开关状态
    Left_CurrentSw = (RC_SWITCH_STATE)rc_raw->rc.s[1];
    Right_CurrentSw = (RC_SWITCH_STATE)rc_raw->rc.s[0];

    if (Left_PreviousSw == RC_SW_UP && Left_CurrentSw == RC_SW_MID)
        Left_SwChange = RC_SWITCH_U2M;
    else if (Left_PreviousSw == RC_SW_MID && Left_CurrentSw == RC_SW_UP)
        Left_SwChange = RC_SWITCH_M2U;
    else if (Left_PreviousSw == RC_SW_MID && Left_CurrentSw == RC_SW_DOWN)
        Left_SwChange = RC_SWITCH_M2D;
    else if (Left_PreviousSw == RC_SW_DOWN && Left_CurrentSw == RC_SW_MID)
        Left_SwChange = RC_SWITCH_D2M;

    if (Right_PreviousSw == RC_SW_UP && Right_CurrentSw == RC_SW_MID)
        Right_SwChange = RC_SWITCH_U2M;
    else if (Right_PreviousSw == RC_SW_MID && Right_CurrentSw == RC_SW_UP)
        Right_SwChange = RC_SWITCH_M2U;
    else if (Right_PreviousSw == RC_SW_MID && Right_CurrentSw == RC_SW_DOWN)
        Right_SwChange = RC_SWITCH_M2D;
    else if (Right_PreviousSw == RC_SW_DOWN && Right_CurrentSw == RC_SW_MID)
        Right_SwChange = RC_SWITCH_D2M;
}

float Dr16::GetMouseX()
{
    return rc_raw->mouse.x*0.00003;
}

float Dr16::GetMouseY()
{
    return rc_raw->mouse.y*0.00003;
}

void Dr16::UpdateKeyStatus()
{
    PreviousKeyState = CurrentKeyState;
    CurrentKeyState = rc_raw->key.v;
}

uint8_t Dr16::L_Press()
{
    return rc_raw->mouse.press_l;
}

uint8_t Dr16::R_Press()
{
    return rc_raw->mouse.press_r;
}

void Dr16::Update()
{
    UpdateRcStatus();
    UpdateKeyStatus();
}
