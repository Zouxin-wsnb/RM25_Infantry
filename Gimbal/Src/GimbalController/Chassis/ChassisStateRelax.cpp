#include "ChassisStateRelax.hpp"

void ChassisStateRelax::Init()
{
    Vx = 0.0f;
    Vy = 0.0f;
    Vw = 0.0f;
}

void ChassisStateRelax::Enter()
{
    Vx = 0.0f;
    Vy = 0.0f;
    Vw = 0.0f;

    xyMsg.type = BOARD_CONNECTIVITY_CAN_2;
    xyMsg.id = 0xB1;
    xyMsg.len = 8;

    wAndAngleMsg.type = BOARD_CONNECTIVITY_CAN_2;
    wAndAngleMsg.id = 0xB2;
    wAndAngleMsg.len = 8;
}

void ChassisStateRelax::Execute()
{
    memcpy(xyMsg.data, &Vx, 4);
    memcpy(xyMsg.data + 4, &Vy, 4);
    BoardConnectivity::Instance()->Add2Memory(xyMsg);

    memcpy(wAndAngleMsg.data, &Vw, 4);
    memcpy(wAndAngleMsg.data + 4, &Vw, 4);
    BoardConnectivity::Instance()->Add2Memory(wAndAngleMsg);
}

void ChassisStateRelax::Exit()
{
}

