#include "BoardConnectivity.hpp"

// 对于C板，有两个can口，分别是hcan1和hcan2，如果使用了其他的板子，需要修改这里
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

BoardConnectivity::BoardConnectivity()
{
    Init();
}

BoardConnectivity::~BoardConnectivity()
{
}

void BoardConnectivity::Init()
{
    for (int i = 0; i < 8; i++)
    {
        BoardMemory_receive[i].len = 0;
        BoardMemory_receive[i].type = BOARD_CONNECTIVITY_CAN_2;

        BoardMemory_send[i].len = 0;
        BoardMemory_send[i].type = BOARD_CONNECTIVITY_CAN_2;
    }
    Monitor::Instance()->Log_Messages(Monitor::INFO, (uint8_t *)("BoardConnectivity Init Success!\r\n"));
}

void BoardConnectivity::Add2Memory(BoardMsg &msg)
{
    uint8_t len = msg.len;
    uint16_t id = msg.id;
    if (id >= 0xB1 && id <= 0xB8) // 接受的数据
    {
        id -= 0xB1;
        BoardMemory_receive[id].id = id + 0xB1;
        BoardMemory_receive[id].len = len;
        BoardMemory_receive[id].type = msg.type;
        memcpy(BoardMemory_receive[id].data, msg.data, len);
        return;
    }
    else if (id >= 0xC1 && id <= 0xC8) // 发送的数据
    {
        id -= 0xC1;

        BoardMemory_send[id].id = id + 0xC1;
        BoardMemory_send[id].len = len;
        BoardMemory_send[id].type = msg.type;
        // BoardMemory_send[id].data = msg.data; // 传递指针
        memcpy(BoardMemory_send[id].data, msg.data, len);
        return;
    }
    else
    {
        // 日志输出, id不在范围内
        return;
    }
}

void BoardConnectivity::GetBoardMemory(uint8_t id, uint8_t *data, uint8_t len)
{
    if (id >= 0xC1 && id <= 0xC8) // 对于底盘C板，发送的id范围为0xCx
    {
        id -= 0xC1;
        if (BoardMemory_send[id].len != 0 && len <= BoardMemory_send[id].len)
        {
            memcpy(data, BoardMemory_send[id].data, len);
        }
    }
    else if (id >= 0xB1 && id <= 0xB8) // 对于底盘C板，接受的id范围为0xBx
    {
        id -= 0xB1;
        if (BoardMemory_receive[id].len != 0 && len <= BoardMemory_receive[id].len)
        {
            memcpy(data, BoardMemory_receive[id].data, len);
        }
    }
    else
    {
        // 日志输出, id不在范围内
    }
}

void BoardConnectivity::ClearBoardMemory(uint8_t id)
{
    if (id >= 0xC1 && id <= 0xC8) // 对于底盘C板，发送的id范围为0xCx
    {
        id -= 0xC1;
        BoardMemory_send[id].len = 0;
    }
    else if (id >= 0xB1 && id <= 0xB8) // 对于底盘C板，接受的id范围为0xBx
    {
        id -= 0xB1;
        BoardMemory_receive[id].len = 0;
    }
    else
    {
        // 日志输出, id不在范围内
    }
}

void BoardConnectivity::BoardConnectivity_Send()
{
    for (int i = 0; i < 8; i++)
    {
        if (BoardMemory_send[i].len != 0)
        {
            uint8_t len = BoardMemory_send[i].len;
            if (len <= 8)
            {
                switch (BoardMemory_send[i].type)
                {
                case BOARD_CONNECTIVITY_CAN_1:
                    CAN_Transmit(&hcan1, BoardMemory_send[i].id, BoardMemory_send[i].data, BoardMemory_send[i].len);
                    break;
                case BOARD_CONNECTIVITY_CAN_2:
                    CAN_Transmit(&hcan2, BoardMemory_send[i].id, BoardMemory_send[i].data, BoardMemory_send[i].len);
                    break;
                case BOARD_CONNECTIVITY_USART:
                    // USART_SendData(BoardMemory_send[i].data, BoardMemory_send[i].len);// 尚未完成
                    break;
                default:
                    break;
                }
            }
            else
            {
                // 日志输出, 数据长度超过8
            }
        }
    }
}
