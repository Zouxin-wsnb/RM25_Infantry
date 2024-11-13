#include "Referee.hpp"

Referee::Referee()
{
}

Referee::~Referee()
{
}

void Referee::ProcessData(uint8_t *pData, uint16_t Size)
{
    FrameHeader Header;
    uint8_t start = 0;
    // for (; start < Size; start++)
    // {
    //     if (pData[start] == 0xA5)
    //     {
    //         break;
    //     }
    //     else if (start == Size - 1)
    //     {
    //         Monitor::Instance()->Log_Messages(Monitor::ERROR, (uint8_t *)"Referee data error\n\r");
    //         return;
    //     }
    // }
    // // 读取帧头
    // memcpy(&Header, pData + start, sizeof(FrameHeader));
    // // 检查帧头crc8
    // if (!Verify_CRC8_Check_Sum(pData + start, sizeof(FrameHeader)))
    // {
    //     Monitor::Instance()->Log_Messages(Monitor::ERROR, (uint8_t *)"Referee data crc8 error\n\r");
    //     return;
    // }
    // start += sizeof(FrameHeader);
    // uint16_t cmd_id = 0;
    // memcpy(&cmd_id, pData + start, sizeof(uint16_t));
    // // 读取数据
    // switch (cmd_id)
    // {
    // case 0x0201:
    //     memcpy(&GameRobotStatus, pData + start + 2, sizeof(GameRobotStatus_t));
    //     break;
    // case 0x0202:
    //     memcpy(&PowerHeatData, pData + start + 2, sizeof(PowerHeatData_t));
    //     break;
    // }
    for (; start < Size; start++)
    {
        if (pData[start] == 0xA5) // 发现SOF
        {
            // 检查帧头crc8
            if (!Verify_CRC8_Check_Sum(pData + start, sizeof(FrameHeader)))
            {
                continue; // crc8错误，跳过
            }

            // 读取帧头
            memcpy(&Header, pData + start, sizeof(FrameHeader));
            // 整包校验crc16
            if (!Verify_CRC16_Check_Sum(pData + start, 9 + Header.dataLength))
            {
                continue; // crc16错误，跳过
            }
            start += sizeof(FrameHeader); // 跳过帧头
            uint16_t cmd_id = 0;
            memcpy(&cmd_id, pData + start, sizeof(uint16_t)); // 读取命令ID
            start += sizeof(uint16_t);                        // 跳过命令ID
            switch (cmd_id)
            {
            case 0x0201:
                memcpy(&GameRobotStatus, pData + start, sizeof(GameRobotStatus_t));
                break;
            case 0x0202:
                memcpy(&PowerHeatData, pData + start, sizeof(PowerHeatData_t));
                break;
            }
            start += Header.dataLength + 2; // 跳过数据和crc16
        }
    }
}

PowerHeatData_t Referee::GetPowerHeatData()
{
    return PowerHeatData;
}