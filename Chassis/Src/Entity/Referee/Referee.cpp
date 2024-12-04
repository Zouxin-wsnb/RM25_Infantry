#include "Referee.hpp"

// 以下发送的UI均为测试内容
bool Visibility;
char str[] = "Man!What can I say!";
Uiid ID;

Referee::Referee()
{
}

Referee::~Referee()
{
}

void Referee::Init()
{
    JLUI_SetSenderReceiverId(RobotId::RedInfantry_3,RobotId::ClientRedInfantry_3);
    JLUI_CreateLine(10,UiCyan,2,110,335,1810,745);
	JLUI_CreateRect(10,UiCyan,2,110,335,1810,745);
	JLUI_CreateString(5,UiBlack,2,100,700,50,str);
    JLUI_CreateFloat(5,UiOrange,2,700,800,50,3.14);
    JLUI_CreateArc(10,UiPink,2,1000,700,20,30,45,180);
    JLUI_CreateEllipse(10,UiYellow,3,1000,700,80,100);
	ID = JLUI_CreateCircle(10,UiGreen,1,700,400,8);
}

void Referee::Update()
{
    if (Time::GetTick() % 100 == 0)
    {
        if (Visibility)
            Visibility = false;
        else
            Visibility = true;
//        JLUI_SetRadius(ID,Referee::Instance()->PowerHeatData.chassis_power);
//		JLUI_CreateLine(10,UiCyan,1,710,335,1210,745);
				JLUI_SetVisible(ID,Visibility);
        JLUI_10HzTick();
    }
    TimeOutAdjust();
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
            case JudgeID::GameStatus:
                memcpy(&GameStatus, pData + start, sizeof(GameStatus));
                break;
    
            case JudgeID::GameResult:
                memcpy(&GameResult, pData + start, sizeof(GameResult));
                break;

            case JudgeID::RobotHP:
                memcpy(&RobotHP, pData + start, sizeof(RobotHP));
                break;
                    
            case JudgeID::EventData:
                memcpy(&EventData, pData + start, sizeof(EventData));
                break;
                    
            case JudgeID::SupplyProjectileAction:
                memcpy(&BulletRemaining, pData + start, sizeof(SupplyProjectileAction));
                break;
                    
            case JudgeID::RefereeWarning:
                memcpy(&RefereeWarning, pData + start, sizeof(RefereeWarning));
                break;
                    
            case JudgeID::DartInfo:
                memcpy(&DartInfo, pData + start, sizeof(DartInfo));
                break;
                    
            case JudgeID::GameRobotStatus:
                memcpy(&GameRobotStatus, pData + start, sizeof(GameRobotStatus));
                GameRobotStatusTick = Time::GetTick();
                break;
                    
            case JudgeID::PowerHeatData:
                memcpy(&PowerHeatData, pData + start, sizeof(PowerHeatData));
                PowerHeatTick = Time::GetTick();
                break;
            
            case JudgeID::GameRobotPos:
                memcpy(&GameRobotPos, pData + start, sizeof(GameRobotPos));
                break;
                    
            case JudgeID::Buff:
                memcpy(&Buff, pData + start, sizeof(Buff));
                break;
                    
            case JudgeID::AirSupportData:
                memcpy(&AirSupportData, pData + start, sizeof(AirSupportData));
                break;
                    
            case JudgeID::RobotHurt:
                memcpy(&RobotHurt, pData + start, sizeof(RobotHurt));
                break;

            case JudgeID::ShootData:
                memcpy(&ShootData, pData + start, sizeof(ShootData));
                break;

            case JudgeID::RfidStatus:
                memcpy(&RfidStatus, pData + start, sizeof(RfidStatus));
                break;

            case JudgeID::DartClientCmd:
                memcpy(&DartClientCmd, pData + start, sizeof(DartClientCmd));
                break;
            
            case JudgeID::RoboInteractData:
                memcpy(&RoboInteractData, pData + start, sizeof(RoboInteractData));
                // HandleCustomPacket();
                // uint16_t _len = 0;
                // memcpy(&_len, &m_CurrentFrame[1], sizeof(uint16_t));
                // _len -= sizeof(JS_CustomHeader);
                // HandleNewMsg(_ptr, _len);
                break;

            default:
                break;
            }
            start += Header.dataLength + 2; // 跳过数据和crc16
        }
        else
            break;
    }
}

RobotId Referee::GetClientID()
{
    uint8_t ID = Referee::Instance()->GameRobotStatus.robot_id;

    RobotId ClientID;
    switch (ID)
    {
    case RedHero:
        ClientID = ClientRedHero;
        break;

    case RedEngineer:
        ClientID = ClientRedEngineer;
        break;

    case RedInfantry_3:
        ClientID = ClientRedInfantry_3;
        break;

    case RedInfantry_4:
        ClientID = ClientRedInfantry_4;
        break;                
    
    case RedInfantry_5:
        ClientID = ClientRedInfantry_5;
        break;

    case RedDrone:
        ClientID = ClientRedDrone;
        break;

    case BlueHero:
        ClientID = ClientBlueHero;
        break;

    case BlueEngineer:
        ClientID = ClientBlueEngineer;
        break;

    case BlueInfantry_3:
        ClientID = ClientBlueInfantry_3;
        break;

    case BlueInfantry_4:
        ClientID = ClientBlueInfantry_4;
        break;                
    
    case BlueInfantry_5:
        ClientID = ClientBlueInfantry_5;
        break;

    case BlueDrone:
        ClientID = ClientBlueDrone;
        break;

    default:
        break;
    }
		
	return ClientID;
}

PowerHeatData_t Referee::GetPowerHeatData()
{
    return PowerHeatData;
}

void Referee::TimeOutAdjust()
{
    if(Time::GetTick() > 1000 + PowerHeatTick)
    {
        PowerHeatData.chassis_power_buffer = 50;
    }
    if(Time::GetTick() > 1000 + GameRobotStatusTick)
    {
        GameRobotStatus.chassis_power_limit = 45;
        GameRobotStatus.power_management_chassis_output = 1;
        GameRobotStatus.shooter_barrel_heat_limit = 240;
    }
}

