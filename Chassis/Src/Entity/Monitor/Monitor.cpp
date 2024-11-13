#include "Monitor.hpp"

extern UART_HandleTypeDef huart1;

Monitor::Monitor()
{
    Monitor_Init();
}

Monitor::~Monitor()
{
}

void Monitor::Monitor_RobotFinishInit()
{
    Log_Messages(Monitor::INFO, (uint8_t *)("Standard robot Init success!\r\n"));

    Is_Robot_Init = true;
}

void Monitor::Monitor_Init()
{
    for (int i = 0; i < 8; i++)
    {
        MotorMes_Probe[i].Is_Loged = false;
        MotorMes_Probe[i].Status = Probe_NORMAL;
        MotorMes_Probe[i].Update_Time_s = DWT_GetTimeline_s();
    }

    RemoteController_Probe.Status = Probe_NORMAL;
    RemoteController_Probe.Is_Loged = false;
    RemoteController_Probe.Update_Time_s = DWT_GetTimeline_s();

    IMU_Probe.Status = Probe_NORMAL;
    IMU_Probe.Is_Loged = false;

    Normal_CanFreq = 1000;
    Normal_USBRecvFreq = 0;
    Normal_MotorTemp = 0;

    Is_Robot_Init = false;

    Log_Messages(INFO, (uint8_t *)("Monitor Init Success!\r\n"));
}

Monitor::Monitor_Status Monitor::Log_Messages(LOGLEVEL level, uint8_t *message)
{
    if (huart1.gState != HAL_UART_STATE_READY) // 这里的设计不好，应该避免对硬件底层的交互，考虑在BSP层进行处理
    {
        return Monitor_LOG_BUSY;
    }

    uint8_t len = strlen((char *)message);
    // 预先定义所有可能前缀的长度
    const char *prefix = nullptr;
    uint8_t prefix_len = 0;

    // 根据日志级别选择前缀
    switch (level)
    {
    case INFO:
        prefix = "[INFO]";
        prefix_len = 6;
        break;

    case WARNING:
        prefix = "[WARNING]";
        prefix_len = 9;
        break;

    case ERROR:
        prefix = "[ERROR]";
        prefix_len = 7;
        break;

    default:
        return Monitor_LOG_ERROR; // 如果没有匹配的级别，直接返回
    }

    // 计算总的消息大小，并只分配一次内存
    uint8_t size = len + prefix_len;
    uint8_t *buffer = (uint8_t *)malloc(size);
    if (buffer == nullptr)
    {
        // 如果内存分配失败，直接返回
        return Monitor_LOG_ERROR;
    }

    // 将前缀和消息拷贝到缓冲区
    memcpy(buffer, prefix, prefix_len);
    memcpy(buffer + prefix_len, message, len);

    // 调用串口发送函数
    if (Is_Robot_Init == true)
    {
        USART_Transmit(&huart1, buffer, size, USART_MODE_DMA);
    }
    else
    {
        USART_Transmit(&huart1, buffer, size, USART_MODE_BLOCK);
    }
    // 释放内存
    free(buffer);
    return Monitor_LOG_OK;
}

void Monitor::Set_Normal_CanFreq(uint16_t freq)
{
    this->Normal_CanFreq = freq;
}

void Monitor::Set_Normal_USBRecvFreq(uint16_t freq)
{
    this->Normal_USBRecvFreq = freq;
}

void Monitor::Set_Normal_MotorTemp(uint8_t temp)
{
    this->Normal_MotorTemp = temp;
}

void Monitor::MotorMes_Monitor(uint8_t id)
{
    if (id >= 4)
    {
        Log_Messages(ERROR, (uint8_t *)("Motor ID is out of range!\r\n"));
        return;
    }
    MotorMes_Probe[id].Update_Time_s = DWT_GetTimeline_s();
}

void Monitor::Vision_USBRecvFreq_Monitor()
{
}

void Monitor::IMU_Monitor(float roll, float pitch, float yaw)
{
    IMU_Probe.Update_Time_s = DWT_GetTimeline_s(); // 更新时间

    // 如果出现nan错误，设置探针状态为错误
    if (isnan(roll) || isnan(pitch) || isnan(yaw))
    {
        IMU_Probe.Status = Probe_ERROR;
    }
    else
    {
        IMU_Probe.Status = Probe_NORMAL;
    }
}

void Monitor::RemoteController_Monitor()
{
    RemoteController_Probe.Update_Time_s = DWT_GetTimeline_s();
}

Monitor::Probe_Status Monitor::Monitor_QueryProbeStatus(Probe *probe)
{
    return probe->Status;
}

void Monitor::Monitor_Run()
{
    if (Monitor_Counter == 1 && Is_Robot_Init == true) // 控制监测频率
    {
        Monitor_Counter = 0;
        Monitor_Time = DWT_GetTimeline_s();
        Monitor_Status log_status;

        // 电机can通信包更新时间监测
        for (int i = 0; i < 4; i++)
        {
            if (fabs(Monitor_Time - MotorMes_Probe[i].Update_Time_s) > 0.1f) // 如果电机更新时间超过0.1s，发出警告
            {
                if (MotorMes_Probe[i].Status == Probe_NORMAL || MotorMes_Probe[i].Is_Loged == false)
                {
                    // status = Log_Messages(ERROR, (uint8_t *)("Motor ID %d is not updated!\r\n"), USART_MODE_DMA);
                    char log_message[50];                                       // 预留50个字节足够存储日志信息
                    sprintf(log_message, "Motor ID %d is not updated!\r\n", i); // 格式化日志消息
                    log_status = Log_Messages(ERROR, (uint8_t *)(log_message));
                    MotorMes_Probe[i].Status = Probe_ERROR;

                    if (log_status == Monitor_LOG_OK)
                    {
                        MotorMes_Probe[i].Is_Loged = true;
                    }
                }
            }
            else // 如果电机更新时间正常，恢复正常状态
            {
                if (MotorMes_Probe[i].Is_Loged == true && MotorMes_Probe[i].Status == Probe_ERROR)
                {
                    MotorMes_Probe[i].Is_Loged = false;
                    MotorMes_Probe[i].Status = Probe_NORMAL;
                    char log_message[32];                                        // 预留50个字节足够存储日志信息
                    sprintf(log_message, "Motor ID %d update restored!\r\n", i); // 格式化日志消息
                    Log_Messages(INFO, (uint8_t *)(log_message));
                }
            }
        }

        // IMU数据监测，注意是否出现了nan错误
        if (IMU_Probe.Status == Probe_ERROR && IMU_Probe.Is_Loged == false)
        {
            log_status = Log_Messages(ERROR, (uint8_t *)("IMU Data is not normal!\r\n"));
            if (log_status == Monitor_LOG_OK)
            {
                IMU_Probe.Is_Loged = true;
            }
        }
        // 超电
        // 视觉
        // 裁判系统
    }

    Monitor_Counter++;
}
