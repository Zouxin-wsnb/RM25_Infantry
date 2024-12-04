#ifndef _AHRS_ICM42688_HPP
#define _AHRS_ICM42688_HPP

#include "math.h"
#include "bsp_dwt.hpp"
#include "arm_math.h"
#include "stdint.h"
#include "string.h"
#include "Monitor.hpp"
#include "BoardConnectivity.hpp"
#include "Math.hpp"

#define IMU_MSG_ID 0xC5 // ICM42688数据接收ID

typedef struct
{
    float q[4]; // 四元数估计值
    // 位姿
    float Pitch;
    float Yaw;
    float Roll;
    float YawTotalAngle;
} INS_Data;

class AHRS
{
public:
    AHRS() {}
    ~AHRS() {}

    void INS_Init();

    void AHRS_Update();

    //直接指向IMU数据的指针
    BoardMsg *IMU_Msg_1;
    BoardMsg *IMU_Msg_2;

    INS_Data INS;

    static AHRS *Instance()
    {
        static AHRS ahrs;
        return &ahrs;
    }
};

#endif // !_AHRS_HPP
