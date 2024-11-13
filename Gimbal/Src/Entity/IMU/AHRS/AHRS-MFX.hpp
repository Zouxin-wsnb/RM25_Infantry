#ifndef _AHRS_MFX_HPP
#define _AHRS_MFX_HPP

#include "math.h"
#include "BMI088.hpp"
#include "bsp_dwt.hpp"
#include "arm_math.h"
#include "stdint.h"
#include "string.h"
#include "Monitor.hpp"
#include "motion_fx.h"

typedef struct
{
    float q[4]; // 四元数估计值
    // 位姿
    float Roll;
    float Pitch;
    float Yaw;
    float YawTotalAngle;
    float LastYaw;
    int16_t YawRoundCount;
} INS_Data;

class AHRS
{
public:
    AHRS() {}
    ~AHRS() {}

    float AHRS_Time;
    float dt;

    void INS_Init();

    void AHRS_Update();

    INS_Data INS;

    static AHRS *Instance()
    {
        static AHRS ahrs;
        return &ahrs;
    }
};

#endif // !_AHRS_HPP
