#include "IMU_Config.hpp"

#ifdef IMU_LIB_MOTION_FX_
#include "AHRS-MFX.hpp"
uint8_t MFX[0x980] = {0};                         // 用于存储MotionFX的数据
static MFXState_t motionFX_ptr = (MFXState_t)MFX; // MotionFX指针
MFX_knobs_t iKnobs;                               // MotionFX参数
MFX_input_t data_in;                              // MotionFX输入数据，来自IMU芯片
MFX_output_t data_out;                            /// MotionFX输出数据
float gbias[3] = {0, 0, -0.23131f};               // 陀螺仪偏置
int mfx_size = 0;                                 // MotionFX大小
static float InvG = 0.0f;                         // 重力加速度倒数

void AHRS::INS_Init()
{
    if (BMI088::Instance()->gNorm != 0.0f)
    {
        InvG = 1.0f / BMI088::Instance()->gNorm;
    }
    else
    {
        InvG = 0.1020408f;
    }

    mfx_size = MotionFX_GetStateSize();
    if (mfx_size == 0)
    {
        Monitor::Instance()->Log_Messages(Monitor::ERROR, (uint8_t *)("AHRS Init: MotionFX_GetStateSize Error!\r\n"));
        return;
    }
    MotionFX_initialize(motionFX_ptr);
    MotionFX_enable_6X(motionFX_ptr, MFX_ENGINE_ENABLE);
    MotionFX_enable_9X(motionFX_ptr, MFX_ENGINE_DISABLE);
    MotionFX_getKnobs(motionFX_ptr, &iKnobs); // 这里可以加一个检查是否初始化成功的逻辑，输出日志
    // 设置MotionFX参数
    iKnobs.LMode = 1;
    iKnobs.modx = 5;
    iKnobs.start_automatic_gbias_calculation = 0;
    iKnobs.ATime = 3.6f;
    iKnobs.FrTime = 2.8f;
    iKnobs.acc_orientation[0] = 's';
    iKnobs.acc_orientation[1] = 'e';
    iKnobs.acc_orientation[2] = 'u';
    iKnobs.gyro_orientation[0] = 's';
    iKnobs.gyro_orientation[1] = 'e';
    iKnobs.gyro_orientation[2] = 'u';

    iKnobs.gbias_acc_th_sc = (2.0f * 0.0765f);
    iKnobs.gbias_gyro_th_sc = (2.0f * 0.002f);

    iKnobs.output_type = MFX_ENGINE_OUTPUT_ENU;

    MotionFX_setKnobs(motionFX_ptr, &iKnobs);
    MotionFX_setGbias(motionFX_ptr, gbias);

    AHRS_Time = DWT_GetTimeline_s();

    Monitor::Instance()->Log_Messages(Monitor::INFO, (uint8_t *)("AHRS Init Success!\r\n"));
}

void AHRS::AHRS_Update()
{
    if (BMI088::Instance()->bmi088_selfTest.BMI088_INIT_ERR == true)
    {
        AHRS_Time = DWT_GetTimeline_s();
        return;
    }

    dt = DWT_GetTimeline_s() - AHRS_Time; // 获取时间间隔
    AHRS_Time += dt;

    // data_in.acc[0] = BMI088::Instance()->bmi088_data.acc_data.y * 0.1020408f;
    // data_in.acc[1] = BMI088::Instance()->bmi088_data.acc_data.x * 0.1020408f;
    // data_in.acc[2] = BMI088::Instance()->bmi088_data.acc_data.z * 0.1020408f;

    data_in.acc[0] = BMI088::Instance()->bmi088_data.acc_data.x * InvG;
    data_in.acc[1] = BMI088::Instance()->bmi088_data.acc_data.y * InvG;
    data_in.acc[2] = BMI088::Instance()->bmi088_data.acc_data.z * InvG;

    data_in.gyro[0] = BMI088::Instance()->bmi088_data.gyro_data.roll * 57.29f;
    data_in.gyro[1] = BMI088::Instance()->bmi088_data.gyro_data.pitch * 57.29f;
    data_in.gyro[2] = BMI088::Instance()->bmi088_data.gyro_data.yaw * 57.29f;

    MotionFX_propagate(motionFX_ptr, &data_out, &data_in, &dt);
    MotionFX_update(motionFX_ptr, &data_out, &data_in, &dt, NULL);
    MotionFX_getGbias(motionFX_ptr, gbias);

    INS.q[0] = data_out.quaternion[0];
    INS.q[1] = data_out.quaternion[1];
    INS.q[2] = data_out.quaternion[2];
    INS.q[3] = data_out.quaternion[3];

    INS.Yaw = (data_out.rotation[0] > 180) ? (data_out.rotation[0] - 360) : data_out.rotation[0]; // 偏航角，范围归化到-180~180

    INS.Pitch = data_out.rotation[2]; // 俯仰角，范围-180~180
    INS.Roll = data_out.rotation[1];  // 横滚角，范围-90~90

    if (INS.Yaw - INS.LastYaw > 180)
    {
        INS.YawRoundCount--;
    }
    else if (INS.Yaw - INS.LastYaw < -180)
    {
        INS.YawRoundCount++;
    }

    INS.LastYaw = INS.Yaw; // 记录上一次的偏航角

    INS.YawTotalAngle = 360 * INS.YawRoundCount + INS.Yaw; // 计算总偏航角，便于控制
}

#elif defined(IMU_LIB_EKF_)
#include "AHRS-EKF.hpp"

float debug_Yaw = 0.0f;
float debug_Pitch = 0.0f;
float debug_Roll = 0.0f;

AHRS::attitude_t *AHRS::INS_Init(void)
{
    if (!INS.init)
        INS.init = 1;
    else
        return (attitude_t *)&INS.Gyro;

    IMU_Param.scale[X] = 1;
    IMU_Param.scale[Y] = 1;
    IMU_Param.scale[Z] = 1;
    IMU_Param.Yaw = 0;
    IMU_Param.Pitch = 0;
    IMU_Param.Roll = 0;
    IMU_Param.flag = 1;

    float init_quaternion[4] = {0};
    InitQuaternion(init_quaternion);
    IMU_QuaternionEKF_Init(init_quaternion, 10, 0.001, 1000000, 1, 0);
    // noise of accel is relatively big and of high freq,thus lpf is used
    INS.AccelLPF = 0.0085;
    DWT_GetDeltaT(&INS_DWT_Count);

    Monitor::Instance()->Log_Messages(Monitor::INFO, (uint8_t *)("AHRS Init Success!\r\n"));

    return (attitude_t *)&INS.Gyro; // @todo: 这里偷懒了,不要这样做! 修改INT_t结构体可能会导致异常,待修复.
}

void AHRS::AHRS_Update()
{
    static uint32_t count = 0;
    const float gravity[3] = {0, 0, 9.81f};

    dt = DWT_GetDeltaT(&INS_DWT_Count);
    t += dt;

    // ins update
    if ((count % 1) == 0 && BMI088::Instance()->bmi088_selfTest.BMI088_INIT_ERR == false)
    {

        INS.Accel[X] = BMI088::Instance()->bmi088_data.acc_data.x;
        INS.Accel[Y] = BMI088::Instance()->bmi088_data.acc_data.y;
        INS.Accel[Z] = BMI088::Instance()->bmi088_data.acc_data.z;
        INS.Gyro[X] = BMI088::Instance()->bmi088_data.gyro_data.roll;
        INS.Gyro[Y] = BMI088::Instance()->bmi088_data.gyro_data.pitch;
        INS.Gyro[Z] = BMI088::Instance()->bmi088_data.gyro_data.yaw;

        // 核心函数,EKF更新四元数
        IMU_QuaternionEKF_Update(INS.Gyro[X], INS.Gyro[Y], INS.Gyro[Z], INS.Accel[X], INS.Accel[Y], INS.Accel[Z], dt);

        memcpy(INS.q, QEKF_INS.q, sizeof(QEKF_INS.q));

        // 机体系基向量转换到导航坐标系，本例选取惯性系为导航系
        BodyFrameToEarthFrame(xb, INS.xn, INS.q);
        BodyFrameToEarthFrame(yb, INS.yn, INS.q);
        BodyFrameToEarthFrame(zb, INS.zn, INS.q);

        // 将重力从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
        float gravity_b[3];
        EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
        for (uint8_t i = 0; i < 3; ++i) // 同样过一个低通滤波
        {
            INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * dt / (INS.AccelLPF + dt) + INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);
        }
        BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q); // 转换回导航系n

        INS.Yaw = QEKF_INS.Yaw;
        INS.Pitch = QEKF_INS.Pitch;
        INS.Roll = QEKF_INS.Roll;
        INS.YawTotalAngle = QEKF_INS.YawTotalAngle;

        debug_Yaw = INS.Yaw;
        debug_Pitch = INS.Pitch;
        debug_Roll = INS.Roll;
    }
}

void AHRS::QuaternionUpdate(float *q, float gx, float gy, float gz, float dt)
{
    float qa, qb, qc;

    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;
    qa = q[0];
    qb = q[1];
    qc = q[2];
    q[0] += (-qb * gx - qc * gy - q[3] * gz);
    q[1] += (qa * gx + qc * gz - q[3] * gy);
    q[2] += (qa * gy - qb * gz + q[3] * gx);
    q[3] += (qa * gz + qb * gy - qc * gx);
}

void AHRS::EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q)
{
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;
    Yaw /= 57.295779513f;
    Pitch /= 57.295779513f;
    Roll /= 57.295779513f;
    cosPitch = arm_cos_f32(Pitch / 2);
    cosYaw = arm_cos_f32(Yaw / 2);
    cosRoll = arm_cos_f32(Roll / 2);
    sinPitch = arm_sin_f32(Pitch / 2);
    sinYaw = arm_sin_f32(Yaw / 2);
    sinRoll = arm_sin_f32(Roll / 2);
    q[0] = cosPitch * cosRoll * cosYaw + sinPitch * sinRoll * sinYaw;
    q[1] = sinPitch * cosRoll * cosYaw - cosPitch * sinRoll * sinYaw;
    q[2] = sinPitch * cosRoll * sinYaw + cosPitch * sinRoll * cosYaw;
    q[3] = cosPitch * cosRoll * sinYaw - sinPitch * sinRoll * cosYaw;
}

void AHRS::EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

void AHRS::BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}
#elif defined(IMU_LIB_ICM42688_)
#include "AHRS-ICM.hpp"
void AHRS::INS_Init()
{
    if (IMU_MSG_ID < 0xC1 || IMU_MSG_ID > 0xC8)
    {
        Monitor::Instance()->Log_Messages(Monitor::ERROR, (uint8_t *)("IMU_MSG_ID Error!\r\n"));
        return;
    }
    IMU_Msg_1 = &BoardConnectivity::Instance()->BoardMemory_receive[IMU_MSG_ID - 0xC1];
    IMU_Msg_2 = &BoardConnectivity::Instance()->BoardMemory_receive[IMU_MSG_ID - 0xC1 + 1];
}

void AHRS::AHRS_Update()
{
    if (IMU_Msg_1->len == 0 || IMU_Msg_2->len == 0)
        return;

    memcpy(&INS.q, &IMU_Msg_1->data, 8);
    memcpy(&INS.q[2], &IMU_Msg_2->data, 8);

    // 四元数转欧拉角
    float Euler[3] = {0};
    Math::QuaternionToEularAngle(INS.q, Euler);
    INS.Yaw = Euler[0];
    INS.Pitch = Euler[1];
    INS.Roll = Euler[2];
}
#else
#error "No IMU library defined! Please define either IMU_LIB_EKF_ or IMU_LIB_MOTION_FX_."
#endif
