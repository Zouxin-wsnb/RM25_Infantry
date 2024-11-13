/**
 * @file BMI088.cpp
 * @brief BMI088类实现文件
 * @details BMI088类实现文件，用于实现BMI088的初始化、数据读取、自检等功能
 * @version 0.1
 * @date 2021-9-1
 * @note 注意BMI088的配置部分和姿态解算库有关系，请在使用时注意配置
 * 以后可以考虑通过宏定义的方式将配置和姿态解算库解耦
 */

#include "BMI088.hpp"
#include "BMI088reg.h"

BMI088::BMI088() {}

BMI088::~BMI088() {}

void BMI088::BMI088_INIT()
{
    bmi088_selfTest.ACC_CHIP_ID_ERR = true;       // 加速度计ID错误则为true
    bmi088_selfTest.ACC_DATA_ERR = true;          // 加速度计数据错误则为true
    bmi088_selfTest.GYRO_CHIP_ID_ERR = true;      // 陀螺仪ID错误则为true
    bmi088_selfTest.GYRO_DATA_ERR = true;         // 陀螺仪数据错误则为true
    bmi088_selfTest.BMI088_INIT_ERR = true;       // BMI088初始化错误则为true
    bmi088_selfTest.BMI088_CALIBRATE_ERR = false; // BMI088标定错误则为true
    bmi088_selfTest.BMI088_TEMP_CTRL_ERR = false; // BMI088温度控制错误则为true

#if defined(IMU_LIB_EKF_) || defined(IMU_LIB_MOTION_FX_)

    Acc_coef = BMI088_ACCEL_3G_SEN; // 标定完后要乘以9.805/gNorm，注意这里需要和配置的范围对应

    BMI088_CONF_INIT(); //< 初始化配置

    VerifyAccChipID();  //< 验证加速度计ID
    VerifyGyroChipID(); //< 验证陀螺仪ID

    TemperaturePid.mode = PID_POSITION | PID_Integral_Limit | PID_Changing_Integral_Rate | PID_Derivative_On_Measurement; // 位置式PID，积分限幅
    TemperaturePid.kp = 650.0f;
    TemperaturePid.ki = 0.06f;
    TemperaturePid.kd = 0.1f;
    TemperaturePid.maxOut = 900.0f;
    TemperaturePid.maxIOut = 300.0f;
    TemperaturePid.ScalarA = 3.5f;
    TemperaturePid.ScalarB = 0.08f;

    TemperatureFdbFilter.SetTau(0.1f);       // 设置滤波时间常数
    TemperatureFdbFilter.SetUpdatePeriod(1); // 设置更新周期

    SetTargetTemp(50.0f);                              //< 设置目标温度，一般为40度以上
    PWM_Start(&HEATING_RESISTANCE_TIM, TIM_CHANNEL_1); //< 启动加热电阻PWM

    float startTime; // 开始升温时间,用于确定是否超时
    startTime = DWT_GetTimeline_s();
    while (fabs(bmi088_data.acc_data.temperature - TargetTemp) > 0.1f)
    {
        if (DWT_GetTimeline_s() - startTime > 10.00) // 超时则直接进入下一步
        {
            bmi088_selfTest.BMI088_TEMP_CTRL_ERR = true;
            startTime = DWT_GetTimeline_s();
            break;
        }
        ReadAccTemperature(&bmi088_data.acc_data.temperature);
        TemperatureControl(TargetTemp);
    }
    startTime = DWT_GetTimeline_s();
    while (DWT_GetTimeline_s() - startTime < 8.01)
    {
        ReadAccTemperature(&bmi088_data.acc_data.temperature);
        TemperatureControl(TargetTemp);
    }
    CalibrateIMU(); //< 标定IMU

#ifdef IMU_LIB_MOTION_FX_
    Acc_coef = BMI088_ACCEL_3G_SEN; // 如果使用了MFX库，这里需要重新设置一下
#endif

    bmi088_selfTest.BMI088_INIT_ERR = false; //< 初始化成功,错误标志设置为false

    Monitor::Instance()->Log_Messages(Monitor::INFO, (uint8_t *)("BMI088 Init Success!\r\n"));

#endif
}

/* pre calibrate parameter to go here */
#define BMI088_PRE_CALI_ACC_X_OFFSET 0.0f
#define BMI088_PRE_CALI_ACC_Y_OFFSET 0.0f
#define BMI088_PRE_CALI_ACC_Z_OFFSET 0.0f
#define BMI088_PRE_CALI_G_NORM 9.805f
/**
 * @brief BMI088 acc gyro 标定
 * @note 标定后的数据存储在bmi088->bias和gNorm中,用于后续数据消噪和单位转换归一化
 * @attention 不管工作模式是blocking还是IT,标定时都是blocking模式,所以不用担心中断关闭后无法标定(RobotInit关闭了全局中断)
 * @attention 标定精度和等待时间有关,目前使用线性回归.后续考虑引入非线性回归
 * @todo 将标定次数(等待时间)变为参数供设定
 * @section 整体流程为1.累加加速度数据计算gNrom()
 *                   2.累加陀螺仪数据计算零飘
 *                   3. 如果标定过程运动幅度过大,重新标定
 *                   4.保存标定参数
 */
void BMI088::CalibrateIMU()
{

    Acc_coef = BMI088_ACCEL_3G_SEN; // 标定完后要乘以9.805/gNorm，注意这里需要和配置的范围对应

    // 一次性参数用完就丢,不用static
    float startTime;                     // 开始标定时间,用于确定是否超时
    uint16_t CaliTimes = 6000;           // 标定次数(6s)
    float gyroMax[3], gyroMin[3];        // 保存标定过程中读取到的数据最大值判断是否满足标定环境
    float gNormTemp, gNormMax, gNormMin; // 同上,计算矢量范数(模长)
    float gyroDiff[3], gNormDiff;        // 每个轴的最大角速度跨度及其模长

    startTime = DWT_GetTimeline_s();
    // 循环继续的条件为标定环境不满足
    do // 用do while至少执行一次,省得对上面的参数进行初始化
    {  // 标定超时,直接使用预标定参数(如果有)
        if (DWT_GetTimeline_s() - startTime > 12.01)
        { // 两次都没有成功就切换标定模式,丢给下一个if处理,使用预标定参数
            bmi088_selfTest.BMI088_CALIBRATE_ERR = true;
            break;
        }

        DWT_Delay(0.0005);
        gNorm = 0;
        for (uint8_t i = 0; i < 3; i++) // 重置gNorm和零飘
            Gyro_offset[i] = 0;

        // @todo : 这里也有获取bmi088数据的操作,后续与BMI088Acquire合并.注意标定时的工作模式是阻塞,且offset和acc_coef要初始化成0和1,标定完成后再设定为标定值
        for (uint16_t i = 0; i < CaliTimes; ++i) // 提前计算,优化
        {
            ReadAccData(&bmi088_data.acc_data);
            ReadGyroData(&bmi088_data.gyro_data);
            ReadAccTemperature(&bmi088_data.acc_data.temperature);
            TemperatureControl(TargetTemp);

					gNormTemp = Math::Sqrt(bmi088_data.acc_data.x * bmi088_data.acc_data.x +
                             bmi088_data.acc_data.y * bmi088_data.acc_data.y +
                             bmi088_data.acc_data.z * bmi088_data.acc_data.z); // 计算加速度范数

            gNorm += gNormTemp; // 计算范数并累加,最后除以calib times获取单次值

            Gyro_offset[0] += bmi088_data.gyro_data.roll; // 因为标定时传感器静止,所以采集到的值就是漂移,累加当前值,最后除以calib times获得零飘
            Gyro_offset[1] += bmi088_data.gyro_data.pitch;
            Gyro_offset[2] += bmi088_data.gyro_data.yaw;

            if (i == 0) // 避免未定义的行为(else中)
            {
                // 初始化成当前的重力加速度模长
                gNormMax = gNormMin = gNormTemp;

                // 初始化成当前的陀螺仪数据
                gyroMax[0] = bmi088_data.gyro_data.roll;
                gyroMax[1] = bmi088_data.gyro_data.pitch;
                gyroMax[2] = bmi088_data.gyro_data.yaw;

                gyroMin[0] = bmi088_data.gyro_data.roll;
                gyroMin[1] = bmi088_data.gyro_data.pitch;
                gyroMin[2] = bmi088_data.gyro_data.yaw;
            }
            else // 更新gNorm的Min Max和gyro的minmax
            {
                gNormMax = gNormMax > gNormTemp ? gNormMax : gNormTemp;
                gNormMin = gNormMin < gNormTemp ? gNormMin : gNormTemp;

                gyroMax[0] = gyroMax[0] > bmi088_data.gyro_data.roll ? gyroMax[0] : bmi088_data.gyro_data.roll;
                gyroMin[0] = gyroMin[0] < bmi088_data.gyro_data.roll ? gyroMin[0] : bmi088_data.gyro_data.roll;

                gyroMax[1] = gyroMax[1] > bmi088_data.gyro_data.pitch ? gyroMax[1] : bmi088_data.gyro_data.pitch;
                gyroMin[1] = gyroMin[1] < bmi088_data.gyro_data.pitch ? gyroMin[1] : bmi088_data.gyro_data.pitch;

                gyroMax[2] = gyroMax[2] > bmi088_data.gyro_data.yaw ? gyroMax[2] : bmi088_data.gyro_data.yaw;
                gyroMin[2] = gyroMin[2] < bmi088_data.gyro_data.yaw ? gyroMin[2] : bmi088_data.gyro_data.yaw;
            }

            gNormDiff = gNormMax - gNormMin; // 最大值和最小值的差
            for (uint8_t j = 0; j < 3; ++j)
                gyroDiff[j] = gyroMax[j] - gyroMin[j]; // 分别计算三轴
            if (gNormDiff > 0.5f ||
                gyroDiff[0] > 0.15f ||
                gyroDiff[1] > 0.15f ||
                gyroDiff[2] > 0.15f)
                break;         // 超出范围了,重开! remake到while循环,外面还有一层
            DWT_Delay(0.0005); // 休息一会再开始下一轮数据获取,IMU准备数据需要时间
        }
        gNorm /= (float)CaliTimes; // 加速度范数重力
        for (uint8_t i = 0; i < 3; ++i)
            Gyro_offset[i] /= (float)CaliTimes; // 三轴零飘
        // 这里直接存到temperature,可以另外增加BMI088Instance的成员变量TempWhenCalib
        // temperature = raw_data.temperature * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET; // 保存标定时的温度,如果已知温度和零飘的关系
    } while (gNormDiff > 0.5f ||
             fabsf(gNorm - 9.8f) > 0.5f ||
             gyroDiff[0] > 0.15f ||
             gyroDiff[1] > 0.15f ||
             gyroDiff[2] > 0.15f ||
             fabsf(Gyro_offset[0]) > 0.01f ||
             fabsf(Gyro_offset[1]) > 0.01f ||
             fabsf(Gyro_offset[2]) > 0.01f); // 满足条件说明标定环境不好

    if (bmi088_selfTest.BMI088_CALIBRATE_ERR == true) // 如果标定失败，使用预标定参数
    {
        Gyro_offset[0] = BMI088_PRE_CALI_ACC_X_OFFSET;
        Gyro_offset[1] = BMI088_PRE_CALI_ACC_Y_OFFSET;
        Gyro_offset[2] = BMI088_PRE_CALI_ACC_Z_OFFSET;
        gNorm = BMI088_PRE_CALI_G_NORM;
    }
    Acc_coef *= 9.805 / gNorm;
}

void BMI088::TemperatureControl(float target_temp)
{
//    debug_T_ref = target_temp;
//    debug_T_fdb = bmi088_data.acc_data.temperature;

    TemperatureFdbFilter.SetInput(bmi088_data.acc_data.temperature);
    TemperatureFdbFilter.Update();

    TemperaturePid.ref = target_temp;
    TemperaturePid.fdb = TemperatureFdbFilter.GetResult();
    TemperaturePid.UpdateResult();

    if (TemperaturePid.result < 0)
    {
        PWM_SetDutyRatio(&HEATING_RESISTANCE_TIM, 0, TIM_CHANNEL_1);
    }
    else
    {
        PWM_SetDutyRatio(&HEATING_RESISTANCE_TIM, TemperaturePid.result / 999, TIM_CHANNEL_1);
    }
}

void BMI088::SetTargetTemp(float temp)
{
    // 限制温度范围，防止过热或者无效，不过目前这些值是随便给的，后续要根据实际情况调整
    if (temp > 65.0f)
    {
        temp = 55.0f;
    }
    else if (temp < 40.0f)
    {
        temp = 40.0f;
    }
    TargetTemp = temp;
}

void BMI088::VerifyAccChipID()
{
    uint8_t pRxData[2]; //< 读取两个字节,第一个字节是dummy data,第二个字节是chip id

    ReadDataFromReg(BMI088_CS_ACC, ACC_CHIP_ID_ADDR, pRxData, 2); //< 读取加速度计chip id
    //< 如果chip id不等于预设值,则加速度计ID错误,初始化错误
    if (pRxData[1] != ACC_CHIP_ID_VAL)
    {
        bmi088_selfTest.ACC_CHIP_ID_ERR = true;
        bmi088_selfTest.BMI088_INIT_ERR = true;
    }
    else if (pRxData[1] == ACC_CHIP_ID_VAL)
    {
        bmi088_selfTest.ACC_CHIP_ID_ERR = false;
    }
}

void BMI088::VerifyGyroChipID()
{
    uint8_t pRxData;                                                 //< 读取一个字节,chip id
    ReadDataFromReg(BMI088_CS_GYRO, GYRO_CHIP_ID_ADDR, &pRxData, 1); //< 读取陀螺仪chip id
    //< 如果chip id不等于预设值,则陀螺仪ID错误,初始化错误
    if (pRxData != GYRO_CHIP_ID_VAL)
    {
        bmi088_selfTest.GYRO_CHIP_ID_ERR = true;
        bmi088_selfTest.BMI088_INIT_ERR = true;
    }
    else if (pRxData == GYRO_CHIP_ID_VAL)
    {
        bmi088_selfTest.GYRO_CHIP_ID_ERR = false;
    }
}

void BMI088::VerifyAccData() {}

void BMI088::VerifyGyroData() {}

#ifdef IMU_LIB_EKF_
void BMI088::BMI088_CONF_INIT()
{
    //< 加速度计初始化
    //< 先软重启，清空所有寄存器
    uint8_t pTxData;
    pTxData = ACC_SOFTRESET_VAL;
    WriteDataToReg(BMI088_CS_ACC, ACC_SOFTRESET_ADDR, &pTxData, 1);
    DWT_Delay(0.150); //< 延时50ms,重启需要时间

    //< 打开加速度计电源
    pTxData = ACC_PWR_CTRL_ON;
    WriteDataToReg(BMI088_CS_ACC, ACC_PWR_CTRL_ADDR, &pTxData, 1);
    DWT_Delay(0.050); //< 延时50ms,重启需要时间

    //< 加速度计变成正常模式
    pTxData = ACC_PWR_CONF_ACT;
    WriteDataToReg(BMI088_CS_ACC, ACC_PWR_CONF_ADDR, &pTxData, 1);
    DWT_Delay(0.050); //< 延时50ms,重启需要时间

    pTxData = ((0x2 << 0x4) | (0xB << 0x0) | 0x80);
    WriteDataToReg(BMI088_CS_ACC, ACC_CONF_ADDR, &pTxData, 1);
    DWT_Delay(0.050); //< 延时50ms

    //< 测量范围
    pTxData = ACC_RANGE_3G;
    WriteDataToReg(BMI088_CS_ACC, ACC_RANGE_ADDR, &pTxData, 1);
    DWT_Delay(0.050); //< 延时50ms

    pTxData = ((0x1 << 0x3) | (0x0 << 0x2) | (0x0 << 0x1));
    WriteDataToReg(BMI088_CS_ACC, INT1_IO_CTRL_ADDR, &pTxData, 1);
    DWT_Delay(0.050); //< 延时50ms

    pTxData = ((0x1 << 0x2));
    WriteDataToReg(BMI088_CS_ACC, INT_MAP_DATA_ADDR, &pTxData, 1);
    DWT_Delay(0.050); //< 延时50ms

    /*-------------------------------------陀螺仪初始化-------------------------------------*/
    //< 先软重启，清空所有寄存器
    pTxData = GYRO_SOFTRESET_VAL;
    WriteDataToReg(BMI088_CS_GYRO, GYRO_SOFTRESET_ADDR, &pTxData, 1);
    DWT_Delay(0.150); //< 延时50ms,重启需要时间

    pTxData = GYRO_RANGE_2000_DEG_S;
    WriteDataToReg(BMI088_CS_GYRO, GYRO_RANGE_ADDR, &pTxData, 1);
    DWT_Delay(0.050); //< 延时50ms,重启需要时间

    pTxData = GYRO_ODR_1000Hz_BANDWIDTH_116Hz | GYRO_LPM1_SUS;
    WriteDataToReg(BMI088_CS_GYRO, GYRO_BANDWIDTH_ADDR, &pTxData, 1);
    DWT_Delay(0.050); //< 延时50ms

    pTxData = GYRO_LPM1_NOR;
    WriteDataToReg(BMI088_CS_GYRO, GYRO_LPM1_ADDR, &pTxData, 1);
    DWT_Delay(0.050); //< 延时50ms

    pTxData = 0x80;
    WriteDataToReg(BMI088_CS_GYRO, GYRO_INT_CTRL_ADDR, &pTxData, 1);
    DWT_Delay(0.050); //< 延时50ms

    pTxData = ((0x0 << 0x1) | (0x0 << 0x0));
    WriteDataToReg(BMI088_CS_GYRO, GYRO_INT3_INT4_IO_CONF_ADDR, &pTxData, 1);
    DWT_Delay(0.050); //< 延时50ms

    pTxData = 0x01;
    WriteDataToReg(BMI088_CS_GYRO, GYRO_INT3_INT4_IO_MAP_ADDR, &pTxData, 1);
    DWT_Delay(0.050); //< 延时50ms
}
#elif defined(IMU_LIB_MOTION_FX_)
void BMI088::BMI088_CONF_INIT() // 按照MotionFX配置
{
    //< 加速度计初始化
    //< 先软重启，清空所有寄存器
    uint8_t pTxData;
    pTxData = ACC_SOFTRESET_VAL;
    WriteDataToReg(BMI088_CS_ACC, ACC_SOFTRESET_ADDR, &pTxData, 1);
    DWT_Delay(0.150); //< 延时50ms,重启需要时间

    //< 打开加速度计电源
    pTxData = ACC_PWR_CTRL_ON;
    WriteDataToReg(BMI088_CS_ACC, ACC_PWR_CTRL_ADDR, &pTxData, 1);
    DWT_Delay(0.050); //< 延时50ms,重启需要时间

    //< 加速度计变成正常模式
    pTxData = ACC_PWR_CONF_ACT;
    WriteDataToReg(BMI088_CS_ACC, ACC_PWR_CONF_ADDR, &pTxData, 1);
    DWT_Delay(0.050); //< 延时50ms,重启需要时间

    //< 测量范围，+-3g的测量范围
    pTxData = ACC_RANGE_3G;
    WriteDataToReg(BMI088_CS_ACC, ACC_RANGE_ADDR, &pTxData, 1);
    DWT_Delay(0.050); //< 延时50ms

    //< 写入配置，正常带宽，1600hz输出频率
    pTxData = (ACC_CONF_RESERVED << 7) | (ACC_CONF_BWP_NORM << 6) | (ACC_CONF_ODR_1600_Hz);
    WriteDataToReg(BMI088_CS_ACC, ACC_CONF_ADDR, &pTxData, 1);
    DWT_Delay(0.050); //< 延时50ms

    /*-------------------------------------陀螺仪初始化-------------------------------------*/
    //< 先软重启，清空所有寄存器
    pTxData = GYRO_SOFTRESET_VAL;
    WriteDataToReg(BMI088_CS_GYRO, GYRO_SOFTRESET_ADDR, &pTxData, 1);
    DWT_Delay(0.150); //< 延时50ms,重启需要时间

    //< 写入范围，+-2000°/s的测量范围
    pTxData = GYRO_RANGE_2000_DEG_S;
    WriteDataToReg(BMI088_CS_GYRO, GYRO_RANGE_ADDR, &pTxData, 1);
    DWT_Delay(0.050); //< 延时50ms,重启需要时间

    //< 陀螺仪进入正常模式
    pTxData = GYRO_LPM1_NOR;
    WriteDataToReg(BMI088_CS_GYRO, GYRO_LPM1_ADDR, &pTxData, 1);
    DWT_Delay(0.050); //< 延时50ms

    //< 陀螺仪带宽和输出频率
    pTxData = GYRO_ODR_1000Hz_BANDWIDTH_116Hz | GYRO_LPM1_SUS;
    WriteDataToReg(BMI088_CS_GYRO, GYRO_BANDWIDTH_ADDR, &pTxData, 1);
    DWT_Delay(0.050); //< 延时50ms
}
#elif defined(IMU_LIB_ICM42688_)
void BMI088::BMI088_CONF_INIT() // 按照MotionFX配置
{
}
#else
#error "No IMU library defined! Please define either IMU_LIB_EKF_ or IMU_LIB_MOTION_FX_."
#endif

void BMI088::ReadAccData(acc_data_t *data)
{
    uint8_t buf[ACC_XYZ_LEN + 1];                                         //< 读取数据缓存
    int16_t acc[3];                                                       //< 加速度计数据暂存
    ReadDataFromReg(BMI088_CS_ACC, ACC_X_LSB_ADDR, buf, ACC_XYZ_LEN + 1); //< 读取加速度计数据
    //< 拼接和转换数据
    acc[0] = ((int16_t)buf[1 + 1] << 8) + (int16_t)buf[0 + 1];
    acc[1] = ((int16_t)buf[3 + 1] << 8) + (int16_t)buf[2 + 1];
    acc[2] = ((int16_t)buf[5 + 1] << 8) + (int16_t)buf[4 + 1];
    data->x = (float)acc[0] * Acc_coef;
    data->y = (float)acc[1] * Acc_coef;
    data->z = (float)acc[2] * Acc_coef;
}

void BMI088::ReadGyroData(gyro_data_t *data)
{
    uint8_t buf[GYRO_XYZ_LEN]; //, range; //< 读取数据缓存, range其实是写入的配置
    int16_t gyro[3];

    ReadDataFromReg(BMI088_CS_GYRO, GYRO_RATE_X_LSB_ADDR, buf, GYRO_XYZ_LEN);
    //< 拼接和转换数据
    gyro[0] = ((int16_t)buf[1] << 8) + (int16_t)buf[0];
    gyro[1] = ((int16_t)buf[3] << 8) + (int16_t)buf[2];
    gyro[2] = ((int16_t)buf[5] << 8) + (int16_t)buf[4];
    // 注意这里就不要又除法又乘法的了，直接乘以一个常数，根据配置，单位是16.384，所以直接乘以1/16.384 * DEG2SEC即可
    data->roll = (float)gyro[0] * BMI088_GYRO_2000_SEN;
    data->pitch = (float)gyro[1] * BMI088_GYRO_2000_SEN;
    data->yaw = (float)gyro[2] * BMI088_GYRO_2000_SEN;
}

void BMI088::ReadAccSensorTime(float *time) // 未完成
{
    //    uint8_t buf[SENSORTIME_LEN + 1];
    //    ReadDataFromReg(BMI088_CS_ACC, SENSORTIME_0_ADDR, buf, SENSORTIME_LEN + 1);
    //    *time = buf[0 + 1] * SENSORTIME_0_UNIT + buf[1 + 1] * SENSORTIME_1_UNIT + buf[2 + 1] * SENSORTIME_2_UNIT;
}

void BMI088::ReadAccTemperature(float *temp) // 未完成
{
    uint8_t buf[TEMP_LEN + 1];
    ReadDataFromReg(BMI088_CS_ACC, TEMP_MSB_ADDR, buf, TEMP_LEN + 1);
    uint16_t temp_uint11 = (buf[0 + 1] << 3) + (buf[1 + 1] >> 5);
    int16_t temp_int11;
    if (temp_uint11 > 1023)
    {
        temp_int11 = (int16_t)temp_uint11 - 2048;
    }
    else
    {
        temp_int11 = (int16_t)temp_uint11;
    }
    *temp = temp_int11 * TEMP_UNIT + TEMP_BIAS;
}

void BMI088::update()
{
    if (bmi088_selfTest.BMI088_INIT_ERR == false) // 如果初始化成功则更新数据
    {
        ReadAccData(&bmi088_data.acc_data);
        ReadGyroData(&bmi088_data.gyro_data);
        ReadAccTemperature(&bmi088_data.acc_data.temperature);
        TemperatureControl(TargetTemp);
    }
}

void BMI088::WriteDataToReg(enum BMI088_SENSOR cs, uint8_t addr, uint8_t *data, uint8_t len)
{
    //< 片选，考虑以后进行bsp_gpio封装
    if (cs == BMI088_CS_ACC)
        HAL_GPIO_WritePin(BMI088_ACC_GPIOx, BMI088_ACC_GPIOp, GPIO_PIN_RESET);
    else if (cs == BMI088_CS_GYRO)
        HAL_GPIO_WritePin(BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp, GPIO_PIN_RESET);

    uint8_t pTxData = (addr & BMI088_SPI_WRITE_CODE); //< 处理地址为写地址

    SPI_Transmit(&BMI088_SPI, &pTxData, 1, SPI_BLOCK_MODE); //< 发送地址
    SPI_Transmit(&BMI088_SPI, data, len, SPI_BLOCK_MODE);   //< 发送数据

    // 理论上，这里不需要延时，但是如果数据出现问题，请尝试增加延时
    if (bmi088_selfTest.BMI088_INIT_ERR == true)
    {
        DWT_Delay(0.001);
    }
    //< 取消片选
    if (cs == BMI088_CS_ACC)
        HAL_GPIO_WritePin(BMI088_ACC_GPIOx, BMI088_ACC_GPIOp, GPIO_PIN_SET);
    else if (cs == BMI088_CS_GYRO)
        HAL_GPIO_WritePin(BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp, GPIO_PIN_SET);
}

void BMI088::ReadDataFromReg(enum BMI088_SENSOR cs, uint8_t addr, uint8_t *data, uint8_t len)
{
    //< 片选，考虑以后进行bsp_gpio封装
    if (cs == BMI088_CS_ACC)
        HAL_GPIO_WritePin(BMI088_ACC_GPIOx, BMI088_ACC_GPIOp, GPIO_PIN_RESET);
    else if (cs == BMI088_CS_GYRO)
        HAL_GPIO_WritePin(BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp, GPIO_PIN_RESET);

    uint8_t pTxData = (addr | BMI088_SPI_READ_CODE); //< 处理地址为读地址

    SPI_Transmit(&BMI088_SPI, &pTxData, 1, SPI_BLOCK_MODE); //< 发送地址
    SPI_Receive(&BMI088_SPI, data, len, SPI_BLOCK_MODE);    //< 读取数据

    //< 取消片选
    if (cs == BMI088_CS_ACC)
        HAL_GPIO_WritePin(BMI088_ACC_GPIOx, BMI088_ACC_GPIOp, GPIO_PIN_SET);
    else if (cs == BMI088_CS_GYRO)
        HAL_GPIO_WritePin(BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp, GPIO_PIN_SET);
}
