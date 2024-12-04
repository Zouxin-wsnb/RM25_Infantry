#include "ChassisController.hpp"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

float v_loss = 0.1;
float t2_loss = 10;
float relax_loss = 2.6;

/**
 * @brief 初始化函数，用于挂载四个底盘电机，并设置PID参数
 */
void ChassisController::Init()
{
    /*-----------------------初始化一些基本数值-----------------------*/
    Vx = 0;
    Vy = 0;
    Vw = 0;
    RelativeAngle = 0;

    RF_speedSet = 0;
    LF_speedSet = 0;
    RR_speedSet = 0;
    LR_speedSet = 0;

    /*----------------------------PID参数----------------------------*/
    GM3508SpeedPid.kp = 150.0f;
    GM3508SpeedPid.ki = 0.0f;
    GM3508SpeedPid.kd = 4.5f;
    GM3508SpeedPid.maxOut = 15000;
    GM3508SpeedPid.maxIOut = 3;

    /*----------------------------右前轮----------------------------*/
    R_Front.controlMode = GMMotor::SPD_MODE;
    R_Front.speedPid = GM3508SpeedPid;
    R_Front.positionSet = R_Front.motorFeedback.positionFdb;
    R_Front.positionPid.Clear();
    R_Front.speedPid.Clear();
    R_Front.setOutput();
    /*----------------------------左前轮----------------------------*/
    L_Front.controlMode = GMMotor::SPD_MODE;
    L_Front.speedPid = GM3508SpeedPid;
    L_Front.positionSet = L_Front.motorFeedback.positionFdb;
    L_Front.positionPid.Clear();
    L_Front.speedPid.Clear();
    L_Front.setOutput();
    /*----------------------------右后轮----------------------------*/
    R_Rear.controlMode = GMMotor::SPD_MODE;
    R_Rear.speedPid = GM3508SpeedPid;
    R_Rear.positionSet = R_Rear.motorFeedback.positionFdb;
    R_Rear.positionPid.Clear();
    R_Rear.speedPid.Clear();
    R_Rear.setOutput();
    /*----------------------------左后轮----------------------------*/
    L_Rear.controlMode = GMMotor::SPD_MODE;
    L_Rear.speedPid = GM3508SpeedPid;
    L_Rear.positionSet = L_Rear.motorFeedback.positionFdb;
    L_Rear.positionPid.Clear();
    L_Rear.speedPid.Clear();
    L_Rear.setOutput();

    /*---------------------------挂载电机---------------------------*/
    GMMotorHandler::instance()->registerMotor(&R_Front, &hcan1, 0x204);
    GMMotorHandler::instance()->registerMotor(&L_Front, &hcan1, 0x201);
    GMMotorHandler::instance()->registerMotor(&L_Rear, &hcan1, 0x202);
    GMMotorHandler::instance()->registerMotor(&R_Rear, &hcan1, 0x203);

    SCSetUnion.capSetData.type = BOARD_CONNECTIVITY_CAN_2;
    SCSetUnion.capSetData.id = 0xC4; // 发送的ID
    SCSetUnion.capSetData.len = 8;
    SCSetUnion.capSetData.cap_state_set = 0;
    SCSetUnion.capSetData.fly_extra_set = 0;
    SCSetUnion.capSetData.power_limit_set = 45;
    SCSetUnion.capSetData.reserve = 0;

    SCFdbUnion.capFdbData.type = BOARD_CONNECTIVITY_CAN_2;
    SCFdbUnion.capFdbData.id = 0xB4; // 接收的ID
    SCFdbUnion.capFdbData.len = 8;
    SCFdbUnion.capFdbData.cap_state_fdb = 0;
    SCFdbUnion.capFdbData.cap_voltage_x5 = 0;
    SCFdbUnion.capFdbData.input_power_x100 = 0;
    SCFdbUnion.capFdbData.cap_power_x100 = 0;
    SCFdbUnion.capFdbData.cap_inpower_x100 = 0;

    BoardConnectivity::Instance()->Add2Memory(SCSetUnion.msg);
    BoardConnectivity::Instance()->Add2Memory(SCFdbUnion.msg);

    // VxFilter.Init();
    // VxFilter.SetTau(0.25);
    // VxFilter.SetUpdatePeriod(1);

    // VyFilter.Init();
    // VyFilter.SetTau(0.25);
    // VyFilter.SetUpdatePeriod(1);

    // VwFilter.Init();
    // VwFilter.SetTau(0.1);
    // VwFilter.SetUpdatePeriod(1);

    VxFilter.Clear();
    VyFilter.Clear();
    VwFilter.Clear();

    VxFilter.SetQ(0.001f);
    VxFilter.SetR(0.6f);
    VyFilter.SetQ(0.001f);
    VyFilter.SetR(0.6f);
    VwFilter.SetQ(0.001f);
    VwFilter.SetR(0.6f);

    Monitor::Instance()->Log_Messages(Monitor::INFO, (uint8_t *)"ChassisController Init Success!\r\n");
}

// 这里是循环的主要逻辑，用于控制底盘的运动。

void ChassisController::Run()
{

    uint8_t xyData[8];                                                     // 用于存储接收到的xy数据
    uint8_t wAndAngleData[8];                                              // 用于存储接收到的w数据
    BoardConnectivity::Instance()->GetBoardMemory(0xB1, xyData, 8);        // 获取板间通信的数据
    BoardConnectivity::Instance()->GetBoardMemory(0xB2, wAndAngleData, 8); // 获取板间通信的数据

    memcpy(&Vx, xyData, 4);        // 将接收到的数据拷贝到Vx
    memcpy(&Vy, xyData + 4, 4);    // 将接收到的数据拷贝到Vy
    memcpy(&Vw, wAndAngleData, 4); // 将接收到的数据拷贝到Vw
    memcpy(&RelativeAngle, wAndAngleData + 4, 4);

    if (isnan(Vx) || isnan(Vy) || isnan(Vw) || isnan(RelativeAngle)) // 如果出现nan错误，将速度设定值设为0
    {
        Vx = 0;
        Vy = 0;
        Vw = 0;
        RelativeAngle = 0;
    }

    // 一阶滤波
    // VxFilter.SetInput(Vx); // 滤波
    // VxFilter.Update();
    // Vx = VxFilter.GetResult();

    // VyFilter.SetInput(Vy); // 滤波
    // VyFilter.Update();
    // Vy = VyFilter.GetResult();

    // VwFilter.SetInput(Vw); // 滤波
    // VwFilter.Update();
    // Vw = VwFilter.GetResult();

    // 一阶卡尔曼滤波
    Vx = VxFilter.Update(Vx);
    Vy = VyFilter.Update(Vy);
    Vw = VwFilter.Update(Vw);

    float SinAngle = arm_sin_f32(RelativeAngle);
    float CosAngle = arm_cos_f32(RelativeAngle);

    RF_speedSet = +(Vy * CosAngle - Vx * SinAngle) - (Vy * SinAngle + Vx * CosAngle) - Vw * distanceToFrontAxle * distanceToRightSide; // RF 右前轮
    LF_speedSet = +(Vy * CosAngle - Vx * SinAngle) + (Vy * SinAngle + Vx * CosAngle) - Vw * distanceToFrontAxle * distanceToLeftSide;  // LF 左前轮
    LR_speedSet = -(Vy * CosAngle - Vx * SinAngle) + (Vy * SinAngle + Vx * CosAngle) - Vw * distanceToRearAxle * distanceToRightSide;  // LR 左后轮
    RR_speedSet = -(Vy * CosAngle - Vx * SinAngle) - (Vy * SinAngle + Vx * CosAngle) - Vw * distanceToRearAxle * distanceToLeftSide;   // RR 右后轮

    SCSetUnion.capSetData.cap_state_set = 1;
    SCSetUnion.capSetData.fly_extra_set = 0;
    SCSetUnion.capSetData.power_limit_set = 45;

    BoardConnectivity::Instance()->GetBoardMemory(0xB4, SCFdbUnion.msg.data, 8);

    // 将速度设定值传递电机，即设定电机的输出。
    // 3508电机的减速比是1:19，所以需要乘以19。
    R_Front.speedSet = RF_speedSet * 19 / wheelRadius;
    L_Front.speedSet = LF_speedSet * 19 / wheelRadius;
    L_Rear.speedSet = LR_speedSet * 19 / wheelRadius;
    R_Rear.speedSet = RR_speedSet * 19 / wheelRadius;

    R_Front.setOutput();
    L_Front.setOutput();
    R_Rear.setOutput();
    L_Rear.setOutput();

    BoardConnectivity::Instance()->Add2Memory(SCSetUnion.msg);
}

#ifdef PowerLimitNormal
void ChassisController::PowerLimit()
{
    
    double Power_Limit = Referee::Instance()->GameRobotStatus.chassis_power_limit * 1.0f; // 电机底盘功率限制

    //double effort_coeff=13.44; // 功率系数
    //double velocity_coeff=0.15; // 速度系数
    //double power_offset=18.72; // 功率偏移

    // double a=0., b=0., c=-power_offset-power_max; // 二次函数参数

    // float Kt = 0.3; // 电机转矩系数
    // float current_set = 0; // 电机电流设定值
    // // 力矩平方和
    // a+=square(R_Front.currentSet*Kt*20/15000);
    // a+=square(L_Front.currentSet*Kt*20/15000);
    // a+=square(R_Rear.currentSet*Kt*20/15000);
    // a+=square(L_Rear.currentSet*Kt*20/15000);
    // // 力矩乘速度乘积和
	// b+=Math::abs(R_Front.currentSet*Kt*20/15000*R_Front.speedSet);
    // b+=Math::abs(L_Front.currentSet*Kt*20/15000*L_Front.speedSet);
    // b+=Math::abs(R_Rear.currentSet*Kt*20/15000*R_Rear.speedSet);
    // b+=Math::abs(L_Rear.currentSet*Kt*20/15000*L_Rear.speedSet);
    // // 速度平方和
    // c+=square(R_Front.speedSet);
    // c+=square(L_Front.speedSet);
    // c+=square(R_Rear.speedSet);
    // c+=square(L_Rear.speedSet);

    // a*=effort_coeff;
    // c*=velocity_coeff;
    // //缩放系数
    // double zoom_coeff = (square(b) - 4 * a * c) > 0 ? ((-b + sqrt(square(b) - 4 * a * c)) / (2 * a)) : 0.;
    //---------------------------------------------------------------------------------------------------------
    float k1 = v_loss; // 电机转速产生的功率损耗系数
    float k2 = t2_loss; // 电机力矩平方产生的功率损耗系数
    float k3 = relax_loss; // 电机静态损耗

    // 1. 转换系数修正
    const float CURRENT_TO_TORQUE = 0.3f * (20.0f / 16384.0f); // Nm/A
    const float RPM_TO_RADS = 2.0f *Math::Pi / 60.0f; // 转速单位转换
    
    // 2. 计算每个电机功率
    auto calculateMotorPower = [&](GM3508& motor) 
    {
        double torque = motor.motorFeedback.currentFdb * CURRENT_TO_TORQUE;
        double speed = motor.motorFeedback.speedFdb ;
        
        // 计算机械功率和损耗
        double mech_power = Math::abs(torque * speed);
        double loss_power = k1 * Math::abs(speed) + k2 * torque * torque;
        return mech_power + loss_power;
    };
    
    // 3. 计算总功率
    double power_sum = calculateMotorPower(R_Front) +
                      calculateMotorPower(L_Front) +
                      calculateMotorPower(R_Rear) +
                      calculateMotorPower(L_Rear) + k3;
    // 4. 功率限制
    if(power_sum > Power_Limit)
    {
        // 限制系数，添加下限防止急停
        float limit_ratio;
        if(Power_Limit / power_sum > 0.1)
        {
            limit_ratio = Power_Limit / power_sum;
        }
        else
        {
            limit_ratio = 0.1;
        }
        
        // 等比例限制电流
        R_Front.currentSet *= limit_ratio;
        L_Front.currentSet *= limit_ratio;
        R_Rear.currentSet *= limit_ratio;
        L_Rear.currentSet *= limit_ratio;
    }
}
#else
#ifdef PowerLimitWithBuffer
void ChassisController::PowerLimitWithPowerBuffer()
{
    int PowerBuffer = Referee::Instance()->PowerHeatData.chassis_power_buffer;
    double Power_Limit = Referee::Instance()->GameRobotStatus.chassis_power_limit * 1.0f; // 电机底盘功率限制
    double Power_Sum = Power_Limit;
    if (InitialPowerBuffer - PowerBuffer > 0)
    {
        Power_Sum = (InitialPowerBuffer - PowerBuffer)*10.0f + Power_Limit;
        float limit_ratio;
        if(Power_Limit / Power_Sum > 0.1)
        {
            limit_ratio = Power_Limit / Power_Sum;
        }
        else
        {
            limit_ratio = 0.1;
        }
        
        // 等比例限制电流
        R_Front.currentSet *= limit_ratio;
        L_Front.currentSet *= limit_ratio;
        R_Rear.currentSet *= limit_ratio;
        L_Rear.currentSet *= limit_ratio;
    }
}
#else
#ifdef PowerLimitWithSuperCap
void ChassisController::PowerLimitWithSuperCap()
{
}
#endif
#endif
#endif

void ChassisController::HandleInput()
{
}
