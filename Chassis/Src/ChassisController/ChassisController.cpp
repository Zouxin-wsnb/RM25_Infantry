#include "ChassisController.hpp"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
float debug_fdb;
float debug_set;
float delta;
float kp = 150.0f;
float ki = 0.0f;
float kd = 4.5f;
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

    VxFilter.Init();
    VxFilter.SetTau(0.25);
    VxFilter.SetUpdatePeriod(1);

    VyFilter.Init();
    VyFilter.SetTau(0.25);
    VyFilter.SetUpdatePeriod(1);

    VwFilter.Init();
    VwFilter.SetTau(0.1);
    VwFilter.SetUpdatePeriod(1);

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

    VxFilter.SetInput(Vx); // 滤波
    VxFilter.Update();
    Vx = VxFilter.GetResult();

    VyFilter.SetInput(Vy); // 滤波
    VyFilter.Update();
    Vy = VyFilter.GetResult();

//    VwFilter.SetInput(Vw); // 滤波
//    VwFilter.Update();
//    Vw = VwFilter.GetResult();

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
		
		debug_set = R_Front.speedPid.ref;
		debug_fdb = R_Front.speedPid.fdb;
		delta = debug_set - debug_fdb;
		GM3508SpeedPid.kp = kp;
    GM3508SpeedPid.ki = ki;
    GM3508SpeedPid.kd = kd;
}

void ChassisController::HandleInput()
{
}
