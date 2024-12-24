#include "ChassisController.hpp"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

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

    // 限制底盘功率
    PowerLimit(motors);

    R_Front.setOutput();
    L_Front.setOutput();
    R_Rear.setOutput();
    L_Rear.setOutput();

    BoardConnectivity::Instance()->Add2Memory(SCSetUnion.msg);
}

#ifdef PowerLimitNormal
void ChassisController::PowerLimit(GM3508* motors[])
{
    //最大功率
	uint16_t max_power_limit =45;
    max_power_limit = Referee::Instance()->GameRobotStatus.chassis_power_limit;

	float chassis_max_power = 0;
	float input_power = 0;		 // input power from battery (referee system)
	float initial_give_power[4]; // initial power from PID calculation
	float initial_total_power = 0;
	float scaled_give_power[4];

	// float chassis_power = 0.0f;
	float chassis_power_buffer = 0.0f;

    //限制系数
	float toque_coefficient = 1.99688994e-6f; // (20/16384)*(0.3)*(187/3591)/9.55
	float a = 1.23e-07;						 // k1
	float k2 = 1.453e-07;					 // k2
	float constant = 4.081f;

	// get_chassis_power_and_buffer(&chassis_power, &chassis_power_buffer);
	// PID_calc(&chassis_power_control->buffer_pid, chassis_power_buffer, 30);
	// get_chassis_max_power(&max_power_limit);
	input_power = max_power_limit;// - chassis_power_control->buffer_pid.out; // Input power floating at maximum power

    //超级电容部分尚未实现
	// CAN_CMD_CAP(input_power); // set the input power of capacitor controller

	// if (rc_ctrl.key.v & KEY_PRESSED_OFFSET_E)
	// {
	// 	cap_state = 0;
	// }
	// if (rc_ctrl.key.v & KEY_PRESSED_OFFSET_Q)
	// {
	// 	cap_state = 1;
	// }

	// if (cap_measure.cap_percent > 5)
	// {
	// 	if (cap_state == 0)
	// 	{
	 		chassis_max_power = input_power;//+5 // Slightly greater than the maximum power, avoiding the capacitor being full all the time and improving energy utilization
	// 	}
	// 	else
	// 	{
	// 		chassis_max_power = input_power + 200;
	// 	}
	// }
	// else
	// {
	// 	chassis_max_power = input_power;
	// }

    //计算初始功率
	for (uint8_t i = 0; i < 4; i++) // first get all the initial motor power and total motor power
	{
		initial_give_power[i] = motors[i]->speedPid.result * toque_coefficient * motors[i]->motorFeedback.speed_rpm +
								k2 * motors[i]->motorFeedback.speed_rpm * motors[i]->motorFeedback.speed_rpm +
								a * motors[i]->speedPid.result * motors[i]->speedPid.result + constant;

		if (initial_give_power[i] < 0.0f) // negative power not included (transitory)
			continue;
		initial_total_power += initial_give_power[i];
	}

    //如果初始功率大于最大功率，进行功率限制
	if (initial_total_power > chassis_max_power) // determine if larger than max power
	{
		float power_scale = chassis_max_power / initial_total_power;
        //对每个电机单独进行功率限制
		for (uint8_t i = 0; i < 4; i++)
		{
			scaled_give_power[i] = initial_give_power[i] * power_scale; // get scaled power
			if (scaled_give_power[i] < 0)
			{
				continue;
			}

	        float b = toque_coefficient * motors[i]->motorFeedback.speed_rpm;
			float c = k2 * motors[i]->motorFeedback.speed_rpm * motors[i]->motorFeedback.speed_rpm - scaled_give_power[i] + constant;

			if (motors[i]->speedPid.result > 0) // Selection of the calculation formula according to the direction of the original moment
			{
				float temp = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
				if (temp > 16000)
				{
					motors[i]->currentSet = 16000;
				}
				else
					motors[i]->currentSet = temp;
			}
			else
			{
				float temp = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
				if (temp < -16000)
				{
					motors[i]->currentSet = -16000;
				}
				else
					motors[i]->currentSet = temp;
			}
		}
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
#endif
#endif

void ChassisController::HandleInput()
{
}
