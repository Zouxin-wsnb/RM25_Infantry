#include "LK9025.hpp"
#include "bsp_can.h"

LK9025::LK9025()
{
    // 初始化为松开模式
    controlMode = RELAX_MODE;

    // 初始化设定值为0
    speedSet = 0;
    positionSet = 0;
    currentSet = 0;
    maxCurrent = 25000; // 电机最大电压设定

    // 初始化电机反馈数据
    motorFeedback.speedFdb = 0;
    motorFeedback.lastSpeedFdb = 0;
    motorFeedback.positionFdb = 0;
    motorFeedback.lastPositionFdb = 0;
    motorFeedback.temperatureFdb = 0;

    // PID控制器初始化
    speedPid.mode = PID::PID_POSITION;
    speedPid.kp = 0.1;
    speedPid.ki = 0.0;
    speedPid.kd = 0.0;
    speedPid.maxOut = 25000;
    speedPid.maxIOut = 3;

    positionPid.mode = PID::PID_POSITION;
    positionPid.kp = 0.1;
    positionPid.ki = 0.0;
    positionPid.kd = 0.0;
    positionPid.maxOut = 25000;
    positionPid.maxIOut = 3;
}

LK9025::~LK9025()
{
}

void LK9025::setOutput()
{
    if (this->controlMode == RELAX_MODE)
    {
        this->currentSet = 0.0f; // 松开模式下电流设定为0
    }

    if (this->controlMode == CUR_MODE)
    {
        this->currentSet = this->torqueSet * 392.78f;
    }

    if (this->controlMode == POS_MODE)
    {
        // 外环控制，位置环控制
        this->positionPid.ref = this->positionSet;
        this->positionPid.fdb = this->motorFeedback.positionFdb;

        if (this->positionSet - this->positionPid.fdb > Math::Pi)
        {
            this->positionPid.ref -= 2 * Math::Pi;
        }
        else if (this->positionSet - this->positionPid.fdb < -Math::Pi)
        {
            this->positionPid.ref += 2 * Math::Pi;
        }


        this->positionPid.UpdateResult();
        this->speedPid.ref = this->positionPid.result;
    }
        // 内环控制，速度环控制

        this->speedPid.fdb = this->motorFeedback.speedFdb;
        this->speedPid.UpdateResult();

        this->currentSet = this->speedPid.result; // 根据速度PID结果设置电流
    
    // if (this->controlMode == POS_MODE)
    // {
    //     this->currentSet = 0; // 位置模式下的具体控制逻辑未定义
    // }
    // 其他控制模式下的电流设定逻辑同样待确定
	if (this->controlMode == POS_FOR_NO_SPD_MODE || this->controlMode == IMU_MODE)
	{
		this->currentSet = 0; // 具体控制逻辑未定义
	}
	else
	{
		this->currentSet = 0; // 其他情况电流设定为0
	}

	// 限制电流输出不超过最大值
	if (currentSet > maxCurrent)
		currentSet = maxCurrent;
	else if (currentSet < -maxCurrent)
		currentSet = -maxCurrent;
}
