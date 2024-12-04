#include "ShooterController.hpp"
#include "GimbalController.hpp"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void ShooterController::Init()
{
    SetCurrentState(&RelaxState);
    SetDefaultState(&RelaxState);

    /*--------------------左摩擦轮--------------------*/
    LeftFricMotor.controlMode = GM3508::SPD_MODE;

    // 速度环PID参数
    LeftFricMotor.speedPid.kp = 100.0f;
    LeftFricMotor.speedPid.ki = 0.0f;
    LeftFricMotor.speedPid.kd = 0.0f;

    LeftFricMotor.speedSet = 0;
    LeftFricMotor.setOutput();

    /*--------------------右摩擦轮--------------------*/
    RightFricMotor.controlMode = GM3508::SPD_MODE;

    // 速度环PID参数
    RightFricMotor.speedPid.kp = 100.0f;
    RightFricMotor.speedPid.ki = 0.0f;
    RightFricMotor.speedPid.kd = 0.0f;

    RightFricMotor.speedSet = 0;
    RightFricMotor.setOutput();

    /*--------------------拨弹轮--------------------*/
    TriggerMotor.controlMode = GM2006::SPD_MODE;

    // 速度环PID参数
    TriggerMotor.speedPid.kp = 100.0f;
    TriggerMotor.speedPid.ki = 0.0f;
    TriggerMotor.speedPid.kd = 0.0f;

    TriggerMotor.speedSet = 0;
    TriggerMotor.setOutput();

    // 注册电机
    GMMotorHandler::instance()->registerMotor(&LeftFricMotor, &hcan1, 0x201);
    GMMotorHandler::instance()->registerMotor(&RightFricMotor, &hcan1, 0x202);
    GMMotorHandler::instance()->registerMotor(&TriggerMotor, &hcan1, 0x203);

    // 日志输出
    Monitor::Instance()->Log_Messages(Monitor::INFO, (uint8_t *)("ShooterController Init Success!\r\n"));
}

void ShooterController::Run()
{
    HandleInput();
    Update();
}

void ShooterController::HandleInput()
{
    if (Dr16::Instance()->QuerySwStatus(Dr16::RIGHT_SWITCH) == Dr16::RC_SW_MID) // 如果遥控器左边的拨杆中间
    {
        ChangeState(&WarmState);
    }
    else if (Dr16::Instance()->QuerySwStatus(Dr16::RIGHT_SWITCH) == Dr16::RC_SW_UP) // 如果遥控器左边的拨杆向上
    {
        ChangeState(&FireState);
    }
    else // 默认状态
    {
        ChangeState(&RelaxState);
    }

//    if (GimbalController::Instance()->ReceivePacket.fire == true)
//    {
//        GimbalController::Instance()->ReceivePacket.fire = false;
//        ChangeState(&FireState);
//    }

    if (Dr16::Instance()->QuerySwStatus(Dr16::RIGHT_SWITCH) == Dr16::RC_SW_DOWN) // 如果遥控器左边的拨杆向下
    {
        ChangeState(&RelaxState);
    }
}
