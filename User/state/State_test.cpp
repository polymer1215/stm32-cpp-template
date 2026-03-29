//
// Created by Junye Peng on 2026/3/24.
//

#include "State_test.hpp"

// #include "../control/SpeedControl.hpp"
#include "../../Lib/motor/motor.h"
#include "../timer/TimerTask.hpp"
#include "gpio.h"

#include <stdio.h>

extern "C" {
#include "../../Lib/mpu6050/mpu6050.h"
}

State_test::State_test()
{
    // SpeedControl::getInstance().setBalanceEnabled(true);
}

void State_test::init()
{
    TimerTask::AddTask(balanceTestTask, 10);
}

void State_test::loop()
{
    // Print debug info
    printf("Pitch: %.2f, GyroY: %d\r\n", Pitch, gyro[1]);
}

void State_test::exit()
{
    // SpeedControl::getInstance().setBalanceEnabled(false);
}

void State_test::balanceTestTask()
{
    // SpeedControl::getInstance().updateBalanceControl();
}
