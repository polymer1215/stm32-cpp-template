//
// Created by Junye Peng on 2026/3/24.
//

#include "State_test.hpp"

#include "../control/SpeedControl.hpp"
#include "../../Lib/motor/motor.h"
#include "../timer/TimerTask.hpp"
#include "gpio.h"

#include <stdio.h>

State_test::State_test()
{
    SpeedControl::getInstance().setPIDEnabled(false);
}

void State_test::init()
{
    // SpeedControl::getInstance().setPIDEnabled(true);
    // SpeedControl::getInstance().setLeftDegsTarget(0);
    // SpeedControl::getInstance().setRightDegsTarget(0);
    TimerTask::AddTask(timerTaskTest1, 1000);
    TimerTask::AddTask(timerTaskTest2, 1000);
    TimerTask::AddTask(timerTaskTest3, 1000);


}

void State_test::loop()
{
    printf("test\r\n");
}

void State_test::exit()
{

}

void State_test::timerTaskTest1()
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
}

void State_test::timerTaskTest2()
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
}

void State_test::timerTaskTest3()
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
}