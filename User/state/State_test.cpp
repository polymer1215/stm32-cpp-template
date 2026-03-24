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
    SpeedControl::getInstance().setPIDEnabled(true);
}

void State_test::init()
{
    SpeedControl::getInstance().setLeftDegsTarget(1000);
    SpeedControl::getInstance().setRightDegsTarget(1000);
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
}

void State_test::timerTaskTest2()
{
    // SpeedControl::getInstance().setLeftDegsTarget(1000);
    // SpeedControl::getInstance().setRightDegsTarget(1000);
}

void State_test::timerTaskTest3()
{
    // SpeedControl::getInstance().setLeftDegsTarget(500);
    // SpeedControl::getInstance().setRightDegsTarget(500);
}