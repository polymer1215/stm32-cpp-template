//
// Created by Junye Peng on 2026/3/25.
//

#include "State_balance.hpp"
#include "../../Lib/motor/motor.h"
#include "../timer/TimerTask.hpp"
#include "../control/PostureControl.hpp"

void State_balance::init()
{
    PostureControl::getInstance().setTargetSpeed(0);
}

void State_balance::loop()
{
}

void State_balance::exit()
{
    setLeftMotorPwm(0);
    setRightMotorPwm(0);
}
