//
// Created by Junye Peng on 2026/3/24.
//

#include "SpeedControl.hpp"

#include <ios>

#include "../pid/PositionalPID.hpp"
#include "../../Lib/motor/motor.h"

SpeedControl::SpeedControl() : leftPID(KP, KI, KD, PWM_MIN, PWM_MAX),
    rightPID(KP, KI, KD, PWM_MIN, PWM_MAX), PIDEnabled(false)
{
    allMotorInit();
}

bool SpeedControl::getPIDEnabled()
{
    return PIDEnabled;
}

void SpeedControl::setPIDEnabled(bool enabled)
{
    if (enabled)
    {
        PIDEnabled = true;
    } else
    {
        PIDEnabled = false;
        leftPID.reset();
        rightPID.reset();
    }

}

void SpeedControl::setLeftDegsTarget(int32_t degs)
{
    leftDegsTarget = degs;
}

void SpeedControl::setRightDegsTarget(int32_t degs)
{
    rightDegsTarget = degs;
}

void SpeedControl::updateSpeedControl()
{
    updateLeftMotorSpeed();
    updateRightMotorSpeed();

    if (PIDEnabled)
    {
        int32_t leftOutput = leftPID.compute(leftDegsTarget, leftMotorDegs);
        int32_t rightOutput = rightPID.compute(rightDegsTarget, rightMotorDegs);

        setLeftMotorPwm(leftOutput);
        setRightMotorPwm(rightOutput);
    }
}
