//
// Created by Junye Peng on 2026/3/24.
//

#include "SpeedControl.hpp"

#include <ios>

#include "../pid/PositionalPID.hpp"
#include "../../Lib/motor/motor.h"

SpeedControl::SpeedControl() : leftPID(KP, KI, KD, PWM_MIN, PWM_MAX),
    rightPID(KP, KI, KD, PWM_MIN, PWM_MAX), PIDEnabled(false), balanceEnabled(false)
{
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


void SpeedControl::setBalanceEnabled(bool enabled)
{
    balanceEnabled = enabled;
    if (enabled) {
        // Disable regular PID when balance mode is active
        PIDEnabled = false;
    }
}

void SpeedControl::setTargetAngle(float angle)
{
    targetAngle = angle;
}

void SpeedControl::updateBalanceControl()
{
    if (!balanceEnabled) return;
    
    // Vertical Control Loop (Balance Loop)
    // Using Pitch angle and Y-axis Gyroscope
    // PWM = Kp * (CurrentAngle - TargetAngle) + Kd * GyroSpeed
    
    float angle_err = Pitch - targetAngle;
    float gyro_val = (float)gyro[1]; // Y-axis gyro
    
    // Simple PD control for balancing
    float vertical_pwm = VERTICAL_KP * angle_err + VERTICAL_KD * gyro_val;
    
    int32_t final_pwm = (int32_t)vertical_pwm;
    
    // Clamp PWM
    if (final_pwm > PWM_MAX) final_pwm = PWM_MAX;
    if (final_pwm < PWM_MIN) final_pwm = PWM_MIN;
    
    setLeftMotorPwm(final_pwm);
    setRightMotorPwm(final_pwm);
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
    if (PIDEnabled)
    {
        int32_t leftOutput = leftPID.compute(leftDegsTarget, leftMotorDegs);
        int32_t rightOutput = rightPID.compute(rightDegsTarget, rightMotorDegs);

        setLeftMotorPwm(leftOutput);
        setRightMotorPwm(rightOutput);
    }
}
