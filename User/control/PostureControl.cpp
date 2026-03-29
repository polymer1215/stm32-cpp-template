//
// Created by Junye Peng on 2026/3/25.
//

#include "PostureControl.hpp"
#include <math.h>

PostureControl::PostureControl()
    : balancePID(BALANCE_KP, BALANCE_KI, BALANCE_KD, -MAX_PWM, MAX_PWM),
      velocityPID(VELOCITY_KP, VELOCITY_KI, VELOCITY_KD, -10000, 10000)
{
}

void PostureControl::init()
{
    DMP_Init();

    // Reset PIDs
    balancePID.reset();
    velocityPID.reset();
    
    // Reset encoder tracking
    lastLeftDegs = leftMotorDegs;
    lastRightDegs = rightMotorDegs;
}

void PostureControl::setTargetSpeed(float speed)
{
    targetSpeed = speed;
}

void PostureControl::setMechanicalZero(float angle)
{
    mechanicalZeroAngle = angle;
}

void PostureControl::updatePostureControl()
{
    // 1. Read MPU6050 Data
    // Read_DMP is called in main loop, so we just use the global variables.
    // Use Roll as the balance angle (since Z-axis rotation is Roll/Pitch in side-mounted)
    // Try Roll first as it is usually the "tilt" when Y is up.
    float currentAngle = Roll;   
    Angle_Balance = currentAngle; // Update global for debug/display
    
    // 2. Calculate Speed
    // Note: leftMotorDegs/rightMotorDegs in motor.c are already calculated as Speed (Degrees/Second)
    // because the timer counter is reset to 0 in every interrupt.
    // So we should NOT subtract the "last" value, otherwise we are calculating Acceleration.
    
    // Average speed (degrees per second)
    float currentSpeed = (float)(leftMotorDegs + rightMotorDegs) / 2.0f;
    
    // 3. Velocity Loop (Outer Loop)
    // Input: Speed Error
    // Output: Target Angle Adjustment
    // We want the robot to tilt to correct velocity error.
    // If moving too fast forward (currentSpeed > targetSpeed), we want to tilt BACK (negative angle).
    // So targetAngle should be proportional to -SpeedError.
    
    int32_t velocityOutput = velocityPID.compute((int32_t)targetSpeed, (int32_t)currentSpeed);
    
    // Scale velocity output to angle. 
    // We limit the target angle to avoid falling over.
    float targetAngle = (float)velocityOutput;
    
    // Clamp target angle to safe range (e.g. +/- 10 degrees)
    if (targetAngle > 10.0f) targetAngle = 10.0f;
    if (targetAngle < -10.0f) targetAngle = -10.0f;
    
    // Add mechanical zero offset
    targetAngle += mechanicalZeroAngle;

    // 4. Balance Loop (Inner Loop)
    // Input: Angle Error
    // Output: Motor PWM
    // If CurrentAngle > TargetAngle (leaning forward), we need positive PWM to move forward and catch up.
    // Error = CurrentAngle - TargetAngle.
    // If Current (10) > Target (0), Error = 10.
    // Output = Kp * 10. Positive. Correct.
    
    // Scale up the angle by 100 to avoid precision loss when casting float to int32_t
    int32_t scaledCurrentAngle = (int32_t)(currentAngle * 100.0f);
    int32_t scaledTargetAngle = (int32_t)(targetAngle * 100.0f);

    int32_t balanceOutput = balancePID.compute(scaledCurrentAngle, scaledTargetAngle);
    
    // 5. Output to Motors
    // Add steering control if needed, but for now just balance
    // balanceOutput is already PWM
    
    // Check if fallen over (>30 degrees)
    // Protection disabled by user request. Motors will NOT stop even if robot falls.
    if (false && (currentAngle > 30.0f || currentAngle < -30.0f)) {
        setLeftMotorPwm(0);
        setRightMotorPwm(0);
        // Reset integral terms so they don't wind up while fallen
        balancePID.reset();
        velocityPID.reset();
    } else {
        setLeftMotorPwm(balanceOutput);
        setRightMotorPwm(balanceOutput);
    }
}

