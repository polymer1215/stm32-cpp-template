//
// Created by Junye Peng on 2026/3/25.
//

#ifndef POSTURECONTROL_HPP
#define POSTURECONTROL_HPP

#include "../pid/PositionalPID.hpp"
extern "C" {
    #include "../../Lib/mpu6050/mpu6050.h"
    #include "../../Lib/motor/motor.h"
}

class PostureControl
{
public:
    static PostureControl& getInstance()
    {
        static PostureControl instance;
        return instance;
    }

    PostureControl();

    void init();
    void updatePostureControl();
    void setTargetSpeed(float speed);
    void setMechanicalZero(float angle);

private:
    // Inner Loop (Balance)
    // Input: Angle, Output: PWM
    PositionalPID balancePID;
    
    // Outer Loop (Velocity)
    // Input: Speed, Output: Target Angle
    PositionalPID velocityPID;

    float targetSpeed = 0.0f;
    
    // Previous encoder values for speed calculation
    int32_t lastLeftDegs = 0;
    int32_t lastRightDegs = 0;
    
    // Angle offset (mechanical zero)
    float mechanicalZeroAngle = 5.0f;

    // Constants
    // Note: These need tuning!
    static constexpr float BALANCE_KP = 0.8f * 0.6;
    static constexpr float BALANCE_KI = 0.0f;
    static constexpr float BALANCE_KD = 6.5f * 0.6;

    static constexpr float VELOCITY_KP = 1800.0f;
    static constexpr float VELOCITY_KI = VELOCITY_KP / 200;
    static constexpr float VELOCITY_KD = 0.0f;

    static constexpr int32_t MAX_PWM = 3599;
};

#endif
