//
// Created by Junye Peng on 2026/3/24.
//

#ifndef STM_CPP_TEST_SPEEDCONTROL_HPP
#define STM_CPP_TEST_SPEEDCONTROL_HPP

#include "../pid/PositionalPID.hpp"
#include "../../Lib/motor/motor.h"
extern "C" {
#include "../../Lib/mpu6050/mpu6050.h"
}

class SpeedControl
{
private:
    const float KP = 5.0f;
    const float KI = 1.55f;
    const float KD = 0.2f;
    
    // Balance PID parameters
    const float VERTICAL_KP = -80.0f; // Tune these values
    const float VERTICAL_KD = -0.5f;
    
    const int32_t PWM_MIN = -3599;
    const int32_t PWM_MAX = 3599;

    PositionalPID leftPID;
    PositionalPID rightPID;

    int32_t leftDegsTarget = 0;
    int32_t rightDegsTarget = 0;
    float targetAngle = 0.0f;

    bool PIDEnabled = false;
    bool balanceEnabled = false;
    
public:
    static SpeedControl& getInstance()
    {
        static SpeedControl instance;
        return instance;
    }

    SpeedControl();

    void setLeftDegsTarget(int32_t degs);
    void setRightDegsTarget(int32_t degs);
    void updateSpeedControl();
    bool getPIDEnabled();
    void setPIDEnabled(bool enabled);
    void setBalanceEnabled(bool enabled);
    void setTargetAngle(float angle);
    void updateBalanceControl();
};

#endif //STM_CPP_TEST_SPEEDCONTROL_HPP