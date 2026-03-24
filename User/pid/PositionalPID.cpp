//
// Created by Junye Peng on 2026/3/24.
//

#include "PositionalPID.hpp"

int32_t PositionalPID::compute(int32_t target, int32_t measure)
{
    int32_t err = target - measure;
    if (integralMax == 0) integralMax = outMax * 4;

    integral += err;
    if (integral > integralMax) integral = integralMax;
    else if (integral < -integralMax) integral = -integralMax;

    int32_t output = (kp * err + ki * integral + kd * (err - errLast)) >> PID_SHIFT;

    errLast = err;
    return clamp(output);
}

void PositionalPID::reset()
{
    integral = 0;
    errLast = 0;
}