//
// Created by Junye Peng on 2026/3/24.
//

#include "IncrementalPID.hpp"

int32_t IncrementalPID::compute(int32_t target, int32_t measure)
{
    int32_t err = target - measure;

    int32_t deltaOut = (kp * (err - errLast) + ki * err + kp * (err - 2 * errLast + errPrev))
        >> PID_SHIFT;

    currentOut += deltaOut;
    errPrev = errLast;
    errLast = err;

    currentOut = clamp(currentOut);
    return currentOut;
}

void IncrementalPID::reset()
{
    errLast = 0;
    errPrev = 0;
    currentOut = 0;
}
