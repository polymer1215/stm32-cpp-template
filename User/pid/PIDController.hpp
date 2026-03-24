//
// Created by Junye Peng on 2026/3/24.
//

#ifndef _PID_CONTROLLER_HPP_
#define _PID_CONTROLLER_HPP_

#include <stdint.h>

class PIDController
{
public:
    static constexpr int32_t PID_SHIFT = 10;
    static constexpr int32_t PID_SCALE = (1 << PID_SHIFT);

    PIDController(float p, float i, float d, int32_t o_min, int32_t o_max);
    virtual ~PIDController() = default;

    virtual int32_t compute(int32_t target, int32_t measure) = 0;
    virtual void reset() = 0;
protected:
    int32_t kp;
    int32_t ki;
    int32_t kd;
    int32_t outMax;
    int32_t outMin;

    int32_t clamp(int32_t value)
    {
        if (value > outMax) return outMax;
        if (value < outMin) return outMin;
        return value;
    }
};

#endif