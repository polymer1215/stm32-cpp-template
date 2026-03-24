//
// Created by Junye Peng on 2026/3/24.
//

#include "PIDController.hpp"
#include "PIDController.hpp"

PIDController::PIDController(float p, float i, float d, int32_t o_min, int32_t o_max) :
    outMin(o_min), outMax(o_max)
{
    kp = static_cast<int32_t>(p * PID_SCALE);
    ki = static_cast<int32_t>(i * PID_SCALE);
    kd = static_cast<int32_t>(d * PID_SCALE);
}