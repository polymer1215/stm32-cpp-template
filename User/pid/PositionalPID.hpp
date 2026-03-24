//
// Created by Junye Peng on 2026/3/24.
//

#include "PIDController.hpp"

#ifndef POSITIONALPID_HPP
#define POSITIONALPID_HPP

class PositionalPID : public PIDController
{
public:
    using PIDController::PIDController;

    int32_t compute(int32_t target, int32_t measure) override;
    void reset() override;

private:
    int32_t integral = 0;
    int32_t errLast = 0;
    int32_t integralMax = 0;
};

#endif