//
// Created by Junye Peng on 2026/3/24.
//

#ifndef INCREMENTALPID_HPP
#define INCREMENTALPID_HPP

#include "PIDController.hpp"
class IncrementalPID : public PIDController
{
public:
    using PIDController::PIDController;

    int32_t compute(int32_t target, int32_t measure) override;
    void reset() override;
private:
    int32_t errLast = 0;
    int32_t errPrev = 0;
    int32_t currentOut = 0;
};

#endif