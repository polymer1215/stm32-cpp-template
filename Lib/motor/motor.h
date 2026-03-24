//
// Created by Administrator on 2026/3/18.
//

#ifndef MOTOR_H
#define MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

extern volatile int32_t leftMotorPwm;
extern volatile int32_t rightMotorPwm;
    extern volatile int32_t leftMotorDegs;
    extern volatile int32_t rightMotorDegs;

void allMotorInit();
void updateRightMotorSpeed();
void updateLeftMotorSpeed();
void setLeftMotorPwm(int32_t pwm);
void setRightMotorPwm(int32_t pwm);

#ifdef __cplusplus
}
#endif

#endif //MOTOR_H
