//
// Created by Administrator on 2026/3/18.
//

#include "tim.h"
#include "gpio.h"

#include "motor.h"

const uint32_t RATIO = 20; // motor decceration ratio
const uint32_t FREQUENCY = 100; // tim4 interrupt frequency
const uint32_t LINE = 13;
const uint32_t REV = LINE * 4 * RATIO;
const int32_t MAX_pwm = 3599;

volatile int32_t leftMotorPwm = 0;
volatile int32_t rightMotorPwm = 0;
volatile int32_t leftMotorDegs = 0;
volatile int32_t rightMotorDegs = 0;

void allMotorInit() {
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
}

void setRightMotorPwm(int32_t pwm) {
    uint32_t pwm_val = 0;
    if (pwm >= 0) {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
        pwm_val = (uint32_t)pwm;
    } else if (pwm < 0) {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
        pwm_val = (uint32_t)(-pwm);
    }
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm_val);
    rightMotorPwm = pwm;
}

void setLeftMotorPwm(int32_t pwm) {
    uint32_t pwm_val = 0;
    if (pwm >= 0) {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
        pwm_val = (uint32_t)pwm;
    } else if (pwm < 0) {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
        pwm_val = (uint32_t)(-pwm);
    }
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm_val);
    leftMotorPwm = pwm;
}

void updateRightMotorSpeed() {
    int16_t count = (int16_t)__HAL_TIM_GET_COUNTER(&htim1);
    __HAL_TIM_SET_COUNTER(&htim1, 0);

    // float rpm = (float)count * 60.0f * (float)FREQUENCY / (float)REV;
    // rightMotorDeg = rpm * 6;
    rightMotorDegs = ((int32_t)count * 450) / 13;
}

void updateLeftMotorSpeed() {
    int16_t count = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
    __HAL_TIM_SET_COUNTER(&htim2, 0);

    // float rpm = (float)count * 60.0f * (float)FREQUENCY / (float)REV;
    // leftMotorDeg = rpm * 6;
    leftMotorDegs = ((int32_t)count * 450) / 13;
}