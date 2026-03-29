//
// Created by Junye Peng on 2026/3/24.
//

#include "main_entry.h"
#include "../timer/timer.hpp"
#include "../timer/TimerTask.hpp"
#include "../state/StateMachine.hpp"
#include "../state/State_balance.hpp"

extern "C" {
#include "../../Core/Inc/main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "../../Lib/motor/motor.h"
#include "../../Lib/mpu6050/mpu6050.h"
// #include "../control/SpeedControl.hpp"
#include "../control/PostureControl.hpp"
#include <stdio.h>
}

volatile uint32_t tim4_irq_ticks = 0;

void main_entry(void)
{
    MPU6050_initialize();
    allMotorInit();

    PostureControl::getInstance();
    PostureControl::getInstance().init();

    StateMachine::getInstance();
    StateMachine::getInstance().changeState(State_balance::getInstance());

    HAL_TIM_Base_Start_IT(&htim4);

    uint32_t lastTimeMillis = 0;
    uint32_t main_tim4_ticks = 0;

    for (;;)
    {
        Read_DMP();
        
        if (tim4_irq_ticks != main_tim4_ticks)
        {
            main_tim4_ticks++;
            PostureControl::getInstance().updatePostureControl();
            StateMachine::getInstance().stateLoop();

            // Handle the timed callback
            TimerTask::Update();
        }

        if (BenchTimer::timeMillis - lastTimeMillis >= 1000)
        {
            // printf("test\r\n");
            // printf("left pwm:%d\r\n", leftMotorPwm);
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            printf("Pitch: %d, Roll: %d, Yaw: %d\r\n", (int32_t)(Pitch * 1000),
                (int32_t)(Roll * 1000), (int32_t)(Yaw * 1000));
            lastTimeMillis = BenchTimer::timeMillis;
        }

    }
}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4)
    {
        BenchTimer::timeMillis += 10;
        updateLeftMotorSpeed();
        updateRightMotorSpeed();
        tim4_irq_ticks++;
    }
}

extern "C" int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, 0xFFFF);
    return len;
}

extern "C" int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
}