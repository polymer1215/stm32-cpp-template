//
// Created by Junye Peng on 2026/3/24.
//

#include "main_entry.h"
#include "../timer/timer.hpp"
#include "../timer/TimerTask.hpp"
#include "../state/StateMachine.hpp"
#include "../state/State_test.hpp"

extern "C" {
#include "../../Core/Inc/main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "../control/SpeedControl.hpp"
#include <stdio.h>
}

void main_entry(void)
{
    SpeedControl::getInstance();
    StateMachine::getInstance();
    StateMachine::getInstance().changeState(State_test::getInstance());
    HAL_TIM_Base_Start_IT(&htim4);

    uint32_t lastTimeMillis = 0;
    for (;;)
    {
        if (BenchTimer::timeMillis - lastTimeMillis >= 1000)
        {
            // printf("test\r\n");
            // printf("left pwm:%d\r\n", leftMotorPwm);
            lastTimeMillis = BenchTimer::timeMillis;
        }

    }
}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4)
    {
        BenchTimer::timeMillis += 10;
        SpeedControl::getInstance().updateSpeedControl();
        StateMachine::getInstance().stateLoop();

        // Handle the timed callback
        TimerTask::Update();
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