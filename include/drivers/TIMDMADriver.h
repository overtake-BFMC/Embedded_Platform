#ifndef WSDRIVER_H
#define WSDRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

//#include "globals.hpp"
//#include "stm32f4xx_it.h"

extern TIM_HandleTypeDef htim3;
extern DMA_HandleTypeDef hdma_tim3_ch3;

void HAL_TIM_MspPostInit( TIM_HandleTypeDef *htim );

void MX_DMA_Init(void);
void MX_TIM3_Init(void);

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_pwm);
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* htim_pwm);

void DMA1_Stream7_IRQHandler(void);

void startDMAHelper(uint16_t *pwmData, uint32_t idx);
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif

extern volatile bool bool_global_dmaPulse_isActive;

#endif