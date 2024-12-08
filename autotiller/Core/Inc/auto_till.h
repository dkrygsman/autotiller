/**
  ******************************************************************************
  * @file    auto_till.h
  * @author  Daniel Krygsman
  * @brief   push pull header file
  ******************************************************************************
  *
  ******************************************************************************
  */

#include "stm32f4xx_hal_def.h"



void push_pull_init(GPIO_TypeDef* GPIO_forward_port, uint16_t GPIO_forward_pin, GPIO_TypeDef* GPIO_reverse_port, uint16_t GPIO_reverse_pin);

void linear_set_stateM(uint32_t target_value, GPIO_TypeDef* GPIO_forward_port, uint16_t GPIO_forward_pin, GPIO_TypeDef* GPIO_reverse_port, uint16_t GPIO_reverse_pin, uint32_t pot);

void linear_set(uint32_t target_value, GPIO_TypeDef* GPIO_forward_port, uint16_t GPIO_forward_pin, GPIO_TypeDef* GPIO_reverse_port, uint16_t GPIO_reverse_pin, uint32_t pot);

int remap_val(float value, float in_min, float in_max, float out_min, float out_max);
