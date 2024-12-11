/**
  ******************************************************************************
  * @file    push_pull.c
  * @author  Daniel Krygsman
  * @brief   push pull c file
  *
  ******************************************************************************
  *
  *
  ******************************************************************************
*/
#include <stdio.h>
#include <stdlib.h>
#include "stm32f4xx_hal.h"
#include "push_pull.h"

#define THRESHOLD 50

uint32_t state;


/**
  * @brief  sets gpio pins high or low for forward or reverse, from microcontroller to h-bridge to drive the linear actuator
  *
  * @note
  *
  * @param
  * @param
  * @param
  * @retval None
  */
void push_pull_init(GPIO_TypeDef* GPIO_forward_port, uint16_t GPIO_forward_pin, GPIO_TypeDef* GPIO_reverse_port, uint16_t GPIO_reverse_pin);




/**
  * @brief  sets gpio pins high or low for forward or reverse, from microcontroller to h-bridge to drive the linear actuator
  *
  * @note
  *
  * @param
  * @param
  * @param
  * @retval None
  */
void linear_set(uint32_t target_value, GPIO_TypeDef* GPIO_forward_port, uint16_t GPIO_forward_pin, GPIO_TypeDef* GPIO_reverse_port, uint16_t GPIO_reverse_pin, uint32_t pot);


/**
  * @brief  remapps a values range
  *
  * @note
  *
  * @param
  * @param
  * @param
  * @retval None
  */
int remap_val(uint32_t value, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max);
