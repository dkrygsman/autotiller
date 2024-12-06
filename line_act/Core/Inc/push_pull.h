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
void push_pull_init(GPIO_TypeDef* GPIO_forward_port, uint16_t GPIO_forward_pin, GPIO_TypeDef* GPIO_reverse_port, uint16_t GPIO_reverse_pin)
{
	state = 1;

	HAL_GPIO_WritePin(GPIO_forward_port, GPIO_forward_pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIO_reverse_port, GPIO_reverse_pin, GPIO_PIN_RESET);

}




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
void linear_set(uint32_t target_value, GPIO_TypeDef* GPIO_forward_port, uint16_t GPIO_forward_pin, GPIO_TypeDef* GPIO_reverse_port, uint16_t GPIO_reverse_pin, uint32_t pot)
{

	if(target_value > 0 && target_value < 1100)
	{
		switch(state)
		{
			case 1:				//chill state
			{
				if((target_value- THRESHOLD) >= pot)	//transition to reverse state
				{
					HAL_GPIO_WritePin(GPIO_reverse_port, GPIO_reverse_pin, GPIO_PIN_SET);
					state = 2;
				}
				if((target_value+THRESHOLD) <= pot)			//transition to forward
				{
					HAL_GPIO_WritePin(GPIO_forward_port, GPIO_forward_pin, GPIO_PIN_SET);
					state = 3;
				}
				//printf("Not moving\t");
			}
			break;
			case 2:			//forward state
			{
				if((target_value- THRESHOLD) >= pot)	 		//transition to reverse
				{
					HAL_GPIO_WritePin(GPIO_forward_port, GPIO_forward_pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIO_reverse_port, GPIO_reverse_pin, GPIO_PIN_SET);
					state = 3;
				}
				if((target_value-THRESHOLD) < pot && (target_value+THRESHOLD) > pot)	//transition to chill state
				{
					HAL_GPIO_WritePin(GPIO_forward_port, GPIO_forward_pin, GPIO_PIN_RESET);
					state = 1;
				}
				//printf("Moving forward\t");
			}
			break;
			case 3:				//reverse state
			{
				if((target_value+THRESHOLD) <= pot)			 //transition to forward state
				{
					HAL_GPIO_WritePin(GPIO_reverse_port, GPIO_reverse_pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIO_forward_port, GPIO_forward_pin, GPIO_PIN_SET);
					state = 2;
				}
				if((target_value-THRESHOLD) < pot && (target_value+THRESHOLD) > pot)	//transition to chill
				{
					HAL_GPIO_WritePin(GPIO_reverse_port, GPIO_reverse_pin, GPIO_PIN_RESET);
					state = 1;
				}
				//printf("Moving backwards\t");
			}
			break;
		}
	}
	else
	{
		printf("Error, target value is outside of range");
		fflush(stdout);
	}
}


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
int remap_val(uint32_t value, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max)
{
	return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
