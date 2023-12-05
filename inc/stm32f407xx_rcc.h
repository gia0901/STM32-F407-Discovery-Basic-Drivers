/*
 * stm32f407xx_rcc.h
 *
 *  Created on: Nov 30, 2023
 *      Author: gia nguyen
 */

#ifndef INC_STM32F407XX_RCC_H_
#define INC_STM32F407XX_RCC_H_

#include "stm32f407xx.h"



uint32_t RCC_Get_PCLK1_Value();	// APB1
uint32_t RCC_Get_PCLK2_Value();	// APB2

#endif /* INC_STM32F407XX_RCC_H_ */
