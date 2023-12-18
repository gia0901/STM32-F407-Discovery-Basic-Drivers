/*
 * stm32f407xx_rcc.h
 *
 *  Created on: Nov 30, 2023
 *      Author: gia nguyen
 */

#ifndef INC_STM32F407XX_RCC_H_
#define INC_STM32F407XX_RCC_H_

#include "stm32f407xx.h"

typedef struct
{
	uint8_t PLLState;
	uint8_t PLLSource;
	uint8_t PLLM;
	uint8_t PLLN;
} RCC_PLLInit_t;

typedef struct
{
	uint8_t OscType;
	RCC_PLLInit_t PLL;
} RCC_OscInit_t;



typedef struct
{
	uint8_t ClockType;			// Clock to be configured
	uint32_t SysclkSource;		// Clock will be supplied for System clock
	uint8_t APB1ClockDivider;	//
	uint8_t APB2ClockDivider;
	uint8_t AHBClockDivider;
} RCC_ClkInit_t;

uint8_t RCC_GetFlagStatus(uint32_t FlagName);
void RCC_OscInit(RCC_OscInit_t *pOscHandle);
void RCC_ClkInit(RCC_ClkInit_t *pOscHandle);


uint32_t RCC_Get_SYSCLK_Value();	// System clock
uint32_t RCC_Get_HCLK_Value();		// AHB clock
uint32_t RCC_Get_PCLK1_Value();		// APB1 clock
uint32_t RCC_Get_PCLK2_Value();		// APB2 clock






/****************** Oscillator Configuration Macros *********************/

/* Osc Type */
#define RCC_OSC_TYPE_HSI	0
#define RCC_OSC_TYPE_HSE	1

/* HSI state */
#define RCC_HSI_OFF			DISABLE
#define RCC_HSI_ON			ENABLE

/* HSE state */
#define RCC_HSE_OFF			DISABLE
#define RCC_HSE_ON			ENABLE

/* PLL state */
#define RCC_PLL_OFF			DISABLE
#define RCC_PLL_ON			ENABLE

/* PLL source */
#define RCC_PLL_SOURCE_HSI	0
#define RCC_PLL_SOURCE_HSE	1



/****************** Clock Configuration Macros *********************/

/* SYSCLK source */
#define RCC_SYSCLK_SOURCE_HSI		0
#define RCC_SYSCLK_SOURCE_HSE		1
#define RCC_SYSCLK_SOURCE_PLLCLK	2

/* Clock Type */
#define RCC_CLOCKTYPE_SYSCLK		1
#define RCC_CLOCKTYPE_HCLK			2
#define RCC_CLOCKTYPE_PCLK1			4
#define RCC_CLOCKTYPE_PCLK2			8

/* AHB Clock Divider --> APB1/APB2 clock */
#define RCC_AHB_DIV1				0
#define RCC_AHB_DIV2				4
#define RCC_AHB_DIV4				5
#define RCC_AHB_DIV8				6
#define RCC_AHB_DIV16				7

/* SYSCLK Divider --> AHB clock */
#define RCC_SYSCLK_DIV1				0
#define RCC_SYSCLK_DIV2				8
#define RCC_SYSCLK_DIV4				9
#define RCC_SYSCLK_DIV8				10
#define RCC_SYSCLK_DIV16			11
#define RCC_SYSCLK_DIV64			12
#define RCC_SYSCLK_DIV128			13
#define RCC_SYSCLK_DIV256			14
#define RCC_SYSCLK_DIV512			15

/* MCO1 Source & Divider */
#define RCC_MCO1_SOURCE_HSI			0
#define RCC_MCO1_SOURCE_HSE			2
#define RCC_MCO1_SOURCE_PLL			3

#define RCC_MCO1_DIV1				0
#define RCC_MCO1_DIV2				4
#define RCC_MCO1_DIV3				5
#define RCC_MCO1_DIV4				6
#define RCC_MCO1_DIV5				7

/* Flash Latency */
#define RCC_FLASH_ACR_0WS			0


/**********************  Flag status  ***********************/

#define RCC_FLAG_HSI_READY	(1 << RCC_CR_HSIRDY)
#define RCC_FLAG_HSE_READY	(1 << RCC_CR_HSERDY)
#define RCC_FLAG_PLL_READY	(1 << RCC_CR_PLLRDY)

#endif /* INC_STM32F407XX_RCC_H_ */
