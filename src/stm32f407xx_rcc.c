/*
 * stm32f407xx_rcc.c
 *
 *  Created on: Nov 30, 2023
 *      Author: gia nguyen
 */
#include "stm32f407xx_rcc.h"


uint16_t AHB_prescaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint8_t  APB_prescaler[4] = {2, 4, 8, 16};

uint32_t RCC_Get_PCLK1_Value()
{
	uint8_t check_system_clk;
	uint32_t SysClk_freq, Pclk1_freq;
	uint16_t apb1_pre, ahb1_pre;
	uint8_t temp;

	/* Check which clock source is the system clock */
	check_system_clk = (RCC->RCC_CFGR >> RCC_CFGR_SWS) & 0x3;
	if (check_system_clk == 0)
	{
		// HSI is used: 16 Mhz
		SysClk_freq = 16000000;
	}
	else if (check_system_clk == 1)
	{
		// HSE is used: 8 Mhz
		SysClk_freq = 8000000;
	}

	/* Calculate AHB prescaler */
	temp = (RCC->RCC_CFGR >> RCC_CFGR_HRPE) & 0xF;

	if (temp < 8)
	{
		ahb1_pre = 1;
	}
	else
	{
		ahb1_pre = AHB_prescaler[temp - 8];	// temp = 9, 10, 11,...
	}

	/* Calculate APB1 prescaler */
	temp = (RCC->RCC_CFGR >> RCC_CFGR_PPRE1) & 0x7;

	if (temp < 4)
	{
		apb1_pre = 1;
	}
	else
	{
		apb1_pre = APB_prescaler[temp - 4];
	}

	/* Calculate Pclk1 frequency */
	Pclk1_freq = (SysClk_freq / ahb1_pre) / apb1_pre;

	return Pclk1_freq;
}
