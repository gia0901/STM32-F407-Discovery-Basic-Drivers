/*
 * stm32f407xx_rcc.c
 *
 *  Created on: Nov 30, 2023
 *      Author: gia nguyen
 */
#include "stm32f407xx_rcc.h"


uint16_t AHB_PreTable[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint8_t  APB_PreTable[4] = {2, 4, 8, 16};


static void RCC_HSI_Enable(void)
{
	// Enable HSI
	RCC->RCC_CR |= (1 << RCC_CR_HSION);
	// Wait until HSI is ready
	while (RCC_GetFlagStatus(RCC_FLAG_HSI_READY) == FLAG_NOT_SET);
}

static void RCC_HSE_Enable(void)
{
	// Enable HSE
	RCC->RCC_CR |= (1 << RCC_CR_HSEON);
	while (RCC_GetFlagStatus(RCC_FLAG_HSE_READY) == FLAG_NOT_SET);
}

static void RCC_PLL_Enable(void)
{
	// Enable PLL
	RCC->RCC_CR |= (1 << RCC_CR_PLLON);
	while (RCC_GetFlagStatus(RCC_FLAG_PLL_READY) == FLAG_NOT_SET);
}

static void RCC_PLL_Config(RCC_PLLInit_t PLL)
{

}

void RCC_OscInit(RCC_OscInit_t *pOscHandle)
{
	/* PLL is not used */
	if (pOscHandle->PLL.PLLState != RCC_PLL_ON)
	{
		if (pOscHandle->OscType == RCC_OSC_TYPE_HSI)
		{
			RCC_HSI_Enable();
		}
		else if (pOscHandle->OscType == RCC_OSC_TYPE_HSE)
		{
			RCC_HSE_Enable();
		}
	}
	/* PLL is used */
	else if (pOscHandle->PLL.PLLState == RCC_PLL_ON)
	{
		/* Configure PLL parameters */
		RCC_PLL_Config(pOscHandle->PLL);

		/* Configure PLL source */
		if (pOscHandle->PLL.PLLSource == RCC_PLL_SOURCE_HSI)
		{
			/* Enable HSI */
			RCC_HSI_Enable();

			/* Set HSI as PLL source. Must set before enabling PLL */
			RCC->RCC_PLLCFGR &= ~(1 << RCC_PLLCFGR_PLLSRC);
		}
		else if (pOscHandle->PLL.PLLSource == RCC_PLL_SOURCE_HSE)
		{
			/* Enable HSE */
			RCC_HSE_Enable();

			/* Set HSE as PLL source. Must set before enabling PLL */
			RCC->RCC_PLLCFGR |= (1 << RCC_PLLCFGR_PLLSRC);
		}

		/* Enable PLL */
		RCC_PLL_Enable();

		/* Wait for Flash Latency */


	}

}

void RCC_ClkInit(RCC_ClkInit_t *pClkHandle)
{
	/******************* Init HCLK = SYCLK / divider *******************/
	if ((pClkHandle->ClockType & RCC_CLOCKTYPE_HCLK) == RCC_CLOCKTYPE_HCLK)
	{
		RCC->RCC_CFGR |= (pClkHandle->AHBClockDivider << RCC_CFGR_HRPE);
	}


	/******************* PCLK1 (APB1 clock = AHB / divider) Configuration *******************/
	if ((pClkHandle->ClockType & RCC_CLOCKTYPE_PCLK1) == RCC_CLOCKTYPE_PCLK1)
	{
		/* Set the divider */
		RCC->RCC_CFGR |= (pClkHandle->APB1ClockDivider << RCC_CFGR_PPRE1);
	}

	/******************* PCLK2 (APB2 clock) Configuration *******************/
	if ((pClkHandle->ClockType & RCC_CLOCKTYPE_PCLK1) == RCC_CLOCKTYPE_PCLK2)
	{
		/* Set the divider */
		RCC->RCC_CFGR |= (pClkHandle->APB2ClockDivider << RCC_CFGR_PPRE2);
	}

	/******************* SYSCLK (Clock source) Configuration *******************/

	/* Wait until the Clock is ready */
	if (pClkHandle->SysclkSource == RCC_SYSCLK_SOURCE_HSI)
	{
		/* Wait until HSI is ready */
		while (RCC_GetFlagStatus(RCC_FLAG_HSI_READY) == FLAG_NOT_SET);
	}
	else if (pClkHandle->SysclkSource == RCC_SYSCLK_SOURCE_HSE)
	{
		/* Wait until HSE is ready */
		while (RCC_GetFlagStatus(RCC_FLAG_HSE_READY) == FLAG_NOT_SET);
	}
	else if (pClkHandle->SysclkSource == RCC_SYSCLK_SOURCE_PLLCLK)
	{
		/* Wait until PLL is ready */
		while (RCC_GetFlagStatus(RCC_FLAG_PLL_READY) == FLAG_NOT_SET);
	}

	/* Set the clock as SYSCLK System Clock */
	RCC->RCC_CFGR |= (pClkHandle->SysclkSource << RCC_CFGR_SW);



}

/*
 * MCO functions must be configured before enabling the clock or PLL
 */
void RCC_MCO1Config(uint8_t MCO1Source, uint8_t MCO1Div)
{
	uint32_t tempreg = 0;

	tempreg |= (MCO1Source << RCC_CFGR_MCO1);
	tempreg |= (MCO1Div << RCC_CFGR_MCO1PRE);

	RCC->RCC_CFGR &= ~(0x3 << RCC_CFGR_MCO1);		// Clear 2 bits of MCO1 field
	RCC->RCC_CFGR &= ~(0x7 << RCC_CFGR_MCO1PRE);	// Clear 2 bits of MCO1 prescaler field

	RCC->RCC_CFGR |= tempreg;

}

uint32_t RCC_Get_PLL_Value()
{

	return 0;
}

uint32_t RCC_Get_SYSCLK_Value()
{
	uint8_t check_sysclk_source;
	uint32_t sysclk_freq;

	check_sysclk_source = (RCC->RCC_CFGR >> RCC_CFGR_SWS) & 0x3;
	if (check_sysclk_source == 0)
	{
		// HSI is used as system clock source
		sysclk_freq = 16000000;
	}
	else if (check_sysclk_source == 1)
	{
		// HSE is used as system clock source
		sysclk_freq = 8000000;
	}
	else if (check_sysclk_source == 2)
	{
		// PLL is used as system clock source
		sysclk_freq = RCC_Get_PLL_Value();
	}
	return sysclk_freq;
}

/*
 * Note: HCLK (AHB clock) = Sysclock / AHB prescaler
 */
uint32_t RCC_Get_HCLK_Value()
{
	uint32_t sysclk_freq, hclk_freq;
	uint8_t ahb1_pre;
	uint8_t temp;

	sysclk_freq = RCC_Get_SYSCLK_Value();

	/* Get AHB prescaler value */
	temp = (RCC->RCC_CFGR >> RCC_CFGR_HRPE) & 0xF;

	if (temp < 8)
		ahb1_pre = 1;
	else
		ahb1_pre = AHB_PreTable[temp - 8];


	/* Calculate HCLK frequency */
	hclk_freq = sysclk_freq / ahb1_pre;

	return hclk_freq;
}


/*
 * Note: PCLK1 (APB1 clock) = HCLK / APB1 Prescaler
 */
uint32_t RCC_Get_PCLK1_Value()
{
	uint32_t hclk_freq, pclk1_freq;
	uint16_t apb1_pre;
	uint8_t temp;

	hclk_freq = RCC_Get_HCLK_Value();

	/* Calculate APB1 prescaler */
	temp = (RCC->RCC_CFGR >> RCC_CFGR_PPRE1) & 0x7;

	if (temp < 4)
		apb1_pre = 1;
	else
		apb1_pre = APB_PreTable[temp - 4];

	/* Calculate Pclk1 frequency */
	pclk1_freq = hclk_freq / apb1_pre;

	return pclk1_freq;
}

uint32_t RCC_Get_PCLK2_Value()
{
	uint32_t hclk_freq, pclk2_freq;
	uint16_t apb2_pre;
	uint8_t temp;

	hclk_freq = RCC_Get_HCLK_Value();

	temp = (RCC->RCC_CFGR >> RCC_CFGR_PPRE2) & 0x7;
	if (temp < 4)
		apb2_pre = 1;
	else
		apb2_pre = APB_PreTable[temp - 4];

	pclk2_freq = hclk_freq / apb2_pre;

	return pclk2_freq;
}



uint8_t RCC_GetFlagStatus(uint32_t FlagName)
{
	if (RCC->RCC_CR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_NOT_SET;
}

