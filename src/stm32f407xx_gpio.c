/*
 * stm32f407xx_gpio.c
 *
 *  Created on: Nov 22, 2023
 *      Author: gia nguyen
 */
#include "stm32f407xx.h"



/*************************************************************************
 * @fn				- GPIO_Peripheral_Clk_Ctrl
 *
 * @brief			- Enable/Disable the GPIO peripheral clock
 *
 * @param[in]		- pGPIOx: GPIO instance
 * @param[in]		- EnorDi: Enable/Disable
 *
 * @return			- none
 *
 */
void GPIO_Peripheral_Clk_Ctrl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pGPIOx == GPIOA)
			RCC_GPIOA_CLK_EN();
		else if (pGPIOx == GPIOB)
			RCC_GPIOB_CLK_EN();
		else if (pGPIOx == GPIOC)
			RCC_GPIOC_CLK_EN();
		else if (pGPIOx == GPIOD)
			RCC_GPIOD_CLK_EN();
		else if (pGPIOx == GPIOE)
			RCC_GPIOE_CLK_EN();
		else if (pGPIOx == GPIOF)
			RCC_GPIOF_CLK_EN();
		else if (pGPIOx == GPIOG)
			RCC_GPIOG_CLK_EN();
	}
	else if (EnorDi == DISABLE)
	{
		if (pGPIOx == GPIOA)
			RCC_GPIOA_CLK_DI();
		else if (pGPIOx == GPIOB)
			RCC_GPIOB_CLK_DI();
		else if (pGPIOx == GPIOC)
			RCC_GPIOC_CLK_DI();
		else if (pGPIOx == GPIOD)
			RCC_GPIOD_CLK_DI();
		else if (pGPIOx == GPIOE)
			RCC_GPIOE_CLK_DI();
		else if (pGPIOx == GPIOF)
			RCC_GPIOF_CLK_DI();
		else if (pGPIOx == GPIOG)
			RCC_GPIOG_CLK_DI();
	}
}

/*************************************************************************
 * @fn				- GPIO_Init
 *
 * @brief			- Initialize the GPIO: clock, mode, speed, pull-up/down...
 *
 * @param[in]		- pGPIO_Handle: holds GPIO instance & configurations
 *
 * @return			- none
 */
void GPIO_Init(GPIO_Handle_t *pGPIO_Handle)
{
	uint32_t tempreg = 0;	// temporary register

	/* 1. Enable the GPIO peripheral clock */
	GPIO_Peripheral_Clk_Ctrl(pGPIO_Handle->pGPIOx, ENABLE);

	/* 2. Configure mode: interrupt & non-interrupt modes */
	if (pGPIO_Handle->GPIO_Config.GPIO_Mode <= GPIO_MODE_ANALOG)
	{
		/* non-interrupt mode: input, output, analog */
		tempreg |= (pGPIO_Handle->GPIO_Config.GPIO_Mode << (2*pGPIO_Handle->GPIO_Config.GPIO_PinNumber));
		pGPIO_Handle->pGPIOx->GPIO_MODER &= ~(0x3 << (2*pGPIO_Handle->GPIO_Config.GPIO_PinNumber));
		pGPIO_Handle->pGPIOx->GPIO_MODER |= tempreg;
	}
	else
	{
		/* configure interrupt mode: rising, falling edge detection or both */
		if (pGPIO_Handle->GPIO_Config.GPIO_Mode == GPIO_MODE_IT_FT)
		{
			// configure Falling trigger mode
			EXTI->EXTI_FTSR |= (1 << pGPIO_Handle->GPIO_Config.GPIO_PinNumber);	// EXTI line = Pin number
			// clear rising trigger to make sure it won't use both mode
			EXTI->EXTI_RTSR &= ~(1 << pGPIO_Handle->GPIO_Config.GPIO_PinNumber);

		}
		else if (pGPIO_Handle->GPIO_Config.GPIO_Mode == GPIO_MODE_IT_RT)
		{
			// configure Rising trigger mode
			EXTI->EXTI_RTSR |= (1 << pGPIO_Handle->GPIO_Config.GPIO_PinNumber);	// EXTI line = Pin number
			// clear falling trigger to make sure it won't use both mode
			EXTI->EXTI_FTSR &= ~(1 << pGPIO_Handle->GPIO_Config.GPIO_PinNumber);
		}
		else if (pGPIO_Handle->GPIO_Config.GPIO_Mode == GPIO_MODE_IT_FR)
		{
			// configure Falling trigger mode
			EXTI->EXTI_FTSR |= (1 << pGPIO_Handle->GPIO_Config.GPIO_PinNumber);	// EXTI line = Pin number
			// configure Rising trigger mode
			EXTI->EXTI_RTSR |= (1 << pGPIO_Handle->GPIO_Config.GPIO_PinNumber);
		}
		/* configure SYSCFG registers to enable EXTI */
		uint32_t temp1 = pGPIO_Handle->GPIO_Config.GPIO_PinNumber / 4;
		uint32_t temp2 = (pGPIO_Handle->GPIO_Config.GPIO_PinNumber % 4) * 4;
		uint8_t port_number = GPIO_BASEADDR_TO_PORT_NUMBER(pGPIO_Handle->pGPIOx);

		RCC_SYSCFG_CLK_EN();
		SYSCFG->SYSCFG_EXTICR[temp1] = (port_number << temp2);

		/* enable the delivery from to NVIC by enabling the mask bit in EXTI IMR*/
		EXTI->EXTI_IMR |= (1 << pGPIO_Handle->GPIO_Config.GPIO_PinNumber);
	}


	/* 3. Configure other settings */
	// input pull-up/pull-down/no pull
	tempreg = (pGPIO_Handle->GPIO_Config.GPIO_PUPD << (2*pGPIO_Handle->GPIO_Config.GPIO_PinNumber));
	pGPIO_Handle->pGPIOx->GPIO_PUPDR &= ~(0x3 << (2*pGPIO_Handle->GPIO_Config.GPIO_PinNumber));
	pGPIO_Handle->pGPIOx->GPIO_PUPDR |= tempreg;

	// output type: push-pull/open-drain
	tempreg = (pGPIO_Handle->GPIO_Config.GPIO_OutputType << pGPIO_Handle->GPIO_Config.GPIO_PinNumber);
	pGPIO_Handle->pGPIOx->GPIO_OTYPER &= ~(0x1 << pGPIO_Handle->GPIO_Config.GPIO_PinNumber);
	pGPIO_Handle->pGPIOx->GPIO_OTYPER |= tempreg;

	// output speed
	tempreg = (pGPIO_Handle->GPIO_Config.GPIO_OutputSpeed << (2 * pGPIO_Handle->GPIO_Config.GPIO_PinNumber));
	pGPIO_Handle->pGPIOx->GPIO_OSPEEDR &= ~(0x3 << (2*pGPIO_Handle->GPIO_Config.GPIO_PinNumber));
	pGPIO_Handle->pGPIOx->GPIO_OSPEEDR |= tempreg;


	// configure alternate function
	uint32_t AFRx = pGPIO_Handle->GPIO_Config.GPIO_PinNumber / 8;
	uint32_t AFRx_bit_position = (pGPIO_Handle->GPIO_Config.GPIO_PinNumber % 8) * 4;
	tempreg = (pGPIO_Handle->GPIO_Config.GPIO_AlterMode << AFRx_bit_position);

	if (AFRx == 0)
	{
		// AFRL
		pGPIO_Handle->pGPIOx->GPIO_AFRL &= ~(0xF << AFRx_bit_position);
		pGPIO_Handle->pGPIOx->GPIO_AFRL |= tempreg;
	}
	else if (AFRx == 1)
	{
		// AFRH
		pGPIO_Handle->pGPIOx->GPIO_AFRH &= ~(0xF << AFRx_bit_position);
		pGPIO_Handle->pGPIOx->GPIO_AFRH |= tempreg;
	}
}


/*************************************************************************
 * @fn				- GPIO_DeInit
 *
 * @brief			- Reset GPIO peripheral
 *
 * @param[in]		- pSPI_Handle: holds SPI instance & configurations
 *
 * @return			- none
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if (pGPIOx == GPIOA)
		RCC_GPIOA_RESET();
	else if (pGPIOx == GPIOB)
		RCC_GPIOB_RESET();
	else if (pGPIOx == GPIOC)
		RCC_GPIOC_RESET();
	else if (pGPIOx == GPIOD)
		RCC_GPIOD_RESET();
	else if (pGPIOx == GPIOE)
		RCC_GPIOE_RESET();
	else if (pGPIOx == GPIOF)
		RCC_GPIOF_RESET();
	else if (pGPIOx == GPIOG)
		RCC_GPIOG_RESET();
}

/*************************************************************************
 * @fn				- GPIO_WriteOutputPin
 *
 * @brief			- Write Output Pin as High or Low
 *
 * @param[in]		- pGPIO_Handle: holds GPIO instance & configurations
 *
 * @return			- none
 */
void GPIO_WriteOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value)
{
	if (value == GPIO_PIN_SET)
		pGPIOx->GPIO_ODR |= (1 << PinNumber);
	else if (value == GPIO_PIN_RESET)
		pGPIOx->GPIO_ODR &= ~(1 << PinNumber);
}

/*************************************************************************
 * @fn				- GPIO_ReadInputPin
 *
 * @brief			- Read Input value of a Pin
 *
 * @param[in]		- pGPIOx: holds GPIO instance
 * @param[in]		- PinNumber: holds GPIO Pin number
 *
 * @return			- input value
 */
uint8_t GPIO_ReadInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t read_value;
	read_value = (uint8_t)((pGPIOx->GPIO_IDR >> PinNumber) & 0x00000001);
	return read_value;
}

/*************************************************************************
 * @fn				- GPIO_ToggleOutputPin
 *
 * @brief			- Toggle Output Pin state
 *
 * @param[in]		- pGPIOx: holds GPIO instance
 * @param[in]		- PinNumber: holds GPIO Pin number
 *
 * @return			- none
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->GPIO_ODR ^= (1 << PinNumber);
}

/*************************************************************************
 * @fn				- GPIO_IRQInterruptConfig
 *
 * @brief			- Enable the IRQ number in the NVIC register
 *
 * @param[in]		- IRQ_Number: IRQ number of the GPIOx
 * @param[in]		- EnorDi: enable/disable
 *
 * @return			- none
 */
void GPIO_IRQInterruptConfig(uint8_t IRQ_Number, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		// Each ISER contains 32 bits = 32 interrupt enable bits
		if (IRQ_Number <= 31)
			NVIC->NVIC_ISER[0] |= (1 << IRQ_Number);		// IRQ_Number: 0 - 31
		else if (IRQ_Number >= 32 && IRQ_Number <= 63)
			NVIC->NVIC_ISER[1] |= (1 << (IRQ_Number % 32));	// IRQ_Number: 32 - 63
		else if (IRQ_Number >= 64 && IRQ_Number <= 95)
			NVIC->NVIC_ISER[2] |= (1 << (IRQ_Number % 64));	// IRQ_Number: 64 - 95
	}
	else if (EnorDi == DISABLE)
	{
		if (IRQ_Number <= 31)
			NVIC->NVIC_ICER[0] |= (1 << IRQ_Number);
		else if (IRQ_Number >= 32 && IRQ_Number <= 63)
			NVIC->NVIC_ICER[1] |= (1 << (IRQ_Number % 32));
		else if (IRQ_Number >= 64 && IRQ_Number <= 95)
			NVIC->NVIC_ICER[2] |= (1 << (IRQ_Number % 64));
	}
}

/*************************************************************************
 * @fn				- GPIO_IRQPriorityConfig
 *
 * @brief			- Configure the priority in the NVIC peripheral
 *
 * @param[in]		- IRQ_Number: IRQ number of the GPIO instance
 * @param[in]		- Priority: IRQ priority level.
 *
 * @return			- none
 */
void GPIO_IRQPriorityConfig(uint8_t IRQ_Number, uint32_t IRQ_Priority)
{
	uint8_t NVIC_IPRx;
	uint8_t NVIC_IPRx_field;
	uint32_t shift_amount;

	// find out which IPR holds the IRQ number
	NVIC_IPRx = IRQ_Number / 4;

	// find out which field of the IPR holds the priority number
	NVIC_IPRx_field = IRQ_Number % 4;

	// calculate shift amount
	shift_amount = (NVIC_IPRx_field * 8) + 4;	// first 4 bits of a field are ignored, so add 4

	// set the priority in the corresponding IPR
	NVIC->NVIC_IPR[NVIC_IPRx] |= (IRQ_Priority << shift_amount);

}

/*************************************************************************
 * @fn				- GPIO_IRQHandler
 *
 * @brief			- Clear the pending register manually for every interrupt event.
 * 					- If we don't call it, interrupt will be triggered forever.
 *
 * @param[in]		- IRQ_Number: IRQ number of the GPIO instance
 * @param[in]		- Priority: IRQ priority level.
 *
 * @return			- none
 */
void GPIO_IRQHandler(uint8_t PinNumber)
{
	// This bit is set when a event arrives on the EXTI line. Must clear it manually.
	if (EXTI->EXTI_PR & (1 << PinNumber))
	{
		EXTI->EXTI_PR |= (1 << PinNumber);
	}
}
