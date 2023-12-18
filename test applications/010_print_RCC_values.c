/*
 * 010_print_RCC_values.c
 *
 *  Created on: Dec 10, 2023
 *      Author: gia nguyen
 */

#include "stm32f407xx.h"


RCC_OscInit_t 	osc_init;
RCC_ClkInit_t 	clk_init;
GPIO_Handle_t  	button_gpio;
GPIO_Handle_t  	USART2Gpios;
USART_Handle_t 	USART2Handle;

void USART2_gpios_init();
void USART2_init();
void Button_init();
void delay();
void SystemClkConfig();

char sclk[20];
char hclk[20];
char pclk1[20];
char pclk2[20];

int main()
{
	SystemClkConfig();

	Button_init();
	USART2_gpios_init();
	USART2_init();


	sprintf(sclk, "Sysclk: %d\r\n", (int)RCC_Get_SYSCLK_Value());
	sprintf(hclk, "Hclk:   %d\r\n", (int)RCC_Get_HCLK_Value());
	sprintf(pclk1, "Pclk1:  %d\r\n", (int)RCC_Get_PCLK1_Value());
	sprintf(pclk2, "Pclk2:  %d\r\n", (int)RCC_Get_PCLK2_Value());

	USART_Start(USART2Handle.pUSARTx);

	while(1)
	{
		while (!GPIO_ReadInputPin(GPIOA, GPIO_PIN_0));
		delay();

		USART_SendData(&USART2Handle, (uint8_t*)sclk, strlen(sclk));
		USART_SendData(&USART2Handle, (uint8_t*)hclk, strlen(hclk));
		USART_SendData(&USART2Handle, (uint8_t*)pclk1, strlen(pclk1));
		USART_SendData(&USART2Handle, (uint8_t*)pclk2, strlen(pclk2));
	}
	return 0;
}


void SystemClkConfig()
{
	osc_init.OscType = RCC_OSC_TYPE_HSE;
	//osc_init.HSEState = RCC_OSC_HSE_ON;
	RCC_OscInit(&osc_init);

	clk_init.SysclkSource = RCC_SYSCLK_SOURCE_HSE;
	clk_init.AHBClockDivider = RCC_SYSCLK_DIV1;
	clk_init.APB1ClockDivider = RCC_AHB_DIV1;
	clk_init.APB2ClockDivider = RCC_AHB_DIV1;

	// Find a way to fix this
	clk_init.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInit(&clk_init);
}

void USART2_gpios_init()
{
	USART2Gpios.pGPIOx = GPIOA;
	USART2Gpios.GPIO_Config.GPIO_Mode = GPIO_MODE_ALT_FUN;
	USART2Gpios.GPIO_Config.GPIO_AlterMode = AF7_USART2;
	USART2Gpios.GPIO_Config.GPIO_OutputSpeed = GPIO_OPSPEED_HIGH;
	USART2Gpios.GPIO_Config.GPIO_OutputType = GPIO_OPTYPE_PP;
	USART2Gpios.GPIO_Config.GPIO_PUPD = GPIO_PULLUP;

	USART2Gpios.GPIO_Config.GPIO_PinNumber = GPIO_PIN_2;
	GPIO_Init(&USART2Gpios);
	USART2Gpios.GPIO_Config.GPIO_PinNumber = GPIO_PIN_3;
	GPIO_Init(&USART2Gpios);
}

void USART2_init()
{
	USART2Handle.pUSARTx = USART2;
	USART2Handle.USARTConfig.USART_Baudrate = USART_BAUD_9600;
	USART2Handle.USARTConfig.USART_HWFlowControl = USART_HWFLOW_DISABLE;
	USART2Handle.USARTConfig.USART_Mode = USART_MODE_TX;
	USART2Handle.USARTConfig.USART_NumOfStopBits = USART_STOP_1_BIT;
	USART2Handle.USARTConfig.USART_OverSampling = USART_OVERSAMPLING_16;
	USART2Handle.USARTConfig.USART_ParityControl = USART_PARITY_DISABLE;
	USART2Handle.USARTConfig.USART_WordLength = USART_WORDLEN_8BITS;

	USART_Init(&USART2Handle);
}

void Button_init()
{
	button_gpio.pGPIOx = GPIOA;
	button_gpio.GPIO_Config.GPIO_PinNumber = GPIO_PIN_0;
	button_gpio.GPIO_Config.GPIO_Mode = GPIO_MODE_INPUT;
	button_gpio.GPIO_Config.GPIO_OutputSpeed = GPIO_OPSPEED_HIGH;
	button_gpio.GPIO_Config.GPIO_PUPD = GPIO_NOPULL;

	GPIO_Init(&button_gpio);
}

void delay()
{
	for (uint32_t i = 0; i <= 250000; i++);
}
