/*
 * 009_uart_tx_testing_IT.c
 *
 *  Created on: Dec 10, 2023
 *      Author: gia nguyen
 */

#include "stm32f407xx.h"

// PA2 - USART2_TX
// PA3 - USART2_RX

GPIO_Handle_t  button_gpio;
GPIO_Handle_t  USART2Gpios;
USART_Handle_t USART2Handle = {.TxState = USART_TX_READY};

void USART2_gpios_init();
void USART2_init();
void Button_init();
void delay();


char data_send[1024] = "Hello. This is USART Tx Interrupt testing\r\n";

int main()
{
	Button_init();
	USART2_gpios_init();
	USART2_init();

	USART_IRQInterruptConfig(IRQ_NUM_USART2, ENABLE);
	USART_IRQPriorityConfig(IRQ_NUM_USART2, IRQ_PRIORITY_10);


	//sprintf(data_send, "sysclk: %d - hclk: %d - pclk1: %d - pclk2: %d\n",(int)RCC_Get_SYSCLK_Value(), (int)RCC_Get_HCLK_Value(), (int)RCC_Get_PCLK2_Value(), (int)RCC_Get_PCLK2_Value());

	USART_Start(USART2Handle.pUSARTx);


	while(1)
	{
		while (!GPIO_ReadInputPin(GPIOA, GPIO_PIN_0));
		delay();

		USART_SendDataIT(&USART2Handle, (uint8_t*)data_send, strlen(data_send));
		while (USART2Handle.pTxBuffer == USART_TX_BUSY);
	}
	return 0;
}

void USART2_IRQHandler()
{
	USART_IRQHandler(&USART2Handle);
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

