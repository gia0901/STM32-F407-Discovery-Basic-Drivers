#include <stdio.h>
#include <string.h>

#include "stm32f407xx.h"


GPIO_Handle_t LED_Pin;
GPIO_Handle_t Button_Pin;


void LED_init()
{
	LED_Pin.pGPIOx = GPIOD;
	LED_Pin.GPIO_Config.GPIO_Mode = GPIO_MODE_OUTPUT;
	LED_Pin.GPIO_Config.GPIO_PinNumber = GPIO_PIN_12;
	LED_Pin.GPIO_Config.GPIO_OutputSpeed = GPIO_OPSPEED_LOW;
	LED_Pin.GPIO_Config.GPIO_OutputType = GPIO_OPTYPE_PP;

	GPIO_Init(&LED_Pin);
	LED_Pin.GPIO_Config.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&LED_Pin);
	LED_Pin.GPIO_Config.GPIO_PinNumber = GPIO_PIN_14;
	GPIO_Init(&LED_Pin);
	LED_Pin.GPIO_Config.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&LED_Pin);
}


void Button_init()
{
	Button_Pin.pGPIOx = GPIOA;
	Button_Pin.GPIO_Config.GPIO_PinNumber = GPIO_PIN_0;
	Button_Pin.GPIO_Config.GPIO_Mode = GPIO_MODE_IT_FT;
	Button_Pin.GPIO_Config.GPIO_PUPD = GPIO_PULLUP;

	GPIO_Init(&Button_Pin);
}

void delay()
{
	for (uint32_t i = 0; i <= 250000; i++);
}

int main(void)
{
	LED_init();
	Button_init();

	while (1)
	{

		delay();
	}
}
