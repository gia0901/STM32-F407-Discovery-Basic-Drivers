#include "stm32f407xx.h"


/* Function prototypes */
void LED_init();
void Button_init();
void delay();


int main()
{
	LED_init();
	Button_init();

	while(1);

	return 0;
}


/* IRQ Handler for button (Button is PA0 -> EXTI0) */
void EXTI0_IRQHandler(void)
{
	delay();
	GPIO_IRQHandler(GPIO_PIN_0);
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_12);
}

void LED_init()
{
	GPIO_Handle_t led_gpio;
	memset(&led_gpio, 0, sizeof(led_gpio));

	led_gpio.pGPIOx = GPIOD;
	led_gpio.GPIO_Config.GPIO_Mode = GPIO_MODE_OUTPUT;
	led_gpio.GPIO_Config.GPIO_OutputSpeed = GPIO_OPSPEED_LOW;
	led_gpio.GPIO_Config.GPIO_OutputType = GPIO_OPTYPE_PP;
	led_gpio.GPIO_Config.GPIO_PinNumber = GPIO_PIN_12;

	GPIO_Init(&led_gpio);
}

void Button_init()
{
	GPIO_Handle_t button_gpio;
	memset(&button_gpio, 0, sizeof(button_gpio));

	button_gpio.pGPIOx = GPIOA;
	button_gpio.GPIO_Config.GPIO_PinNumber = GPIO_PIN_0;
	button_gpio.GPIO_Config.GPIO_Mode = GPIO_MODE_IT_RT;
	button_gpio.GPIO_Config.GPIO_PUPD = GPIO_NOPULL;

	GPIO_Init(&button_gpio);

	GPIO_IRQInterruptConfig(IRQ_NUM_EXTI0, ENABLE);
	GPIO_IRQPriorityConfig(IRQ_NUM_EXTI0, 10);
}

void delay()
{
	for (uint32_t i = 0; i < 250000; i++);
}

