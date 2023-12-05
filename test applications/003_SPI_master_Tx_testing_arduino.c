/*
 * 003_SPI_master_test.c
 *
 *  Created on: Nov 27, 2023
 *      Author: gia nguyen
 */
#include <string.h>
#include "stm32f407xx.h"

// PB12 --> SPI2_NSS
// PB13 --> SPI2_SCK
// PB14	--> SPI2_MISO
// PB15 --> SPI2_MOSI

void SPI2_Pins_Init()
{
	GPIO_Handle_t SPI_pin;

	SPI_pin.pGPIOx = GPIOB;
	SPI_pin.GPIO_Config.GPIO_Mode = GPIO_MODE_ALT_FUN;
	SPI_pin.GPIO_Config.GPIO_AlterMode = AF5_SPI2;
	SPI_pin.GPIO_Config.GPIO_OutputSpeed = GPIO_OPSPEED_HIGH;
	SPI_pin.GPIO_Config.GPIO_OutputType = GPIO_OPTYPE_PP;
	SPI_pin.GPIO_Config.GPIO_PUPD = GPIO_NOPULL;

	// SCLK
	SPI_pin.GPIO_Config.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&SPI_pin);

	// MOSI
	SPI_pin.GPIO_Config.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&SPI_pin);

	// MISO
	//SPI_pin.GPIO_Config.GPIO_PinNumber = GPIO_PIN_14;
	//GPIO_Init(&SPI_pin);

	// NSS
	SPI_pin.GPIO_Config.GPIO_PinNumber = GPIO_PIN_12;
	GPIO_Init(&SPI_pin);
}


void SPI2_Init()
{
	SPI_Handle_t SPI2_Handle;

	SPI2_Handle.pSPIx = SPI2;
	SPI2_Handle.SPI_Config.SPI_Mode = SPI_MODE_MASTER;
	SPI2_Handle.SPI_Config.SPI_BusConfig = SPI_BUS_FULL_DUPLEX;
	SPI2_Handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI2_Handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2_Handle.SPI_Config.SPI_DFF = SPI_DFF_8_BITS;
	SPI2_Handle.SPI_Config.SPI_SSM = SPI_SSM_DISABLE;
	SPI2_Handle.SPI_Config.SPI_Baudrate = SPI_BAUD_DIV_32;	// generate SCLK = 8 Mhz

	SPI_Init(&SPI2_Handle);
}


void Button_init()
{
	GPIO_Handle_t button_gpio;

	button_gpio.pGPIOx = GPIOA;
	button_gpio.GPIO_Config.GPIO_PinNumber = GPIO_PIN_0;
	button_gpio.GPIO_Config.GPIO_Mode = GPIO_MODE_INPUT;
	button_gpio.GPIO_Config.GPIO_OutputSpeed = GPIO_OPSPEED_HIGH;
	button_gpio.GPIO_Config.GPIO_PUPD = GPIO_NOPULL;

	GPIO_Init(&button_gpio);

}


void delay(void)
{
	for(uint32_t i = 0 ; i < 500000 ; i ++);
}

int main()
{
	char TxBuf[] = "Hello Arduino UNO R3. This is STM32F407 DISC";

	Button_init();

	SPI2_Pins_Init();
	SPI2_Init();

	//SPI_ConfigSSI(SPI2, ENABLE);
	SPI_ConfigSSOE(SPI2, ENABLE);

	while(1)
	{
		while( !GPIO_ReadInputPin(GPIOA, GPIO_PIN_0));

		delay();

		SPI_Start(SPI2);

		uint8_t data_len = strlen(TxBuf);

		SPI_SendData(SPI2, &data_len, 1);

		SPI_SendData(SPI2, (uint8_t*)TxBuf, strlen(TxBuf));

		while (SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

		SPI_Stop(SPI2);
	}
	return 0;
}

