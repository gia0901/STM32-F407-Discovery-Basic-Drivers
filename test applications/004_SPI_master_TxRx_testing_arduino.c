/*
 * 004_SPI_master_TxRx_testing_arduino.c
 *
 *  Created on: Nov 29, 2023
 *      Author: gia nguyen
 */

#include "stm32f407xx.h"

// command codes that are recognized by the Arduino slave
#define CMD_LED_CTRL		0x50
#define CMD_LED_REQ			0x51	// Require slave to send a led pin to control


// ACK & NACK status sent by the Arduino
#define ACK					0xF5
#define NACK				0xA5

// Arduino LED control
#define LED_PIN				8
#define LED_ON				1
#define LED_OFF				0

// Function prototypes
void Button_Init();
void SPI2_PinInit();
void SPI2_Init();
void LED_Init();
void delay();


int main()
{
	Button_Init();
	LED_Init();
	SPI2_PinInit();
	SPI2_Init();
	SPI_ConfigSSOE(SPI2, ENABLE);

	uint8_t command_code;
	uint8_t commands[2];
	uint8_t dummy_write = 0xF;
	uint8_t dummy_read;
	uint8_t ACK_status;
	uint8_t led_req[2];

	while(1)
	{

		SPI_Start(SPI2);

		// 1. Send 1st command to control LED of the arduino
		while ( !GPIO_ReadInputPin(GPIOA, GPIO_PIN_0));
		delay();

		command_code = CMD_LED_CTRL;

		SPI_SendData(SPI2, &command_code, 1);
		SPI_ReceiveData(SPI2, &dummy_read, 1);	// do dummy read to clear the RXNE flag


		SPI_SendData(SPI2, &dummy_write, 1);	// do dummy write to fetch the response from the Arduino
		SPI_ReceiveData(SPI2, &ACK_status, 1);

		// check if the response is ACK, send the command
		if (ACK_status == ACK)
		{
			commands[0] = LED_PIN;
			commands[1] = LED_ON;
			SPI_SendData(SPI2, commands, 2);
			SPI_ReceiveData(SPI2, commands, 2);	// do dummy read to clear the RXNE flag
			//SPI_ReceiveData(SPI2, &dummy_read, 1);
		}

		// 2. Request the response from Arduino then control the corresponding LED on the board
		while ( !GPIO_ReadInputPin(GPIOA, GPIO_PIN_0));
		delay();

		command_code = CMD_LED_REQ;

		// Send command code & do dummy read
		SPI_SendData(SPI2, &command_code, 1);
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		// Do dummy write then fetch the response from slave
		SPI_SendData(SPI2, &dummy_write, 1);
		SPI_ReceiveData(SPI2, &ACK_status, 1);

		if (ACK_status == ACK)
		{
			delay();
			for (uint8_t i = 0; i < 2; i++)
			{
				SPI_SendData(SPI2, &dummy_write, 1);
				SPI_ReceiveData(SPI2, &led_req[i], 1);
			}
			// Control the LED of the board
			GPIO_WriteOutputPin(GPIOD, led_req[0], led_req[1]);
		}

		while (SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG) == FLAG_SET);

		SPI_Stop(SPI2);
	}

	return 0;
}

void Button_Init()
{
	GPIO_Handle_t button_gpio;

	button_gpio.pGPIOx = GPIOA;
	button_gpio.GPIO_Config.GPIO_PinNumber = GPIO_PIN_0;
	button_gpio.GPIO_Config.GPIO_Mode = GPIO_MODE_INPUT;
	button_gpio.GPIO_Config.GPIO_OutputSpeed = GPIO_OPSPEED_HIGH;
	button_gpio.GPIO_Config.GPIO_PUPD = GPIO_NOPULL;

	GPIO_Init(&button_gpio);

}

void SPI2_PinInit()
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
	SPI_pin.GPIO_Config.GPIO_PinNumber = GPIO_PIN_14;
	GPIO_Init(&SPI_pin);

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

void LED_Init()
{
	GPIO_Handle_t LED_Pin;

	LED_Pin.pGPIOx = GPIOD;
	LED_Pin.GPIO_Config.GPIO_Mode = GPIO_MODE_OUTPUT;
	LED_Pin.GPIO_Config.GPIO_OutputSpeed = GPIO_OPSPEED_LOW;
	LED_Pin.GPIO_Config.GPIO_OutputType = GPIO_OPTYPE_PP;

	LED_Pin.GPIO_Config.GPIO_PinNumber = GPIO_PIN_12;
	GPIO_Init(&LED_Pin);
	LED_Pin.GPIO_Config.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&LED_Pin);
	LED_Pin.GPIO_Config.GPIO_PinNumber = GPIO_PIN_14;
	GPIO_Init(&LED_Pin);
	LED_Pin.GPIO_Config.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&LED_Pin);
}

void delay()
{
	for(uint32_t i = 0 ; i < 500000 ; i ++);
}
