/*
 * 005_SPI_master_TxRx_interrupt_testing_arduino.c
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


GPIO_Handle_t button_gpio;
SPI_Handle_t SPI2_Handle;
GPIO_Handle_t SPI_pin;

int main()
{
	uint8_t command_code;
	uint8_t commands[2];
	uint8_t dummy_write = 0xF;
	uint8_t dummy_read;
	uint8_t ACK_status;
	uint8_t led_req[2] = {0};

	Button_Init();
	LED_Init();
	SPI2_PinInit();
	SPI2_Init();
	SPI_ConfigSSOE(SPI2, ENABLE);

	while(1)
	{
		// 1. Send command to Arduino
		while ( !GPIO_ReadInputPin(GPIOA, GPIO_PIN_0));
		delay();

		SPI_Start(SPI2);
		command_code = CMD_LED_CTRL;

		while (SPI_SendDataIT(&SPI2_Handle, &command_code, 1) == SPI_TX_BUSY);
		while (SPI_ReceiveDataIT(&SPI2_Handle, &dummy_read, 1) == SPI_RX_BUSY);

		while( SPI_SendDataIT(&SPI2_Handle, &dummy_write, 1) == SPI_TX_BUSY);
		while( SPI_ReceiveDataIT(&SPI2_Handle, &ACK_status, 1) == SPI_RX_BUSY);

		// check if the response is ACK, send the command
		if (ACK_status == ACK)
		{
			commands[0] = LED_PIN;
			commands[1] = LED_ON;
			while (SPI_SendDataIT(&SPI2_Handle, commands, 2) == SPI_TX_BUSY);
			while (SPI_ReceiveDataIT(&SPI2_Handle, commands, 2) == SPI_RX_BUSY);	// do dummy read to clear the RXNE flag

		}
		SPI_Stop(SPI2);

		// 2. Receive command from Arduino
		while ( !GPIO_ReadInputPin(GPIOA, GPIO_PIN_0));
		delay();
		SPI_Start(SPI2);

		//ACK_status = 0;
		command_code = CMD_LED_REQ;

		while (SPI_SendDataIT(&SPI2_Handle, &command_code, 1) == SPI_TX_BUSY);
		while (SPI_ReceiveDataIT(&SPI2_Handle, &dummy_read, 1) == SPI_RX_BUSY);

		while (SPI_SendDataIT(&SPI2_Handle, &dummy_write, 1) == SPI_TX_BUSY);
		while (SPI_ReceiveDataIT(&SPI2_Handle, &ACK_status, 1) == SPI_RX_BUSY);

		delay();

		if (ACK_status == ACK)
		{
			while (SPI_SendDataIT(&SPI2_Handle, &dummy_write, 1) == SPI_TX_BUSY);
			while (SPI_ReceiveDataIT(&SPI2_Handle, &led_req[0], 1) == SPI_RX_BUSY);

			// Control the LED of the board
			GPIO_WriteOutputPin(GPIOD, led_req[0], 1);
		}

		SPI_Stop(SPI2);

	}

	return 0;
}


// SPI IRQ Handler
void SPI2_IRQHandler()
{
	SPI_IRQHandler(&SPI2_Handle);
}



void Button_Init()
{
	button_gpio.pGPIOx = GPIOA;
	button_gpio.GPIO_Config.GPIO_PinNumber = GPIO_PIN_0;
	button_gpio.GPIO_Config.GPIO_Mode = GPIO_MODE_INPUT;
	button_gpio.GPIO_Config.GPIO_OutputSpeed = GPIO_OPSPEED_HIGH;
	button_gpio.GPIO_Config.GPIO_PUPD = GPIO_NOPULL;

	GPIO_Init(&button_gpio);

}

void SPI2_PinInit()
{

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

	SPI2_Handle.pSPIx = SPI2;
	SPI2_Handle.SPI_Config.SPI_Mode = SPI_MODE_MASTER;
	SPI2_Handle.SPI_Config.SPI_BusConfig = SPI_BUS_FULL_DUPLEX;
	SPI2_Handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI2_Handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2_Handle.SPI_Config.SPI_DFF = SPI_DFF_8_BITS;
	SPI2_Handle.SPI_Config.SPI_SSM = SPI_SSM_DISABLE;
	SPI2_Handle.SPI_Config.SPI_Baudrate = SPI_BAUD_DIV_32;	// generate SCLK = 8 Mhz

	SPI2_Handle.TxState = SPI_TX_READY;
	SPI2_Handle.RxState = SPI_RX_READY;


	SPI_Init(&SPI2_Handle);\

	SPI_IRQInterruptConfig(IRQ_NUM_SPI2, ENABLE);
	SPI_IRQPriorityConfig(IRQ_NUM_SPI2, IRQ_PRIORITY_10);
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


