/*
 * 006_I2C_Master_send_arduino.c
 *
 *  Created on: Nov 30, 2023
 *      Author: gia nguyen
 */

#include "stm32f407xx.h"


GPIO_Handle_t button_gpio;
GPIO_Handle_t i2c1_pins;
I2C_Handle_t i2c1_handle;

// Function prototypes
void Button_Init();
void I2C1_PinInit();
void I2C1_Init();
void delay();

#define SLAVEADDR	((uint8_t)0x68)

uint8_t send_buf[] = "Hello Arduino, how's your doing.";

int main()
{
	Button_Init();
	I2C1_PinInit();
	I2C1_Init();

	I2C_Start(I2C1);

	while(1)
	{
		while (! GPIO_ReadInputPin(GPIOA, GPIO_PIN_0));
		delay();


		I2C_MasterSend(&i2c1_handle, send_buf, sizeof(send_buf), SLAVEADDR, DISABLE);

		I2C_Stop(I2C1);
	}

	return 0;
}

void I2C1_PinInit()
{
	i2c1_pins.pGPIOx = GPIOB;
	i2c1_pins.GPIO_Config.GPIO_Mode = GPIO_MODE_ALT_FUN;
	i2c1_pins.GPIO_Config.GPIO_AlterMode = AF4_I2C1;
	i2c1_pins.GPIO_Config.GPIO_OutputType = GPIO_OPTYPE_OD;
	i2c1_pins.GPIO_Config.GPIO_OutputSpeed = GPIO_OPSPEED_HIGH;
	i2c1_pins.GPIO_Config.GPIO_PUPD = GPIO_PULLUP;

	// I2C1_SCL	- PB8
	i2c1_pins.GPIO_Config.GPIO_PinNumber = GPIO_PIN_6;
	GPIO_Init(&i2c1_pins);

	// I2C1_SDA - PB9
	i2c1_pins.GPIO_Config.GPIO_PinNumber = GPIO_PIN_7;
	GPIO_Init(&i2c1_pins);
}

void I2C1_Init()
{
	i2c1_handle.pI2Cx = I2C1;
	i2c1_handle.I2C_Config.I2C_SclSpeed = I2C_SCL_SPEED_STANDARD;
	i2c1_handle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
	i2c1_handle.I2C_Config.I2C_SlaveAddr = 0x61;	// Random number
	I2C_Init(&i2c1_handle);
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

void delay()
{
	for(uint32_t i = 0 ; i < 500000 ; i ++);
}

