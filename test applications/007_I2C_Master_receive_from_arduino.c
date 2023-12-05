/*
 * 007_I2C_Master_receive_from_arduino.c
 *
 *  Created on: Dec 3, 2023
 *      Author: gia nguyen
 */

#include "stm32f407xx.h"


// Function prototypes
void Button_Init();
void I2C1_PinInit();
void I2C1_Init();
void delay();

#define SLAVEADDR	0x68

// Global variables
GPIO_Handle_t button_gpio;
GPIO_Handle_t i2c1_pins;
I2C_Handle_t i2c1_handle;

uint8_t data_len;
uint8_t recv_buff[100];
uint8_t command_req_len = 0x51;
uint8_t command_req_data = 0x52;

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

		/* Master sends command 0x51 to request slave to send the data length */
		I2C_MasterSend(&i2c1_handle, &command_req_len, 1, SLAVEADDR, I2C_REPEATED_START_ENABLE);

		/* Master receives 1 byte of length from slave */
		I2C_MasterReceive(&i2c1_handle, &data_len, 1, SLAVEADDR, I2C_REPEATED_START_ENABLE);

		/* Master sends command 0x52 to request slave to send all the data */
		I2C_MasterSend(&i2c1_handle, &command_req_data, 1, SLAVEADDR, I2C_REPEATED_START_ENABLE);

		/* Master receives the data */
		I2C_MasterReceive(&i2c1_handle, recv_buff, data_len, SLAVEADDR, I2C_REPEATED_START_DISABLE);

	}
	I2C_Stop(I2C1);

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





