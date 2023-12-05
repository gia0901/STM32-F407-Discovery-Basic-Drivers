/*
 * stm32f407xx_gpio.h
 *
 *  Created on: Nov 22, 2023
 *      Author: gia nguyen
 */

#ifndef INC_STM32F407XX_GPIO_H_
#define INC_STM32F407XX_GPIO_H_

/* GPIO Configuration structure */
typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_Mode;
	uint8_t GPIO_OutputType;
	uint8_t GPIO_OutputSpeed;
	uint8_t GPIO_PUPD;
	uint8_t GPIO_AlterMode;
}GPIO_Config_t;


/* GPIO Handle structure */
typedef struct
{
	GPIO_RegDef_t *pGPIOx;
	GPIO_Config_t GPIO_Config;
}GPIO_Handle_t;


/********************** APIs for GPIO **************************/

void GPIO_Peripheral_Clk_Ctrl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);
void GPIO_Init(GPIO_Handle_t *pGPIO_Handle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint8_t GPIO_ReadInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

// IRQ Config & IRQ Handlers
void GPIO_IRQInterruptConfig(uint8_t IRQ_Number, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQ_Number, uint32_t IRQ_Priority);
void GPIO_IRQHandler(uint8_t PinNumber);

/********************** Configuration Macros ***********************/
// GPIO_PinNumber
#define GPIO_PIN_0			0
#define GPIO_PIN_1			1
#define GPIO_PIN_2			2
#define GPIO_PIN_3			3
#define GPIO_PIN_4			4
#define GPIO_PIN_5			5
#define GPIO_PIN_6			6
#define GPIO_PIN_7			7
#define GPIO_PIN_8			8
#define GPIO_PIN_9			9
#define GPIO_PIN_10			10
#define GPIO_PIN_11			11
#define GPIO_PIN_12			12
#define GPIO_PIN_13			13
#define GPIO_PIN_14			14
#define GPIO_PIN_15			15
#define GPIO_PIN_16			16

// GPIO_Mode
#define GPIO_MODE_INPUT		0
#define GPIO_MODE_OUTPUT	1
#define GPIO_MODE_ALT_FUN	2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4		// interrupt falling-edge trigger
#define GPIO_MODE_IT_RT		5		// interrupt rising-edge trigger
#define GPIO_MODE_IT_FR		6		// interrupt falling-rising-edge trigger

// GPIO_OutputType
#define GPIO_OPTYPE_PP		0		// push-pull
#define GPIO_OPTYPE_OD		1		// open-drain

// GPIO_OutputSpeed
#define GPIO_OPSPEED_LOW	0		// output low speed
#define GPIO_OPSPEED_MED	1		// output medium speed
#define GPIO_OPSPEED_HIGH	2		// output high speed
#define GPIO_OPSPEED_VERYHIGH	3	// output very high speed

// GPIO_PUPD
#define GPIO_NOPULL			0		// no pull up, pull down
#define GPIO_PULLUP			1		// pull up
#define GPIO_PULLDOWN		2		// pull down

// GPIO Pin state
#define GPIO_PIN_SET		ENABLE
#define GPIO_PIN_RESET		DISABLE

#endif /* INC_STM32F407XX_GPIO_H_ */
