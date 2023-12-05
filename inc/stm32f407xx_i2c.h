/*
 * stm32f407xx_i2c.h
 *
 *  Created on: Nov 30, 2023
 *      Author: gia nguyen
 */

#ifndef INC_STM32F407XX_I2C_H_
#define INC_STM32F407XX_I2C_H_

#include "stm32f407xx.h"


/* I2C Configuration Structure */

typedef struct
{
	uint32_t I2C_SclSpeed;
	uint8_t I2C_AckControl;
	uint8_t I2C_SlaveAddr;			// used when MCU is a slave
	uint8_t I2C_FastModeDutyCycle;	// only when using fast mode

} I2C_Config_t;


/* I2C Handle Structure */

typedef struct
{
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;

	// For interrupt APIs
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t RxTotalLen;	// Determine that total byte is 1 byte or more
	uint8_t TxRxState;	// Half-duplex uses 1 data line only -> one state for both Tx & Rx events.
	uint8_t SlaveAddr;
	uint8_t Sr;			// Repeated start check
} I2C_Handle_t;



/********************** APIs for I2C **************************/

void I2C_Peripheral_Clk_Ctrl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
void I2C_Init(I2C_Handle_t *pI2C_Handle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);
void I2C_Start(I2C_RegDef_t *pI2Cx);
void I2C_Stop(I2C_RegDef_t *pI2Cx);

/* ACK control */
void I2C_AckControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/* Master Send & receive data APIs */
void I2C_MasterSend(I2C_Handle_t *pI2C_Handle, uint8_t *pTxBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t Sr);
void I2C_MasterReceive(I2C_Handle_t *pI2C_Handle, uint8_t *pRxBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t Sr);
uint8_t I2C_MasterSendIT(I2C_Handle_t *pI2C_Handle, uint8_t *pTxBuffer, uint32_t len,  uint8_t SlaveAddr, uint8_t Sr);
uint8_t I2C_MasterReceiveIT(I2C_Handle_t *pI2C_Handle, uint8_t *pRxBuffer, uint32_t len,  uint8_t SlaveAddr, uint8_t Sr);

/* IRQ */
void I2C_IRQInterruptConfig(uint8_t IRQ_Number, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQ_Number, uint8_t Priority);
void I2C_Event_IRQHandler(I2C_Handle_t *pI2C_Handle);
void I2C_Error_IRQHandler(I2C_Handle_t *pI2C_Handle);

void I2C_EventCallback(I2C_Handle_t *pI2C_Handle, uint8_t Event);


/********************** Configuration Macros ***********************/

#define I2C_SCL_SPEED_STANDARD		100000	// 100 Khz
#define I2C_SCL_SPEED_FAST_400K		400000	// 400 Khz
#define I2C_SCL_SPEED_FAST_200K		200000	// 200 Khz

#define I2C_ACK_ENABLE				ENABLE
#define I2C_ACK_DISABLE				DISABLE

#define I2C_FAST_DUTY_CYCLE_2		0
#define I2C_FAST_DUTY_CYCLE_16_9	1

#define I2C_REPEATED_START_ENABLE	1
#define I2C_REPEATED_START_DISABLE	0

/**********************  Flag status  ***********************/

#define I2C_FLAG_SB		(1 << I2C_SR1_SB)
#define I2C_FLAG_TXE	(1 << I2C_SR1_TxE)
#define I2C_FLAG_RXNE	(1 << I2C_SR1_RxNE)
#define I2C_FLAG_BTF	(1 << I2C_SR1_BTF)
#define I2C_FLAG_ADDR	(1 << I2C_SR1_ADDR)
#define I2C_FLAG_OVR	(1 << I2C_SR1_OVR)


/********************** Possible Interrupt Events ***********************/

#define I2C_EVENT_TX_CMPLT	1
#define I2C_EVENT_RX_CMPLT	2
#define I2C_EVENT_STOPF		3

#define I2C_ERROR_BERR		4
#define I2C_ERROR_ARLO		5
#define I2C_ERROR_AF		6
#define I2C_ERROR_OVR		7
#define I2C_ERROR_TIMEOUT	8

/********************** Application states ****************************/

#define I2C_READY		1
#define I2C_TX_BUSY		2
#define I2C_RX_BUSY		3



#endif /* INC_STM32F407XX_I2C_H_ */
