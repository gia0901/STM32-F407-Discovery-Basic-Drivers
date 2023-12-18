/*
 * stm32f407xx_uart.h
 *
 *  Created on: Dec 8, 2023
 *      Author: gia nguyen
 */

#ifndef INC_STM32F407XX_UART_H_
#define INC_STM32F407XX_UART_H_

#include "stm32f407xx.h"



/*
 * USART configuration structure
 */
typedef struct
{
	uint8_t 	USART_Mode;
	uint32_t 	USART_Baudrate;
	uint8_t 	USART_WordLength;
	uint8_t 	USART_ParityControl;
	uint8_t 	USART_HWFlowControl;	// Hardware Flow Control
	uint8_t		USART_NumOfStopBits;
	uint8_t 	USART_OverSampling;
} USART_Config_t;

/*
 * USART Handle structure
 */
typedef struct
{
	USART_RegDef_t *pUSARTx;
	USART_Config_t USARTConfig;

	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxState;
	uint8_t RxState;
} USART_Handle_t;


/********************** APIs for I2C **************************/
void USART_Peripheral_Clk_Ctrl(USART_RegDef_t *pUSARTx, uint8_t EnorDi);
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint32_t FlagName);


void USART_Start(USART_RegDef_t *pUSARTx);
void USART_Stop(USART_RegDef_t *pUSARTx);

void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_RegDef_t *pUSARTx);

void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t len);
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t len);
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t len);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t len);


void USART_IRQHandler();
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint8_t Priority);
void USART_EventCallback(USART_Handle_t * pUSARTHandle, uint8_t Event);

/********************** Configuration Macros ***********************/
#define USART_MODE_TX	0
#define USART_MODE_RX	1
#define USART_MODE_TXRX	2

#define USART_BAUD_9600			9600
#define USART_BAUD_19200		19200
#define USART_BAUD_115200		115200
#define USART_BAUD_921600		921600

#define USART_PARITY_DISABLE	0
#define USART_PARITY_EVEN		1
#define USART_PARITY_ODD		2

#define USART_HWFLOW_DISABLE	0
#define USART_HWFLOW_CTS		1
#define USART_HWFLOW_RTS		2
#define USART_HWFLOW_CTS_RTS	3

#define USART_WORDLEN_8BITS		0
#define USART_WORDLEN_9BITS		1

#define USART_STOP_1_BIT		0
#define USART_STOP_1_5_BIT		1
#define USART_STOP_2_BIT		2
#define USART_STOP_2_5_BIT		3

#define USART_OVERSAMPLING_16	0
#define USART_OVERSAMPLING_8	1

/********************** Flag status  ***********************/
#define USART_FLAG_TXE			(1 << USART_SR_TXE)
#define USART_FLAG_RXNE			(1 << USART_SR_RXNE)
#define USART_FLAG_TC			(1 << USART_SR_TC)
#define USART_FLAG_PE			(1 << USART_SR_PE)
#define USART_FLAG_ORE			(1 << USART_SR_ORE)
#define USART_FLAG_CTS			(1 << USART_SR_CTS)
#define USART_FLAG_IDLE			(1 << USART_SR_IDLE)

/********************** Possible Interrupt Events ***********************/
#define USART_EVENT_TXE			0
#define USART_EVENT_RXNE		1
#define USART_EVENT_TC			2
#define USART_EVENT_CTS			3
#define USART_EVENT_ORE			4	// Overrun error detected
#define USART_EVENT_IDLE		5
#define USART_EVENT_PE			6


/********************** Application Callback states ****************************/
#define USART_TX_BUSY			0
#define USART_RX_BUSY			1
#define USART_TX_READY			2
#define USART_RX_READY			3
#define USART_TX_CMPLT			4
#define USART_RX_CMPLT			5
#define USART_OVR_ERROR			6
#define USART_CTS_EVENT			7
#define USART_IDLE_ERROR		8
#define USART_PE_ERROR			9


#endif /* INC_STM32F407XX_UART_H_ */
