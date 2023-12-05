/*
 * stm32f407xx_spi.h
 *
 *  Created on: Nov 27, 2023
 *      Author: gia nguyen
 */

#ifndef INC_STM32F407XX_SPI_H_
#define INC_STM32F407XX_SPI_H_

#include "stm32f407xx.h"

/* SPI Configuration structure */
typedef struct
{
	uint8_t SPI_Mode;		// Mode: master/slave
	uint8_t SPI_CPOL;		// Clock polarity
	uint8_t SPI_CPHA;		// Clock phase
	uint8_t SPI_DFF;		// Data frame format: 8/16bit
	uint8_t SPI_BusConfig;	// Bus configuration: full-duplex/half-duplex
	uint8_t SPI_Baudrate;	// Baudrate control: f_pclk/2, f_pclk/4,...
	uint8_t SPI_SSM;		// Software Slave management
}SPI_Config_t;


/* SPI Handle structure */
typedef struct
{
	SPI_RegDef_t *pSPIx;		// SPI instance
	SPI_Config_t SPI_Config;	// User configurations
	uint8_t *pTxBuffer;			// Pointer to user's Tx data
	uint8_t *pRxBuffer;			// Pointer to user's Rx data
	uint32_t TxLen;				// Tx length in bytes
	uint32_t RxLen;				// Rx length in bytes
	uint8_t TxState;			// Tx state: ready/busy
	uint8_t RxState;			// Rx state: ready/busy
}SPI_Handle_t;



/********************** APIs for SPI **************************/
void SPI_Peripheral_Clk_Ctrl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_Init(SPI_Handle_t *pSPI_Handle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len);
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPI_Handle, uint8_t *pTxBuffer, uint32_t len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPI_Handle, uint8_t *pRxBuffer, uint32_t len);

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t Flag);

void SPI_IRQInterruptConfig(uint8_t IRQ_Number, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQ_Number, uint32_t IRQ_Priority);
void SPI_IRQHandler();
void SPI_EventCallback(SPI_Handle_t *pSPI_Handle, uint8_t Event);

void SPI_Start(SPI_RegDef_t *pSPIx);
void SPI_Stop(SPI_RegDef_t *pSPIx);

void SPI_ConfigSSI(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_ConfigSSOE(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

void SPI_EventCallback(SPI_Handle_t *pSPI_Handle, uint8_t Event);

/********************** Configuration Macros ***********************/
#define SPI_MODE_SLAVE			0
#define SPI_MODE_MASTER			1

#define SPI_BUS_FULL_DUPLEX		1
#define SPI_BUS_HALF_DUPLEX		2
#define SPI_BUS_SIMPLEX_RXONLY 	3

#define SPI_DFF_8_BITS			0
#define SPI_DFF_16_BITS			1

#define SPI_BAUD_DIV_2			0
#define SPI_BAUD_DIV_4			1
#define SPI_BAUD_DIV_8			2
#define SPI_BAUD_DIV_16			3
#define SPI_BAUD_DIV_32			4
#define SPI_BAUD_DIV_64			5
#define SPI_BAUD_DIV_128		6
#define SPI_BAUD_DIV_256		7

#define SPI_CPHA_LOW			0
#define SPI_CPHA_HIGH			1
#define SPI_CPOL_LOW			0
#define SPI_CPOL_HIGH			1

#define SPI_SSM_DISABLE			0	// Hardware control
#define SPI_SSM_ENABLE			1	// Software control


/********************** Interrupt Flags ***********************/
#define SPI_TXE_FLAG			(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG			(1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG			(1 << SPI_SR_BSY)

/********************** Application states ****************************/
#define SPI_TX_READY			0
#define SPI_RX_READY			1
#define SPI_TX_BUSY				2
#define SPI_RX_BUSY				3

/********************** Possible events *******************************/
#define SPI_EVENT_TX_CMPLT		0
#define SPI_EVENT_RX_CMPLT		1
#define SPI_EVENT_OVR_ERR		2



#endif /* INC_STM32F407XX_SPI_H_ */
