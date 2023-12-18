/*
 * stm32f407xx_uart.c
 *
 *  Created on: Dec 8, 2023
 *      Author: gia nguyen
 */

#include "stm32f407xx_uart.h"


/**************** Private helper functions of this driver *****************/
static void USART_CloseSendData(USART_Handle_t *pUSARTHandle);
static void USART_CloseReceiveData(USART_Handle_t *pUSARTHandle);
static void USART_TxITHandler(USART_Handle_t *pUSARTHandle);
static void USART_RxITHandler(USART_Handle_t *pUSARTHandle);
static void USART_SetBaudRate(USART_Handle_t *pUSARTHandle);


void USART_Start(USART_RegDef_t *pUSARTx)
{
	pUSARTx->USART_CR1 |= (1 << USART_CR1_UE);
}

void USART_Stop(USART_RegDef_t *pUSARTx)
{
	pUSARTx->USART_CR1 &= ~(1 << USART_CR1_UE);
}


void USART_Init(USART_Handle_t *pUSARTHandle)
{
	uint32_t tempreg = 0;

	USART_Peripheral_Clk_Ctrl(pUSARTHandle->pUSARTx, ENABLE);

	/* Configure word length */
	if (pUSARTHandle->USARTConfig.USART_WordLength == USART_WORDLEN_8BITS)
	{
		pUSARTHandle->pUSARTx->USART_CR1 &= ~(1 << USART_CR1_M);
	}
	else if (pUSARTHandle->USARTConfig.USART_WordLength == USART_WORDLEN_9BITS)
	{
		pUSARTHandle->pUSARTx->USART_CR1 |= (1 << USART_CR1_M);
	}

	/* Configure number of stop bits */
	pUSARTHandle->pUSARTx->USART_CR2 &= ~(0x3 << USART_CR2_STOP);
	tempreg = (pUSARTHandle->USARTConfig.USART_NumOfStopBits << USART_CR2_STOP);
	pUSARTHandle->pUSARTx->USART_CR2 |= tempreg;

	/* Configure parity control */
	if (pUSARTHandle->USARTConfig.USART_ParityControl == USART_PARITY_EVEN)
	{
		pUSARTHandle->pUSARTx->USART_CR1 |= (1 << USART_CR1_PCE);
		pUSARTHandle->pUSARTx->USART_CR1 &= ~(1 << USART_CR1_PS);
	}
	else if (pUSARTHandle->USARTConfig.USART_ParityControl == USART_PARITY_ODD)
	{
		pUSARTHandle->pUSARTx->USART_CR1 |= (1 << USART_CR1_PCE);
		pUSARTHandle->pUSARTx->USART_CR1 |= (1 << USART_CR1_PS);
	}

	/* Configure Hardware flow control */
	if (pUSARTHandle->USARTConfig.USART_HWFlowControl == USART_HWFLOW_CTS)
	{
		pUSARTHandle->pUSARTx->USART_CR3 |= (1 << USART_CR3_CTSE);
		pUSARTHandle->pUSARTx->USART_CR3 &= ~(1 << USART_CR3_RTSE);
	}
	else if (pUSARTHandle->USARTConfig.USART_HWFlowControl == USART_HWFLOW_RTS)
	{
		pUSARTHandle->pUSARTx->USART_CR3 |= (1 << USART_CR3_RTSE);
		pUSARTHandle->pUSARTx->USART_CR3 &= ~(1 << USART_CR3_CTSE);
	}
	else if (pUSARTHandle->USARTConfig.USART_HWFlowControl == USART_HWFLOW_CTS_RTS)
	{
		pUSARTHandle->pUSARTx->USART_CR3 |= (1 << USART_CR3_CTSE);
		pUSARTHandle->pUSARTx->USART_CR3 |= (1 << USART_CR3_RTSE);
	}

	/* Configure oversampling */
	if (pUSARTHandle->USARTConfig.USART_OverSampling == USART_OVERSAMPLING_16)
	{
		pUSARTHandle->pUSARTx->USART_CR1 &= ~(1 << USART_CR1_OVER8);
	}
	else if (pUSARTHandle->USARTConfig.USART_OverSampling == USART_OVERSAMPLING_8)
	{
		pUSARTHandle->pUSARTx->USART_CR1 |= (1 << USART_CR1_OVER8);
	}

	/* Configure baudrate */
	USART_SetBaudRate(pUSARTHandle);

	/* Configure the mode: Tx, Rx or both */
	if (pUSARTHandle->USARTConfig.USART_Mode == USART_MODE_TX)
	{
		pUSARTHandle->pUSARTx->USART_CR1 |= (1 << USART_CR1_TE);
		pUSARTHandle->pUSARTx->USART_CR1 &= ~(1 << USART_CR1_RE);
	}
	else if (pUSARTHandle->USARTConfig.USART_Mode == USART_MODE_RX)
	{
		pUSARTHandle->pUSARTx->USART_CR1 |= (1 << USART_CR1_RE);
		pUSARTHandle->pUSARTx->USART_CR1 &= ~(1 << USART_CR1_TE);
	}
	else if (pUSARTHandle->USARTConfig.USART_Mode == USART_MODE_TXRX)
	{
		pUSARTHandle->pUSARTx->USART_CR1 |= (1 << USART_CR1_TE);
		pUSARTHandle->pUSARTx->USART_CR1 |= (1 << USART_CR1_RE);
	}

}

void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t len)
{
	uint16_t *pTxBuffer_16;
	uint32_t i;

	for (i = 0; i < len; i++)
	{
		while (USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TXE) == FLAG_NOT_SET);

		/* If word length is 8 bits */
		if (pUSARTHandle->USARTConfig.USART_WordLength == USART_WORDLEN_8BITS)
		{
			pUSARTHandle->pUSARTx->USART_DR = *pTxBuffer;
			pTxBuffer++;
		}
		/* If word length is 9bits */
		else if (pUSARTHandle->USARTConfig.USART_WordLength == USART_WORDLEN_9BITS)
		{
			pTxBuffer_16 = (uint16_t *)pTxBuffer;
			*pTxBuffer_16 &= (uint16_t)0x1FF;
			pUSARTHandle->pUSARTx->USART_DR = *pTxBuffer_16;

			/* If parity is used -> 9 bits of data -> Buffer increases 2 times */
			if (pUSARTHandle->USARTConfig.USART_ParityControl == USART_PARITY_DISABLE)
			{
				pTxBuffer++;
				pTxBuffer++;
			}
			/* If parity isn't used -> 8 bits of data -> Buffer increases 1 time */
			else
			{
				pTxBuffer++;
			}
		}

	}

	/* Wait until Transmission complete flag is set */
	while (USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TC) == FLAG_NOT_SET);
}

void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t len)
{
	uint32_t i;

	for (i = 0; i < len; i++)
	{
		while (USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_RXNE) == FLAG_NOT_SET);

		if (pUSARTHandle->USARTConfig.USART_WordLength == USART_WORDLEN_8BITS)
		{
			/* If parity isn't used -> 8 bits of data */
			if (pUSARTHandle->USARTConfig.USART_ParityControl == USART_PARITY_DISABLE)
			{
				*pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->USART_DR & (uint8_t)0xFF);
				pRxBuffer++;
			}
			/* If parity is used -> 7 bits of data & 1 bit of parity */
			else
			{
				*pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->USART_DR & (uint8_t)0x7F);
				pRxBuffer++;
			}
		}
		else if (pUSARTHandle->USARTConfig.USART_WordLength == USART_WORDLEN_9BITS)
		{
			/* Parity is not used */
			if (pUSARTHandle->USARTConfig.USART_ParityControl == USART_PARITY_DISABLE)
			{
				/* Get 9 bits of data */
				*((uint16_t*)pRxBuffer) = (uint16_t)(pUSARTHandle->pUSARTx->USART_DR & (uint16_t)0x1FF);

				pRxBuffer++;
				pRxBuffer++;
			}
			/* Parity is used -> 8 bit data + 1 parity bit */
			else
			{
				/* Get 8 bits of data */
				*pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->USART_DR & (uint8_t)0xFF);
				pRxBuffer++;
			}
		}
	}
}


uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t len)
{
	uint8_t TxState = pUSARTHandle->TxState;

	if (TxState != USART_TX_BUSY)
	{
		/* Update data information to the handle variable */
		pUSARTHandle->TxState = USART_TX_BUSY;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxLen = len;

		/* Enable interrupts for Transmission */
		pUSARTHandle->pUSARTx->USART_CR1 |= (1 << USART_CR1_TXEIE);
		pUSARTHandle->pUSARTx->USART_CR1 |= (1 << USART_CR1_TCIE);
	}

	return TxState;
}


uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t len)
{
	uint8_t RxState = pUSARTHandle->RxState;

	if (RxState != USART_RX_BUSY)
	{
		/* Update data information to the handle variable */
		pUSARTHandle->RxState = USART_RX_BUSY;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->RxLen = len;

		/* Enable interrupts for Transmission */
		pUSARTHandle->pUSARTx->USART_CR1 |= (1 << USART_CR1_RXNEIE);
	}
	return RxState;
}


void USART_IRQHandler(USART_Handle_t *pUSARTHandle)
{
	uint32_t check_it_bit, check_event;


	/* Handle TXE Event */
	check_event = (pUSARTHandle->pUSARTx->USART_SR & USART_FLAG_TXE);
	check_it_bit = (pUSARTHandle->pUSARTx->USART_CR1 & (1 << USART_CR1_TXEIE));

	if (check_event && check_it_bit)
	{
		if (pUSARTHandle->TxState == USART_TX_BUSY)
		{
			USART_TxITHandler(pUSARTHandle);
		}
	}

	/* Handle RXNE Event */
	check_event = (pUSARTHandle->pUSARTx->USART_SR & USART_FLAG_RXNE);
	check_it_bit = (pUSARTHandle->pUSARTx->USART_CR1 & (1 << USART_CR1_RXNEIE));

	if (check_event && check_it_bit)
	{
		if (pUSARTHandle->TxState == USART_RX_BUSY)
		{
			USART_RxITHandler(pUSARTHandle);
		}
	}

	/* Handle TC (transmission complete) Event */
	check_event = (pUSARTHandle->pUSARTx->USART_SR & USART_FLAG_TC);
	check_it_bit = (pUSARTHandle->pUSARTx->USART_CR1 & (1 << USART_CR1_TCIE));

	if (check_event && check_it_bit)
	{
		/* Transmission has completed, we should close the communication */
		if (pUSARTHandle->TxState == USART_TX_BUSY)
		{
			USART_CloseSendData(pUSARTHandle);
			USART_EventCallback(pUSARTHandle, USART_TX_CMPLT);
		}
	}

	/* Handle ORE Event */
	check_it_bit = pUSARTHandle->pUSARTx->USART_CR1 & (1 << USART_CR1_RXNEIE);
	check_event = pUSARTHandle->pUSARTx->USART_SR & USART_FLAG_ORE;

	if (check_it_bit && check_event)
	{
		USART_EventCallback(pUSARTHandle, USART_OVR_ERROR);
	}


	/* Handle CTS Event (USART only -> UART4 & UART5 won't work) */
	check_it_bit = pUSARTHandle->pUSARTx->USART_CR3 & (1 << USART_CR3_CTSIE);
	check_event = pUSARTHandle->pUSARTx->USART_SR & USART_FLAG_CTS;

	if (check_it_bit && check_event)
	{
		if ((pUSARTHandle->pUSARTx != UART4) && (pUSARTHandle->pUSARTx != UART5))
			USART_EventCallback(pUSARTHandle, USART_CTS_EVENT);
	}


	/* Handle IDLE Event */
	check_it_bit = pUSARTHandle->pUSARTx->USART_CR1 & (1 << USART_CR1_IDLEIE);
	check_event = pUSARTHandle->pUSARTx->USART_SR & USART_FLAG_IDLE;

	if (check_it_bit && check_event)
	{
		USART_EventCallback(pUSARTHandle, USART_IDLE_ERROR);
	}

	/* Handle PE Event */
	check_it_bit = pUSARTHandle->pUSARTx->USART_CR1 & (1 << USART_CR1_PEIE);
	check_event = pUSARTHandle->pUSARTx->USART_SR & USART_FLAG_PE;

	if (check_it_bit && check_event)
	{
		USART_EventCallback(pUSARTHandle, USART_PE_ERROR);
	}
}

void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (IRQNumber <= 31)
			NVIC->NVIC_ISER[0] |= (1 << IRQNumber);
		else if (IRQNumber >= 32 && IRQNumber <= 63)
			NVIC->NVIC_ISER[1] |= (1 << (IRQNumber % 32));
		else if (IRQNumber >= 64 && IRQNumber <= 95)
			NVIC->NVIC_ISER[2] |= (1 << (IRQNumber % 64));
	}
	else if (EnorDi == DISABLE)
	{
		if (IRQNumber <= 31)
			NVIC->NVIC_ICER[0] |= (1 << IRQNumber);
		else if (IRQNumber >= 32 && IRQNumber <= 63)
			NVIC->NVIC_ICER[1] |= (1 << (IRQNumber % 32));
		else if (IRQNumber >= 64 && IRQNumber <= 95)
			NVIC->NVIC_ICER[2] |= (1 << (IRQNumber % 64));
	}
}


void USART_IRQPriorityConfig(uint8_t IRQNumber, uint8_t Priority)
{
	uint8_t NVIC_IPRx;
	uint8_t NVIC_IPRx_field;
	uint32_t shift_amount;

	// Each IPRx holds 4 IRQ number's priority
	NVIC_IPRx = IRQNumber / 4;

	// Find out which field the IRQ number is staying
	NVIC_IPRx_field = IRQNumber % 4;

	shift_amount = (NVIC_IPRx_field * 8) + 4;

	NVIC->NVIC_IPR[NVIC_IPRx] |= Priority << shift_amount;
}


uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint32_t FlagName)
{
	if (pUSARTx->USART_SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_NOT_SET;
}


void USART_Peripheral_Clk_Ctrl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pUSARTx == USART1)
			RCC_USART1_CLK_EN();
		else if (pUSARTx == USART2)
			RCC_USART2_CLK_EN();
		else if (pUSARTx == USART3)
			RCC_USART3_CLK_EN();
		else if (pUSARTx == UART4)
			RCC_UART4_CLK_EN();
		else if (pUSARTx == UART5)
			RCC_UART5_CLK_EN();
		else if (pUSARTx == USART6)
			RCC_USART6_CLK_EN();
	}
	else if (EnorDi == DISABLE)
	{
		if (pUSARTx == USART1)
			RCC_USART1_CLK_DI();
		else if (pUSARTx == USART2)
			RCC_USART2_CLK_DI();
		else if (pUSARTx == USART3)
			RCC_USART3_CLK_DI();
		else if (pUSARTx == UART4)
			RCC_UART4_CLK_DI();
		else if (pUSARTx == UART5)
			RCC_UART5_CLK_DI();
		else if (pUSARTx == USART6)
			RCC_USART6_CLK_DI();
	}
}

void USART_DeInit(USART_RegDef_t *pUSARTx)
{
	if (pUSARTx == USART1)
		RCC_USART1_RESET();
	else if (pUSARTx == USART2)
		RCC_USART2_RESET();
	else if (pUSARTx == USART3)
		RCC_USART3_RESET();
	else if (pUSARTx == UART4)
		RCC_UART4_RESET();
	else if (pUSARTx == UART5)
		RCC_UART5_RESET();
	else if (pUSARTx == USART6)
		RCC_USART6_RESET();
}

/****************** Helper function implementations **************/

static void USART_CloseSendData(USART_Handle_t *pUSARTHandle)
{
	/* Disable interrupt bits of data transfer */
	pUSARTHandle->pUSARTx->USART_CR1 &= ~(1 << USART_CR1_TCIE);
	pUSARTHandle->pUSARTx->USART_CR1 &= ~(1 << USART_CR1_TXEIE);

	/* Clear the handle's old data. Ready for a new transfer */
	pUSARTHandle->TxLen = 0;
	pUSARTHandle->pTxBuffer = NULL;

	/* Mark TX be ready */
	pUSARTHandle->TxState = USART_TX_READY;
}


static void USART_CloseReceiveData(USART_Handle_t *pUSARTHandle)
{
	pUSARTHandle->pUSARTx->USART_CR1 &= ~(1 << USART_CR1_RXNEIE);
	pUSARTHandle->RxLen = 0;
	pUSARTHandle->pRxBuffer = NULL;
	pUSARTHandle->RxState = USART_RX_READY;
}


static void USART_TxITHandler(USART_Handle_t *pUSARTHandle)
{
	uint16_t *pTxBuffer_16;

	if (pUSARTHandle->TxLen > 0)
	{
		/* Handle data transfer in 9 bits data length */
		if (pUSARTHandle->USARTConfig.USART_WordLength == USART_WORDLEN_9BITS)
		{
			pTxBuffer_16 = (uint16_t*)pUSARTHandle->pTxBuffer;
			pUSARTHandle->pUSARTx->USART_DR = (*pTxBuffer_16 & (uint16_t)0x1FF);

			/* If parity bit isn't used -> 9 bits of data. Buffer increases twice */
			if (pUSARTHandle->USARTConfig.USART_ParityControl == USART_PARITY_DISABLE)
			{
				(uint16_t*)pUSARTHandle->pTxBuffer++;
				pUSARTHandle->TxLen -= 2;
			}
			/* If parity bit is used -> 8 bits of data. Buffer increases once */
			else
			{
				pUSARTHandle->pTxBuffer++;
				pUSARTHandle->TxLen--;
			}
		}
		/* Handle data transfer in 8 bits data length */
		else if (pUSARTHandle->USARTConfig.USART_WordLength == USART_WORDLEN_8BITS)
		{
			pUSARTHandle->pUSARTx->USART_DR = *(pUSARTHandle->pTxBuffer);
			pUSARTHandle->pTxBuffer++;
			pUSARTHandle->TxLen--;
		}
	}

	if (pUSARTHandle->TxLen == 0)
	{
		/* Disable the TXEIE interrupt event */
		pUSARTHandle->pUSARTx->USART_CR1 &= ~(1 << USART_CR1_TXEIE);
	}
}


static void USART_RxITHandler(USART_Handle_t *pUSARTHandle)
{
	if (pUSARTHandle->RxLen > 0)
	{
		/* Handle data receiving in 9 bits word length mode */
		if (pUSARTHandle->USARTConfig.USART_WordLength == USART_WORDLEN_9BITS)
		{
			/* If parity isn't used -> 9 bits of data */
			if (pUSARTHandle->USARTConfig.USART_ParityControl == USART_PARITY_DISABLE)
			{
				*( (uint16_t*)pUSARTHandle->pRxBuffer) = (uint16_t)(pUSARTHandle->pUSARTx->USART_DR & (uint16_t)0x1FF);
				(uint16_t *)pUSARTHandle->pRxBuffer++;
				pUSARTHandle->RxLen -=2;
			}
			/* If parity is used -> 8 bits of data */
			else
			{
				*pUSARTHandle->pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->USART_DR & (uint8_t)0xFF);
				pUSARTHandle->pRxBuffer++;
				pUSARTHandle->RxLen--;
			}
		}
		/* Handle data receiving in 8 bits word length mode */
		else if (pUSARTHandle->USARTConfig.USART_WordLength == USART_WORDLEN_8BITS)
		{
			/* If parity isn't used -> 8 bits of data */
			if (pUSARTHandle->USARTConfig.USART_ParityControl == USART_PARITY_DISABLE)
			{
				*pUSARTHandle->pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->USART_DR & (uint8_t)0xFF);
				pUSARTHandle->pRxBuffer++;
				pUSARTHandle->RxLen--;
			}
			/* If parity is used -> 7 bits of data */
			else
			{
				*pUSARTHandle->pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->USART_DR & (uint8_t)0x7F);
				pUSARTHandle->pRxBuffer++;
				pUSARTHandle->RxLen--;
			}
		}
	}

	if (pUSARTHandle->RxLen == 0)
	{
		/* Close Data receiving */
		USART_CloseReceiveData(pUSARTHandle);

		/* Inform the application */
		USART_EventCallback(pUSARTHandle, USART_RX_CMPLT);
	}
}

static void USART_SetBaudRate(USART_Handle_t *pUSARTHandle)
{
	uint32_t USART_div, mantissa_part, fraction_part;
	uint8_t Oversampling;
	uint32_t Pclk;
	uint32_t tempreg = 0;

	if (pUSARTHandle->pUSARTx == USART1 || pUSARTHandle->pUSARTx == USART6)
	{
		/* PCLK 2 supplies the USART */
		Pclk = RCC_Get_PCLK2_Value();
	}
	else
	{
		/* PCLK 1 supplies the USART */
		Pclk = RCC_Get_PCLK1_Value();
	}

	if (pUSARTHandle->USARTConfig.USART_OverSampling == USART_OVERSAMPLING_8)
	{
		Oversampling = 8;
	}
	else if (pUSARTHandle->USARTConfig.USART_OverSampling == USART_OVERSAMPLING_16)
	{
		Oversampling = 16;
	}

	USART_div = ((Pclk * 100) / (Oversampling * pUSARTHandle->USARTConfig.USART_Baudrate));
	mantissa_part = USART_div / 100;
	fraction_part = USART_div - (mantissa_part * 100);

	if (Oversampling == 8)
	{
		fraction_part = (((fraction_part * Oversampling) + 50) / 100) & (uint8_t)0x07;	// 3 bits max
	}
	else if (Oversampling == 16)
	{
		fraction_part = (((fraction_part * Oversampling) + 50) / 100) & (uint8_t)0x0F; 	// 4 bits max
	}

	tempreg |= mantissa_part << USART_BRR_DIV_Mantissa;
	tempreg |= fraction_part << USART_BRR_DIV_Fraction;

	pUSARTHandle->pUSARTx->USART_BRR = tempreg;
}



__weak void USART_EventCallback(USART_Handle_t * pUSARTHandle, uint8_t Event)
{

}
