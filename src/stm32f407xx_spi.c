/*
 * stm32f407xx_spi.c
 *
 *  Created on: Nov 27, 2023
 *      Author: gia nguyen
 */
#include "stm32f407xx_spi.h"


/*************************************************************************
 * @fn				- SPI_Peripheral_Clk_Ctrl
 *
 * @brief			- Enable/Disable the SPI peripheral clock
 *
 * @param[in]		- pSPIx: SPI instance
 * @param[in]		- EnorDi: Enable/Disable
 * @return			- none
 *
 */
void SPI_Peripheral_Clk_Ctrl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pSPIx == SPI1)
			RCC_SPI1_CLK_EN();
		else if (pSPIx == SPI2)
			RCC_SPI2_CLK_EN();
		else if (pSPIx == SPI3)
			RCC_SPI3_CLK_EN();
	}
	else if (EnorDi == DISABLE)
	{
		if (pSPIx == SPI1)
			RCC_SPI1_CLK_DI();
		else if (pSPIx == SPI2)
			RCC_SPI2_CLK_DI();
		else if (pSPIx == SPI3)
			RCC_SPI3_CLK_DI();
	}
}

/*************************************************************************
 * @fn				- SPI_Init
 *
 * @brief			- Initialize the SPI: clock, mode, baudrate, CPOL, CPHA, bus,..
 *
 * @param[in]		- pSPI_Handle: holds SPI instance & configurations
 *
 * @return			- none
 */
void SPI_Init(SPI_Handle_t *pSPI_Handle)
{
	uint32_t temp_reg = 0;

	/* Enable the SPI peripheral clock */
	SPI_Peripheral_Clk_Ctrl(pSPI_Handle->pSPIx, ENABLE);

	/* Configure SPI mode: master/slave */
	temp_reg |= (pSPI_Handle->SPI_Config.SPI_Mode << SPI_CR1_MSTR);

	/* Configure Bus: full-duplex/half-duplex/simplex */
	if (pSPI_Handle->SPI_Config.SPI_BusConfig == SPI_BUS_FULL_DUPLEX)
	{
		// Clear the BIDIMODE bit (BIDIMODE bit = 0 -> Full duplex)
		temp_reg &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if (pSPI_Handle->SPI_Config.SPI_BusConfig == SPI_BUS_HALF_DUPLEX)
	{
		// Set the BIDIMODE bit (BIDIMODE bit = 1 -> Half duplex)
		temp_reg |= (1 << SPI_CR1_BIDIMODE);
	}
	else if (pSPI_Handle->SPI_Config.SPI_BusConfig == SPI_BUS_SIMPLEX_RXONLY)
	{
		// Clear BIDIMODE bit
		temp_reg &= ~(1 << SPI_CR1_BIDIMODE);
		// Set RXONLY bit
		temp_reg |= (1 << SPI_CR1_RXONLY);
	}

	/* Configure Data frame format */
	temp_reg |= (pSPI_Handle->SPI_Config.SPI_DFF << SPI_CR1_DFF);

	/* Configure Clock phase */
	temp_reg |= (pSPI_Handle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA);

	/* Configure Clock polarity */
	temp_reg |= (pSPI_Handle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL);

	/* Configure Baudrate */
	temp_reg |= (pSPI_Handle->SPI_Config.SPI_Baudrate << SPI_CR1_BR);

	/* Configure Software Slave Management (SSM) */
	temp_reg |= (pSPI_Handle->SPI_Config.SPI_SSM << SPI_CR1_SSM);

	pSPI_Handle->pSPIx->SPI_CR1 = temp_reg;
}

/*************************************************************************
 * @fn				- SPI_DeInit
 *
 * @brief			- Reset SPI peripheral
 *
 * @param[in]		- pSPIx: SPI instance
 *
 * @return			- none
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if (pSPIx == SPI1)
		RCC_SPI1_RESET();
	else if (pSPIx == SPI2)
		RCC_SPI2_RESET();
	else if (pSPIx == SPI3)
		RCC_SPI3_RESET();
}



/*************************************************************************
 * @fn				- SPI_SendData
 *
 * @brief			- Send data over SPI (polling mode)
 *
 * @param[in]		- pSPIx: SPI instance
 *					- pTxBuffer: data wanted to transmit
 *					- len: number of bytes of the data
 * @return			- none
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len)
{
	// Try to send until the length = 0
	while (len > 0)
	{
		// Wait until Tx becomes empty
		while ( SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_NOT_SET);

		// Send data in 16-bit or 8-bit
		if (pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF))
		{
			// 16 bit
			pSPIx->SPI_DR = *( (uint16_t*)pTxBuffer);
			pTxBuffer += 2;
			len -= 2;
		}
		else
		{
			// 8 bit
			pSPIx->SPI_DR = *pTxBuffer	;
			pTxBuffer++;
			len--;
		}
	}
}


/*************************************************************************
 * @fn				- SPI_ReceiveData
 *
 * @brief			- Receive data over SPI (polling mode)
 *
 * @param[in]		- pSPIx: SPI instance
 *					- pRxBuffer: receive data buffer
 *					- len: number of bytes of the data
 * @return			- none
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len)
{
	while (len > 0)
	{
		// Wait until the receive buffer is full (RNXE is set)
		while (SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == (uint8_t)FLAG_NOT_SET);

		if (pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF))
		{
			// 16-bit DFF
			*( (uint16_t*)pRxBuffer) = (uint16_t)pSPIx->SPI_DR;
			(uint16_t*)pRxBuffer++;
			len -= 2;
		}
		else
		{
			// 8-bit DFF
			*pRxBuffer = pSPIx->SPI_DR;
			pRxBuffer++;
			len--;
		}
	}
}

/*************************************************************************
 * @fn				- SPI_SendDataIT
 *
 * @brief			- Send data over SPI (interrupt mode)
 *
 * @param[in]		- pSPI_Handle: SPI handle
 *					- pTxBuffer: data wanted to transmit
 *					- len: number of bytes of the data
 * @return			- SPI Tx state
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPI_Handle, uint8_t *pTxBuffer, uint32_t len)
{
	// Only handle Send data IT if the SPI TX is ready
	uint8_t state = pSPI_Handle->TxState;

	if (pSPI_Handle->TxState == SPI_TX_READY)
	{
		// 1. Save Tx address & Tx length information taken from user app into global handle structure
		pSPI_Handle->pTxBuffer = pTxBuffer;
		pSPI_Handle->TxLen = len;

		// 2. Mark Tx bus as busy so no other send data functions can use this Tx bus until the transmission finish.
		pSPI_Handle->TxState = SPI_TX_BUSY;

		// 3. Enable the TXE interrupt (by enabling the TXEIE bit) so that TXE flag can be set when ready.
		pSPI_Handle->pSPIx->SPI_CR2 |= (1 << SPI_CR2_TXEIE);

		// 4. Data transmission will be handled by ISR function
	}
	return state;
}

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPI_Handle, uint8_t *pRxBuffer, uint32_t len)
{
	// Only handle Receive data IT if the SPI RX is ready
	uint8_t state = pSPI_Handle->RxState;

	if (pSPI_Handle->RxState == SPI_RX_READY)
	{
		pSPI_Handle->pRxBuffer = pRxBuffer;
		pSPI_Handle->RxLen = len;
		pSPI_Handle->RxState = SPI_RX_BUSY;
		pSPI_Handle->pSPIx->SPI_CR2 |= (1 << SPI_CR2_RXNEIE);
	}

	return state;
}

/*************************************************************************
 * @fn				- SPI_IRQInterruptConfig
 *
 * @brief			- Enable the IRQ number (which is SPI line connects to) in the NVIC register
 *
 * @param[in]		- IRQ_Number: IRQ number of SPIx
 * @param[in]		- EnorDi: enable/disable
 *
 * @return			- none
 */
void SPI_IRQInterruptConfig(uint8_t IRQ_Number, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (IRQ_Number <= 31)
			NVIC->NVIC_ISER[0] |= (1 << IRQ_Number);
		else if (IRQ_Number >= 32 && IRQ_Number <= 63)
			NVIC->NVIC_ISER[1] |= (1 << (IRQ_Number % 32));
		else if (IRQ_Number >= 64 && IRQ_Number <= 95)
			NVIC->NVIC_ISER[2] |= (1 << (IRQ_Number % 64));
	}
	else if (EnorDi == DISABLE)
	{
		if (IRQ_Number <= 31)
			NVIC->NVIC_ICER[0] |= (1 << IRQ_Number);
		else if (IRQ_Number >= 32 && IRQ_Number <= 63)
			NVIC->NVIC_ICER[1] |= (1 << (IRQ_Number % 32));
		else if (IRQ_Number >= 64 && IRQ_Number <= 95)
			NVIC->NVIC_ICER[2] |= (1 << (IRQ_Number % 64));
	}
}

/*************************************************************************
 * @fn				- SPI_IRQPriorityConfig
 *
 * @brief			- Configure the priority in the NVIC peripheral
 *
 * @param[in]		- IRQ_Number: IRQ number of the SPIx
 * @param[in]		- Priority: IRQ priority level.
 *
 * @return			- none
 */
void SPI_IRQPriorityConfig(uint8_t IRQ_Number, uint32_t IRQ_Priority)
{
	uint8_t NVIC_IPRx;
	uint8_t NVIC_IPRx_field;
	uint32_t shift_amount;

	// find out which IPR holds the IRQ number
	NVIC_IPRx = IRQ_Number / 4;

	// find out which field of the IPR holds the priority number
	NVIC_IPRx_field = IRQ_Number % 4;

	// calculate shift amount
	shift_amount = (NVIC_IPRx_field * 8) + 4;	// first 4 bits of a field are ignored, so add 4

	// set the priority in the corresponding IPR
	NVIC->NVIC_IPR[NVIC_IPRx] |= (IRQ_Priority << shift_amount);

}


/*************************************************************************
 * @fn				- SPI_TXE_IT_Handle
 *
 * @brief			- Send data IT helper function, called by SPI IRQ Handler
 *
 * @param[in]		- pSPI_Handle
 *
 * @note			- Send 1 byte at one time. When length = 0 -> stop the TXE interrupt
 * @return			- none
 */
static void SPI_TXE_IT_Handle(SPI_Handle_t *pSPI_Handle)
{
	if (pSPI_Handle->TxLen > 0)
	{
		if (pSPI_Handle->SPI_Config.SPI_DFF == SPI_DFF_8_BITS)
		{
			pSPI_Handle->pSPIx->SPI_DR = *(pSPI_Handle->pTxBuffer);
			pSPI_Handle->pTxBuffer++;
			pSPI_Handle->TxLen--;
		}
		else if (pSPI_Handle->SPI_Config.SPI_DFF == SPI_DFF_16_BITS)
		{
			pSPI_Handle->pSPIx->SPI_DR = *( (uint16_t*)pSPI_Handle->pTxBuffer);
			(uint16_t*)pSPI_Handle->pTxBuffer++;
			pSPI_Handle->TxLen -=2;
		}
	}

	if (pSPI_Handle->TxLen == 0)
	{
		// Reset all the Tx values
		pSPI_Handle->pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_TXEIE);
		pSPI_Handle->pTxBuffer = NULL;
		pSPI_Handle->TxLen = 0;
		pSPI_Handle->TxState = SPI_TX_READY;

		// Inform the application
		SPI_EventCallback(pSPI_Handle, SPI_EVENT_TX_CMPLT);

	}
}


/*************************************************************************
 * @fn				- SPI_RXNE_IT_Handle
 *
 * @brief			- Receive data IT helper function, called by SPI IRQ Handler
 *
 * @param[in]		- pSPI_Handle
 *
 * @note			- Receive 1 byte at one time. When length = 0 -> stop the RXNE interrupt
 * @return			- none
 */
static void SPI_RXNE_IT_Handle(SPI_Handle_t *pSPI_Handle)
{
	if (pSPI_Handle->RxLen > 0)
	{
		if (pSPI_Handle->SPI_Config.SPI_DFF == SPI_DFF_8_BITS)
		{
			*(pSPI_Handle->pRxBuffer) = (uint8_t)pSPI_Handle->pSPIx->SPI_DR;
			pSPI_Handle->pRxBuffer++;
			pSPI_Handle->RxLen--;
		}
		else if (pSPI_Handle->SPI_Config.SPI_DFF == SPI_DFF_16_BITS)
		{
			*( (uint16_t*)pSPI_Handle->pRxBuffer) = (uint16_t)pSPI_Handle->pSPIx->SPI_DR;
			pSPI_Handle->pRxBuffer++;
			pSPI_Handle->pRxBuffer++;
			pSPI_Handle->RxLen -=2;
		}
	}

	if (pSPI_Handle->RxLen == 0)
	{
		// Reset all Rx value, close the communication
		pSPI_Handle->pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_RXNEIE);
		pSPI_Handle->pRxBuffer = NULL;
		pSPI_Handle->RxLen = 0;
		pSPI_Handle->RxState = SPI_RX_READY;


		// Inform the application
		SPI_EventCallback(pSPI_Handle, SPI_EVENT_RX_CMPLT);
	}

}

/*************************************************************************
 * @fn				- SPI_Overrun_IT_Handle
 *
 * @brief			- Handle overrun error, called by SPI IRQ Handler
 *
 * @param[in]		- pSPI_Handle
 *
 * @note			-
 * @return			- none
 */
static void SPI_Overrun_IT_Handle(SPI_Handle_t *pSPI_Handle)
{
	// inform the application
	SPI_EventCallback(pSPI_Handle, SPI_EVENT_OVR_ERR);
}


/*************************************************************************
 * @fn				- SPI_IRQHandler
 *
 * @brief			-
 * 					-
 *
 * @param[in]		- IRQ_Number: IRQ number of the SPI
 * @param[in]		- Priority: IRQ priority level.
 *
 * @return			- none
 */
void SPI_IRQHandler(SPI_Handle_t *pSPI_Handle)
{
	uint8_t check_event, check_enable_bit;

	// Check TXE event
	check_event = (pSPI_Handle->pSPIx->SPI_SR & (1 << SPI_SR_TXE));
	check_enable_bit = (pSPI_Handle->pSPIx->SPI_CR2 & (1 << SPI_CR2_TXEIE));

	if ((check_event) && (check_enable_bit))
	{
		SPI_TXE_IT_Handle(pSPI_Handle);
	}

	// Check RXNE event
	check_event = (pSPI_Handle->pSPIx->SPI_SR & (1 << SPI_SR_RXNE));
	check_enable_bit = (pSPI_Handle->pSPIx->SPI_CR2 & (1 << SPI_CR2_RXNEIE));

	if ((check_event) && (check_enable_bit))
	{
		SPI_RXNE_IT_Handle(pSPI_Handle);
	}


	// Check overrun error event
	check_event = (pSPI_Handle->pSPIx->SPI_SR & (1 << SPI_SR_OVR));
	check_enable_bit = (pSPI_Handle->pSPIx->SPI_CR2 & (1 << SPI_CR2_ERRIE));

	if ((check_event) && (check_enable_bit))
	{
		SPI_Overrun_IT_Handle(pSPI_Handle);
	}

}


/*************************************************************************
 * @fn				- SPI_GetFlagStatus
 *
 * @brief			- Determine a Flag is set or not
 *
 * @param[in]		- pSPIx: SPI instance
 * @param[in]		- Flag: Flag name.
 *
 * @return			- FLAG_SET or FLAG_NOT_SET
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t Flag)
{
	// Do mask bit
	if (pSPIx->SPI_SR & Flag)
	{
		return FLAG_SET;
	}
	return FLAG_NOT_SET;
}

/*************************************************************************
 * @fn				- SPI_Start
 *
 * @brief			- Start the communication
 *
 * @param[in]		- pSPIx: SPI instance
 *
 * @return			- none
 */
void SPI_Start(SPI_RegDef_t *pSPIx)
{
	pSPIx->SPI_CR1 |= (1 << SPI_CR1_SPE);
}

/*************************************************************************
 * @fn				- SPI_Stop
 *
 * @brief			- Stop the communication
 *
 * @param[in]		- pSPIx: SPI instance
 *
 * @return			- none
 */
void SPI_Stop(SPI_RegDef_t *pSPIx)
{
	pSPIx->SPI_CR1 &= ~(1 << SPI_CR1_SPE);
}

/*************************************************************************
 * @fn				- SPI_ConfigSSI
 *
 * @brief			- Set/Clear SSI bit (only when SSM = 1)
 *
 * @param[in]		- pSPIx: SPI instance
 * @param[in]		- EnorDi: Enable/Disable.
 *
 * @return			- none
 */
void SPI_ConfigSSI(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		pSPIx->SPI_CR1 |= (1 << SPI_CR1_SSI);
	}
	else if (EnorDi == DISABLE)
	{
		pSPIx->SPI_CR1 &= ~(1 << SPI_CR1_SSI);
	}
}
/*************************************************************************
 * @fn				- SPI_ConfigSSOE
 *
 * @brief			- Set/Clear SSOE bit (only when SSM = 0)
 *
 * @param[in]		- pSPIx: SPI instance
 * @param[in]		- EnorDi: Enable/Disable.
 *
 * @return			- none
 */
void SPI_ConfigSSOE(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		pSPIx->SPI_CR2 |= (1 << SPI_CR2_SSOE);
	}
	else if (EnorDi == DISABLE)
	{
		pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}


__weak void SPI_EventCallback(SPI_Handle_t *pSPI_Handle, uint8_t Event)
{

}
