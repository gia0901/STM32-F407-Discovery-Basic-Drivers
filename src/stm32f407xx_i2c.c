/*
 * stm32f407xx_i2c.c
 *
 *  Created on: Nov 30, 2023
 *      Author: gia nguyen
 */

#include "stm32f407xx_i2c.h"


/********************** Private helper functions of I2C Driver ***********************/

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->I2C_CR1 |= (1 << I2C_CR1_START);
}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->I2C_CR1 |= (1 << I2C_CR1_STOP);
}

static void I2C_ExecuteAddrPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1);
	// Send 7 address bits  + 1 write bit to DR
	pI2Cx->I2C_DR = SlaveAddr;
}

static void I2C_ExecuteAddrPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= (1);
	// Send 7 address bits + 1 read bit to DR
	pI2Cx->I2C_DR = SlaveAddr;
}

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2C_Handle)
{
	uint32_t dummy_read;

	if (pI2C_Handle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_MSL))
	{
		// Master mode: TX or RX. If it is Tx, just clear the flag,
		// but if it is Rx and receive 1 byte only, must disable ACK before clearing the flag
		if (pI2C_Handle->TxRxState == I2C_RX_BUSY)
		{
			// Master mode in RX
			if (pI2C_Handle->RxTotalLen == 1)
			{
				I2C_AckControl(pI2C_Handle->pI2Cx, DISABLE);
			}
			dummy_read = pI2C_Handle->pI2Cx->I2C_SR1;
			dummy_read = pI2C_Handle->pI2Cx->I2C_SR2;
			(void)dummy_read;
		}
		else
		{
			// Master mode in TX
			dummy_read = pI2C_Handle->pI2Cx->I2C_SR1;
			dummy_read = pI2C_Handle->pI2Cx->I2C_SR2;
			(void)dummy_read;
		}
	}
	else
	{
		// Slave mode
		dummy_read = pI2C_Handle->pI2Cx->I2C_SR1;
		dummy_read = pI2C_Handle->pI2Cx->I2C_SR2;
		(void)dummy_read;
	}


}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if (pI2Cx->I2C_SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_NOT_SET;
}

static void I2C_CloseSendData(I2C_Handle_t *pI2C_Handle)
{
	/* Turn off interrupt bits and reset all the Handle's data */
	pI2C_Handle->pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITBUFEN);	// Buffer interrupt bit
	pI2C_Handle->pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITEVTEN);	// Event interrupt bit

	pI2C_Handle->pTxBuffer = NULL;
	pI2C_Handle->TxLen = 0;
	pI2C_Handle->TxRxState = I2C_READY;

}

static void I2C_CloseReceiveData(I2C_Handle_t *pI2C_Handle)
{
	/* Turn off interrupt bits and reset all the Handle's data */
	pI2C_Handle->pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITBUFEN);	// Buffer interrupt bit
	pI2C_Handle->pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITEVTEN);	// Event interrupt bit

	pI2C_Handle->pRxBuffer = NULL;
	pI2C_Handle->RxLen = 0;
	pI2C_Handle->RxTotalLen = 0;
	pI2C_Handle->TxRxState = I2C_READY;

	/* Must enable ACK again because it was disabled when there is 1 receive byte left */
	I2C_AckControl(pI2C_Handle->pI2Cx, I2C_ACK_ENABLE);
}

static void I2C_MasterRXNE_IT_Handle(I2C_Handle_t *pI2C_Handle)
{
	if (pI2C_Handle->RxTotalLen == 1)
	{
		*(pI2C_Handle->pRxBuffer) = pI2C_Handle->pI2Cx->I2C_DR;
		pI2C_Handle->RxLen--;
	}

	if (pI2C_Handle->RxTotalLen > 1)
	{
		if (pI2C_Handle->RxLen == 2)
		{
			// Prepare to close the communication
			I2C_AckControl(pI2C_Handle->pI2Cx, I2C_ACK_DISABLE);
		}
		*(pI2C_Handle->pRxBuffer) = pI2C_Handle->pI2Cx->I2C_DR;
		pI2C_Handle->pRxBuffer++;
		pI2C_Handle->RxLen--;
	}

	if (pI2C_Handle->RxLen == 0)
	{
		// Time to close the communication
		if (pI2C_Handle->Sr == I2C_REPEATED_START_DISABLE)
			I2C_GenerateStopCondition(pI2C_Handle->pI2Cx);

		// Close I2C Reception
		I2C_CloseReceiveData(pI2C_Handle);

		// Inform the application
		I2C_EventCallback(pI2C_Handle, I2C_EVENT_RX_CMPLT);
	}
}

static void I2C_MasterTXE_IT_Handle(I2C_Handle_t *pI2C_Handle)
{
	if (pI2C_Handle->TxLen > 0)
	{
		pI2C_Handle->pI2Cx->I2C_DR = *(pI2C_Handle->pTxBuffer);
		pI2C_Handle->pTxBuffer++;
		pI2C_Handle->TxLen--;
	}
}

/********************** APIs definitions of I2C Driver ***********************/

void I2C_Peripheral_Clk_Ctrl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pI2Cx == I2C1)
			RCC_I2C1_CLK_EN();
		else if (pI2Cx == I2C2)
			RCC_I2C2_CLK_EN();
		else if (pI2Cx == I2C3)
			RCC_I2C3_CLK_EN();
	}
	else if (EnorDi == DISABLE)
	{
		if (pI2Cx == I2C1)
			RCC_I2C1_CLK_DI();
		else if (pI2Cx == I2C2)
			RCC_I2C2_CLK_DI();
		else if (pI2Cx == I2C3)
			RCC_I2C3_CLK_DI();
	}


}


void I2C_AckControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if (EnorDi == I2C_ACK_ENABLE)
		pI2Cx->I2C_CR1 |= (1 << I2C_CR1_ACK);
	else if (EnorDi == I2C_ACK_DISABLE)
		pI2Cx->I2C_CR1 &= ~(1 << I2C_CR1_ACK);
}


void I2C_Init(I2C_Handle_t *pI2C_Handle)
{
	uint32_t tempreg = 0;
	uint32_t CCR_value;

	/* Enable the peripheral clock */
	I2C_Peripheral_Clk_Ctrl(pI2C_Handle->pI2Cx, ENABLE);

	I2C_AckControl(pI2C_Handle->pI2Cx, ENABLE);

	/* Calculate & Configure the SCL */

	// Program Peripheral clock frequency (FREQ field)
	tempreg = (RCC_Get_PCLK1_Value() / 1000000) << I2C_CR2_FREQ;
	pI2C_Handle->pI2Cx->I2C_CR2 = (tempreg & 0x3F);	// mask 6 bits of FREQ only

	// Program Clock Control Register in standard/fast mode
	if (pI2C_Handle->I2C_Config.I2C_SclSpeed <= I2C_SCL_SPEED_STANDARD)
	{
		// standard mode: just config CCR
		CCR_value = (RCC_Get_PCLK1_Value() / (2 * pI2C_Handle->I2C_Config.I2C_SclSpeed));

		tempreg = (CCR_value & 0xFFF);	// mask 12 bits
	}
	else
	{
		// fast mode: set fast mode & duty cycle bits
		tempreg = 1 << I2C_CCR_FS;
		tempreg |= pI2C_Handle->I2C_Config.I2C_FastModeDutyCycle << I2C_CCR_DUTY;

		// calculate CCR value:
		if (pI2C_Handle->I2C_Config.I2C_FastModeDutyCycle == I2C_FAST_DUTY_CYCLE_2)
		{
			CCR_value = (RCC_Get_PCLK1_Value() / (3 * pI2C_Handle->I2C_Config.I2C_SclSpeed));
		}
		else if (pI2C_Handle->I2C_Config.I2C_FastModeDutyCycle == I2C_FAST_DUTY_CYCLE_16_9)
		{
			CCR_value = (RCC_Get_PCLK1_Value() / (25 * pI2C_Handle->I2C_Config.I2C_SclSpeed));
		}

		tempreg |= (CCR_value & 0xFFF);	// mask 12 bits
	}

	pI2C_Handle->pI2Cx->I2C_CCR = tempreg;

	/* Configure Address (when device is slave) */

	tempreg = pI2C_Handle->I2C_Config.I2C_SlaveAddr << 1;
	tempreg |= (1 << 14);	// The RM said must set 14th bit to 1
	pI2C_Handle->pI2Cx->I2C_OAR1 = tempreg;

	/* Configure the rise time (T-rise) */
	if (pI2C_Handle->I2C_Config.I2C_SclSpeed <= I2C_SCL_SPEED_STANDARD)
	{
		tempreg = (RCC_Get_PCLK1_Value() / 1000000) + 1;	//  F_pclk1 * (1000 * 10^-9) = F_pclk1 / (10 ^ 6)
	}
	else
	{
		tempreg = (RCC_Get_PCLK1_Value() * (300/1000000000) ) + 1;
	}

	pI2C_Handle->pI2Cx->I2C_TRISE = (tempreg & 0x3F);

	/* Enable ACK (ACK can be enabled only when PE = 1, so don't enable it here) */
}


void I2C_MasterSend(I2C_Handle_t *pI2C_Handle, uint8_t *pTxBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t Sr)
{
	I2C_AckControl(pI2C_Handle->pI2Cx, I2C_ACK_ENABLE);

	/* Generate a Start condition */
	I2C_GenerateStartCondition(pI2C_Handle->pI2Cx);

	/* Clear SB flag & do the address phase: read SB then write slave address to Data register DR */
	while (I2C_GetFlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_SB) == FLAG_NOT_SET);
	I2C_ExecuteAddrPhaseWrite(pI2C_Handle->pI2Cx, SlaveAddr);

	/* Wait & Clear ADDR flag */
	while (I2C_GetFlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_ADDR) == FLAG_NOT_SET);
	I2C_ClearADDRFlag(pI2C_Handle);

	/* Begin the transmission: send all data until len = 0 */
	while (len > 0)
	{
		// Wait until TxE is set
		while (I2C_GetFlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_TXE) == FLAG_NOT_SET);
		// Send 1 byte
		pI2C_Handle->pI2Cx->I2C_DR = *pTxBuffer;
		pTxBuffer++;
		len--;

	}

	/* Wait until both TXE & BTF flags are set */
	while (I2C_GetFlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_TXE) == FLAG_NOT_SET);
	while (I2C_GetFlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_BTF) == FLAG_NOT_SET);

	/* Generate a STOP condition (only when repeated start is disabled) */
	if (Sr == I2C_REPEATED_START_DISABLE)
	{
		I2C_GenerateStopCondition(pI2C_Handle->pI2Cx);
	}
}



void I2C_MasterReceive(I2C_Handle_t *pI2C_Handle, uint8_t *pRxBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t Sr)
{
	I2C_AckControl(pI2C_Handle->pI2Cx, I2C_ACK_ENABLE);

	/* Generate a Start condition */
	I2C_GenerateStartCondition(pI2C_Handle->pI2Cx);

	/* Clear SB bit & execute the ADDR phase */
	while (I2C_GetFlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_SB) == FLAG_NOT_SET);
	I2C_ExecuteAddrPhaseRead(pI2C_Handle->pI2Cx, SlaveAddr);

	/* Wait for ADDR flag */
	while (I2C_GetFlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_ADDR) == FLAG_NOT_SET);


	if (len == 1)
	{
		/* disable ACK before clearing ADDR flag (refer to the reference manual) */
		I2C_AckControl(pI2C_Handle->pI2Cx, I2C_ACK_DISABLE);

		I2C_ClearADDRFlag(pI2C_Handle);

		/* Data is being sent by the slave after clearing ADDR flag	*/
		while (I2C_GetFlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_RXNE) == FLAG_NOT_SET);

		if (Sr == I2C_REPEATED_START_DISABLE)
			I2C_GenerateStopCondition(pI2C_Handle->pI2Cx);

		*pRxBuffer = pI2C_Handle->pI2Cx->I2C_DR;
	}

	if (len > 1)
	{

		I2C_ClearADDRFlag(pI2C_Handle);
		while (len > 0)
		{
			while (I2C_GetFlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_RXNE) == FLAG_NOT_SET);

			if (len == 2)
			{
				I2C_AckControl(pI2C_Handle->pI2Cx, I2C_ACK_DISABLE);
				I2C_GenerateStopCondition(pI2C_Handle->pI2Cx);
			}

			*pRxBuffer = pI2C_Handle->pI2Cx->I2C_DR;
			pRxBuffer++;
			len--;
		}
		if (Sr == I2C_REPEATED_START_DISABLE)
			I2C_GenerateStopCondition(pI2C_Handle->pI2Cx);
	}

	/* Re-enable the ACK */
	I2C_AckControl(pI2C_Handle->pI2Cx, I2C_ACK_ENABLE);
}


uint8_t I2C_MasterSendIT(I2C_Handle_t *pI2C_Handle, uint8_t *pTxBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t current_state = (pI2C_Handle->TxRxState);

	/* ONLY handle the API when the bus is ready (free) */
	if ( (current_state != I2C_TX_BUSY) && (current_state != I2C_RX_BUSY) )
	{
		/* Mark the bus busy in Tx */
		pI2C_Handle->TxRxState = I2C_TX_BUSY;

		/* Update the data to the Handle variable (it will be passed to IRQ handler to transfer) */
		pI2C_Handle->pTxBuffer = pTxBuffer;
		pI2C_Handle->TxLen = len;
		pI2C_Handle->Sr = Sr;
		pI2C_Handle->SlaveAddr = SlaveAddr;

		/* Generate a start condition */
		I2C_GenerateStartCondition(pI2C_Handle->pI2Cx);

		/* Enable the necessary interrupts */
		pI2C_Handle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITBUFEN);	// Buffer interrupt bit
		pI2C_Handle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITEVTEN);	// Event interrupt bit
		pI2C_Handle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITERREN);	// Error interrupt bit


	}

	return current_state;
}


uint8_t I2C_MasterReceiveIT(I2C_Handle_t *pI2C_Handle, uint8_t *pRxBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t current_state = (pI2C_Handle->TxRxState);

	if ( (current_state != I2C_TX_BUSY) && (current_state != I2C_RX_BUSY) )	// should do something here!!!!
	{
		/* Mark the bus busy in Rx */
		pI2C_Handle->TxRxState = I2C_RX_BUSY;

		/* Update the data to the Handle variable (it will be passed to IRQ handler to transfer) */
		pI2C_Handle->pRxBuffer = pRxBuffer;
		pI2C_Handle->RxLen = len;
		pI2C_Handle->RxTotalLen = len;
		pI2C_Handle->Sr = Sr;
		pI2C_Handle->SlaveAddr = SlaveAddr;

		/* Generate a start condition */
		I2C_GenerateStartCondition(pI2C_Handle->pI2Cx);

		/* Enable the necessary interrupts */
		pI2C_Handle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITBUFEN);	// Buffer interrupt bit
		pI2C_Handle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITEVTEN);	// Event interrupt bit
		pI2C_Handle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITERREN);	// Error interrupt bit
	}

	return current_state;
}

void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if (pI2Cx == I2C1)
		RCC_I2C1_RESET();
	else if (pI2Cx == I2C2)
		RCC_I2C2_RESET();
	else if (pI2Cx == I2C3)
		RCC_I2C3_RESET();
}

void I2C_Start(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->I2C_CR1 |= (1 << I2C_CR1_PE);
}

void I2C_Stop(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->I2C_CR1 &= ~(1 << I2C_CR1_PE);
}



void I2C_Event_IRQHandler(I2C_Handle_t *pI2C_Handle)
{
	uint32_t check_ITEVTEN = (pI2C_Handle->pI2Cx->I2C_CR2 & (1 << I2C_CR2_ITEVTEN));
	uint32_t check_ITBUFEN = (pI2C_Handle->pI2Cx->I2C_CR2 & (1 << I2C_CR2_ITBUFEN));
	uint32_t check_event;

	/* Check SB flag (This event is available in Master mode only) */
	check_event = (pI2C_Handle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_SB));
	if (check_ITEVTEN && check_event)
	{
		// Clear SB flag, then execute the address phase
		if (pI2C_Handle->TxRxState == I2C_TX_BUSY)
		{
			I2C_ExecuteAddrPhaseWrite(pI2C_Handle->pI2Cx, pI2C_Handle->SlaveAddr);
		}
		else if (pI2C_Handle->TxRxState == I2C_RX_BUSY)
		{
			I2C_ExecuteAddrPhaseRead(pI2C_Handle->pI2Cx, pI2C_Handle->SlaveAddr);
		}
	}

	/* Check ADDR flag */
	check_event = (pI2C_Handle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_ADDR));
	if (check_ITEVTEN && check_event)
	{
		// Clear ADDR flag
		I2C_ClearADDRFlag(pI2C_Handle);
	}

	/* Check BTF flag */
	check_event = (pI2C_Handle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_BTF));
	if (check_ITEVTEN && check_event)
	{
		// If BTF flag is set in Tx, means all data has transfered -> Close the communication
		if ( (pI2C_Handle->TxRxState == I2C_TX_BUSY) && (pI2C_Handle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_TxE)))
		{
			// Generate Stop condition & turn off interrupts
			if (pI2C_Handle->Sr == I2C_REPEATED_START_DISABLE)
			{
				I2C_GenerateStopCondition(pI2C_Handle->pI2Cx);
			}

			// Reset the Handle variable (it'll be ready if there is a next transmission)
			I2C_CloseSendData(pI2C_Handle);

			// Inform the application
			I2C_EventCallback(pI2C_Handle, I2C_EVENT_TX_CMPLT);
		}
		else if (pI2C_Handle->TxRxState == I2C_RX_BUSY)
		{
			// BTF is set when there's nothing to read. No action.
		}

	}

	/* Check STOPF flag (Slave mode only) - Clear this to close the communication in Slave mode (RM) */
	check_event = (pI2C_Handle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_STOPF));
	if (check_ITEVTEN && check_event)
	{
		// Write something to CR1 (that doesn't affect CR1) to clear this flag
		pI2C_Handle->pI2Cx->I2C_CR1 |= 0x00000000;

		I2C_EventCallback(pI2C_Handle, I2C_EVENT_STOPF);
	}

	/* Check TXE flag */
	check_event = (pI2C_Handle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_TxE));
	if (check_ITBUFEN && check_ITEVTEN && check_event)
	{
		// Handle data transmission in master mode or slave mode (must check)
		if (pI2C_Handle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_MSL))
		{
			// Master mode:
			if (pI2C_Handle->TxRxState == I2C_TX_BUSY)
				I2C_MasterTXE_IT_Handle(pI2C_Handle);
		}
		else
		{
			// Slave mode:
		}

	}

	/* Check RXNE flag */
	check_event = (pI2C_Handle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_RxNE));
	if (check_ITBUFEN && check_ITEVTEN && check_event)
	{
		// Handle data reception in master mode or slave mode (must check)
		if (pI2C_Handle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_MSL))
		{
			// Master mode:
			I2C_MasterRXNE_IT_Handle(pI2C_Handle);
		}
		else
		{
			// Slave mode:
		}
	}
}




void I2C_Error_IRQHandler(I2C_Handle_t *pI2C_Handle)
{
	/* Determine which error happened then inform the application */
	uint32_t check_ITERREN, check_error;

	/* Check that is the interrupt error bit enabled */
	check_ITERREN = pI2C_Handle->pI2Cx->I2C_CR2 & (1 << I2C_CR2_ITERREN);


	/* Check Bus error flag */
	check_error = pI2C_Handle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_BERR);
	if (check_ITERREN && check_error)
	{
		// Clear the error flag
		pI2C_Handle->pI2Cx->I2C_SR1 &= ~(1 << I2C_SR1_BERR);
		// Inform the application
		I2C_EventCallback(pI2C_Handle, I2C_ERROR_BERR);
	}


	/* Check Arbitration loss flag */
	check_error = pI2C_Handle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_ARLO);
	if (check_ITERREN && check_error)
	{
		pI2C_Handle->pI2Cx->I2C_SR1 &= ~(1 << I2C_SR1_ARLO);

		I2C_EventCallback(pI2C_Handle, I2C_ERROR_ARLO);
	}

	/* Check ACK failure flag */
	check_error = pI2C_Handle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_AF);
	if (check_ITERREN && check_error)
	{
		pI2C_Handle->pI2Cx->I2C_SR1 &= ~(1 << I2C_SR1_AF);

		I2C_EventCallback(pI2C_Handle, I2C_ERROR_AF);
	}

	/* Check Overrun/Underrun flag */
	check_error = pI2C_Handle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_OVR);
	if (check_ITERREN && check_error)
	{
		pI2C_Handle->pI2Cx->I2C_SR1 &= ~(1 << I2C_SR1_OVR);

		I2C_EventCallback(pI2C_Handle, I2C_ERROR_OVR);
	}

	/* Check Timeout flag */
	check_error = pI2C_Handle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_TIMEOUT);
	if (check_ITERREN && check_error)
	{
		pI2C_Handle->pI2Cx->I2C_SR1 &= ~(1 << I2C_SR1_TIMEOUT);

		I2C_EventCallback(pI2C_Handle, I2C_ERROR_TIMEOUT);
	}

}

void I2C_IRQInterruptConfig(uint8_t IRQ_Number, uint8_t EnorDi)
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

void I2C_IRQPriorityConfig(uint8_t IRQ_Number, uint8_t Priority)
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
	NVIC->NVIC_IPR[NVIC_IPRx] |= (Priority << shift_amount);
}


__weak void I2C_EventCallback(I2C_Handle_t *pI2C_Handle, uint8_t Event)
{

}


