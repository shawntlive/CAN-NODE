/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 8;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_8TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
CanRxMsg RxMessage1[CAN_BUF_RX_SIZE];
CanTxMsg TxMessage1[CAN_BUF_TX_SIZE];

uint8_t can_buf_rx_head = 0;
uint8_t can_buf_tx_tail = 0;
uint8_t can_buf_rx_tail = 0;
uint8_t can_buf_tx_head = 0;

uint32_t TxMailboxUsed;

/*
** Used internally to place a CAN data frame into a CAN transmit mailbox.
*/
void can_mbox_write ( CanTxMsg *TxMessage )
{
	/* clear the CAN transmit interrupt (NOTE: this shouldn't be done at the
	bottom of the routine because CPU doesn't clear bit soon enough and
	generates second ISR */

	HAL_CAN_AddTxMessage(&hcan1, &TxMessage->Header, &TxMessage->DataSend[0], &TxMailboxUsed);
}

/*
** Buffers a CAN data frame for transmission.
** INPUT: frame - pointer to CAN data frame
** RETURN: 0 - success
**         1 - failure
*/
uint8_t can_tx (CanTxMsg *TxMessage)
{
	/* sanity check */
	if( TxMessage->Header.DLC > 8 )
	 return (1);

	uint32_t tsr = READ_REG(hcan1.Instance->TSR);

	/* since the below global memory is shared with the tx isr, disable isrs */
	//CAN_LOCK();

	/* if transmission is active we simply place this frame in the transmit
	 buffer, else we write the frame to a mailbox */
    /* Check that all the Tx mailboxes are not full */
    if (((tsr & CAN_TSR_TME0) == 1U) ||
        ((tsr & CAN_TSR_TME1) == 1U) ||
        ((tsr & CAN_TSR_TME2) == 1U))
	{

    	can_mbox_write( TxMessage );


	}
	else
	{
    	/* move CAN frame into the current open index */
    	TxMessage1[can_buf_tx_head] = *TxMessage;

    	can_buf_tx_head++;

    	/* move on to next index in buffer */
    	if (can_buf_tx_head > (CAN_BUF_TX_SIZE - 1) )
    		can_buf_tx_head = 0;
	}

	//CAN_UNLOCK();

	return (0);
}

/*
** Reads a CAN data frame from receive buffer.
** INPUT: frame - pointer to CAN data frame
** RETURN: 0 - success
**         1 - failure
*/
uint8_t can_rx (CanRxMsg *RxMessage)
{
	/* since the below global memory is shared with the rx isr, disable isrs */
	//CAN_LOCK();

	if(can_buf_rx_tail > CAN_BUF_RX_SIZE) // If memory gets corrupted
	   can_buf_rx_tail = 0;

	if(can_buf_rx_head > CAN_BUF_RX_SIZE) // If memory gets corrupted
		can_buf_rx_head = 0;

	/* if there is something in the buffer, return it */
	if( can_buf_rx_tail != can_buf_rx_head )
	{
		/* copy over the new can frame from current buffer location */
		*RxMessage = RxMessage1[can_buf_rx_tail];

		can_buf_rx_tail++;

		/* move on to next index in buffer */
		if(can_buf_rx_tail > (CAN_BUF_RX_SIZE - 1))
		   can_buf_rx_tail = 0;

		if(RxMessage->Header.DLC == 0x65D)
		{

		}

		return (0);
	}

	//CAN_UNLOCK();

	return (1);
}

/* ===============================================================================
 *
 * 	Call backs from CAN interrupts
 *
 * ============================================================================== */
/**
  * @brief  Transmission Mailbox 0 complete callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
	CanTxMsg *TxMessage;

	// Check if more messages need to be sent then load current mailbox
	if (can_buf_tx_tail != can_buf_tx_head)
	{
		/* point to current message to tx */
		TxMessage = (CanTxMsg *)&TxMessage1[can_buf_tx_tail];

		CLEAR_BIT(hcan->Instance->sTxMailBox[0].TIR, CAN_TI0R_TXRQ);

		hcan->Instance->sTxMailBox[0].TIR = ((TxMessage->Header.ExtId << CAN_TI0R_EXID_Pos) |
												TxMessage->Header.IDE |
												TxMessage->Header.RTR);

		/* Set up the DLC */
		hcan->Instance->sTxMailBox[0].TDTR = (TxMessage->Header.DLC);

		/* Set up the Transmit Global Time mode */
		if (TxMessage->Header.TransmitGlobalTime == ENABLE)
		{
			SET_BIT(hcan->Instance->sTxMailBox[0].TDTR, CAN_TDT0R_TGT);
		}

		/* Set up the data field */
		WRITE_REG(hcan->Instance->sTxMailBox[0].TDHR,
					((uint32_t)TxMessage->DataSend[7] << CAN_TDH0R_DATA7_Pos) |
					((uint32_t)TxMessage->DataSend[6] << CAN_TDH0R_DATA6_Pos) |
					((uint32_t)TxMessage->DataSend[5] << CAN_TDH0R_DATA5_Pos) |
					((uint32_t)TxMessage->DataSend[4] << CAN_TDH0R_DATA4_Pos));
		WRITE_REG(hcan->Instance->sTxMailBox[0].TDLR,
					((uint32_t)TxMessage->DataSend[3] << CAN_TDL0R_DATA3_Pos) |
					((uint32_t)TxMessage->DataSend[2] << CAN_TDL0R_DATA2_Pos) |
					((uint32_t)TxMessage->DataSend[1] << CAN_TDL0R_DATA1_Pos) |
					((uint32_t)TxMessage->DataSend[0] << CAN_TDL0R_DATA0_Pos));

		/* Request transmission */
		SET_BIT(hcan->Instance->sTxMailBox[0].TIR, CAN_TI0R_TXRQ);

		can_buf_tx_tail++;

		if (can_buf_tx_tail > (CAN_BUF_TX_SIZE - 1))
			can_buf_tx_tail = 0;
	}
}

/**
  * @brief  Transmission Mailbox 1 complete callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
	CanTxMsg *TxMessage;

	// Check if more messages need to be sent then load current mailbox
	if (can_buf_tx_tail != can_buf_tx_head)
	{
		/* point to current message to tx */
		TxMessage = (CanTxMsg *)&TxMessage1[can_buf_tx_tail];

		CLEAR_BIT(hcan->Instance->sTxMailBox[1].TIR, CAN_TI0R_TXRQ);

		hcan->Instance->sTxMailBox[1].TIR = ((TxMessage->Header.ExtId << CAN_TI0R_EXID_Pos) |
												TxMessage->Header.IDE |
												TxMessage->Header.RTR);

		/* Set up the DLC */
		hcan->Instance->sTxMailBox[1].TDTR = (TxMessage->Header.DLC);

		/* Set up the Transmit Global Time mode */
		if (TxMessage->Header.TransmitGlobalTime == ENABLE)
		{
			SET_BIT(hcan->Instance->sTxMailBox[1].TDTR, CAN_TDT0R_TGT);
		}

		/* Set up the data field */
		WRITE_REG(hcan->Instance->sTxMailBox[1].TDHR,
					((uint32_t)TxMessage->DataSend[7] << CAN_TDH0R_DATA7_Pos) |
					((uint32_t)TxMessage->DataSend[6] << CAN_TDH0R_DATA6_Pos) |
					((uint32_t)TxMessage->DataSend[5] << CAN_TDH0R_DATA5_Pos) |
					((uint32_t)TxMessage->DataSend[4] << CAN_TDH0R_DATA4_Pos));
		WRITE_REG(hcan->Instance->sTxMailBox[1].TDLR,
					((uint32_t)TxMessage->DataSend[3] << CAN_TDL0R_DATA3_Pos) |
					((uint32_t)TxMessage->DataSend[2] << CAN_TDL0R_DATA2_Pos) |
					((uint32_t)TxMessage->DataSend[1] << CAN_TDL0R_DATA1_Pos) |
					((uint32_t)TxMessage->DataSend[0] << CAN_TDL0R_DATA0_Pos));

		/* Request transmission */
		SET_BIT(hcan->Instance->sTxMailBox[1].TIR, CAN_TI0R_TXRQ);

		can_buf_tx_tail++;

		if (can_buf_tx_tail > (CAN_BUF_TX_SIZE - 1))
			can_buf_tx_tail = 0;
	}
}

/**
  * @brief  Transmission Mailbox 2 complete callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
	CanTxMsg *TxMessage;

	// Check if more messages need to be sent then load current mailbox
	if (can_buf_tx_tail != can_buf_tx_head)
	{
		/* point to current message to tx */
		TxMessage = (CanTxMsg *)&TxMessage1[can_buf_tx_tail];

		CLEAR_BIT(hcan->Instance->sTxMailBox[2].TIR, CAN_TI0R_TXRQ);

		hcan->Instance->sTxMailBox[2].TIR = ((TxMessage->Header.ExtId << CAN_TI0R_EXID_Pos) |
												TxMessage->Header.IDE |
												TxMessage->Header.RTR);

		/* Set up the DLC */
		hcan->Instance->sTxMailBox[2].TDTR = (TxMessage->Header.DLC);

		/* Set up the Transmit Global Time mode */
		if (TxMessage->Header.TransmitGlobalTime == ENABLE)
		{
			SET_BIT(hcan->Instance->sTxMailBox[2].TDTR, CAN_TDT0R_TGT);
		}

		/* Set up the data field */
		WRITE_REG(hcan->Instance->sTxMailBox[2].TDHR,
					((uint32_t)TxMessage->DataSend[7] << CAN_TDH0R_DATA7_Pos) |
					((uint32_t)TxMessage->DataSend[6] << CAN_TDH0R_DATA6_Pos) |
					((uint32_t)TxMessage->DataSend[5] << CAN_TDH0R_DATA5_Pos) |
					((uint32_t)TxMessage->DataSend[4] << CAN_TDH0R_DATA4_Pos));
		WRITE_REG(hcan->Instance->sTxMailBox[2].TDLR,
					((uint32_t)TxMessage->DataSend[3] << CAN_TDL0R_DATA3_Pos) |
					((uint32_t)TxMessage->DataSend[2] << CAN_TDL0R_DATA2_Pos) |
					((uint32_t)TxMessage->DataSend[1] << CAN_TDL0R_DATA1_Pos) |
					((uint32_t)TxMessage->DataSend[0] << CAN_TDL0R_DATA0_Pos));

		/* Request transmission */
		SET_BIT(hcan->Instance->sTxMailBox[2].TIR, CAN_TI0R_TXRQ);

		can_buf_tx_tail++;

		if (can_buf_tx_tail > (CAN_BUF_TX_SIZE - 1))
			can_buf_tx_tail = 0;
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CanRxMsg *RxMessage;

	/* get pointer to the next open index */
	RxMessage = (CanRxMsg *)&RxMessage1[can_buf_rx_head];

	HAL_CAN_GetRxMessage(&hcan1, 0, &RxMessage->Header, &RxMessage->DataRecieved[0]);

	/* mark we have one more in the buffer */
	can_buf_rx_head++;

	/* move on to next index in buffer */
	if (can_buf_rx_head > (CAN_BUF_RX_SIZE - 1))
		can_buf_rx_head = 0;
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
