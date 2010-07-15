/*
 * $Id:$
 *
 * Copyright (C) 2010 Piotr Esden-Tempski <piotr@esden.net>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

#include <stdint.h>
#include <string.h>

#include "can_hw.h"

#include <stm32/rcc.h>
#include <stm32/gpio.h>
#include <stm32/flash.h>
#include <stm32/misc.h>
#include <stm32/can.h>

#include "led.h"

#define RCC_APB2Periph_GPIO_CAN RCC_APB2Periph_GPIOA
#define GPIO_CAN GPIOA
#define GPIO_Pin_CAN_RX GPIO_Pin_11
#define GPIO_Pin_CAN_TX GPIO_Pin_12

CanTxMsg can_tx_msg;
RCC_ClocksTypeDef rcc_clocks;

void can_hw_init(void)
{
	GPIO_InitTypeDef gpio;
	NVIC_InitTypeDef nvic;
	CAN_InitTypeDef can;
	CAN_FilterInitTypeDef can_filter;

	/* Enable peripheral clocks */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO |
			       RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO |
	//		       RCC_APB2Periph_GPIO_CAN, ENABLE);
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

	/* Configure CAN pin: RX */
	gpio.GPIO_Pin = GPIO_Pin_11;
	gpio.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &gpio);

	/* Configure CAN pin: TX */
	gpio.GPIO_Pin = GPIO_Pin_12;
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpio);

	/* NVIC configuration */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

	nvic.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 0x00;
	nvic.NVIC_IRQChannelSubPriority = 0x00;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);

	/* get rcc clocks */
	RCC_GetClocksFreq(&rcc_clocks);

	/* CAN register init */
	CAN_DeInit(CAN1);
	CAN_StructInit(&can);

	/* CAN cell init */
	can.CAN_TTCM = DISABLE;
	can.CAN_ABOM = DISABLE;
	can.CAN_AWUM = DISABLE;
	can.CAN_NART = DISABLE;
	can.CAN_RFLM = DISABLE;
	can.CAN_TXFP = DISABLE;
	can.CAN_Mode = CAN_Mode_Normal;
	can.CAN_SJW = CAN_SJW_1tq;
	can.CAN_BS1 = CAN_BS1_3tq;
	can.CAN_BS2 = CAN_BS2_5tq;
	can.CAN_Prescaler = 4;
	CAN_Init(CAN1, &can);

	/* CAN filter init */
	can_filter.CAN_FilterNumber = 0;
	can_filter.CAN_FilterMode = CAN_FilterMode_IdMask;
	can_filter.CAN_FilterScale = CAN_FilterScale_32bit;
	can_filter.CAN_FilterIdHigh = 0x0000;
	can_filter.CAN_FilterIdLow = 0x0000;
	can_filter.CAN_FilterMaskIdHigh = 0x0000;
	can_filter.CAN_FilterMaskIdLow = 0x0000;
	can_filter.CAN_FilterFIFOAssignment = 0;
	can_filter.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&can_filter);

	/* transmit struct init */
	can_tx_msg.StdId = 0x321;
	can_tx_msg.ExtId = 0x01;
	can_tx_msg.RTR = CAN_RTR_DATA;
	can_tx_msg.IDE = CAN_ID_STD;
	can_tx_msg.DLC = 1;

	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);

	can_tx_msg.Data[0] = 0x55;

	CAN_Transmit(CAN1, &can_tx_msg);
}

int can_hw_transmit(uint16_t id, const uint8_t *buf, uint8_t len)
{
	if(len > 8){
		return -1;
	}

//	can_tx_msg.StdId = id >> 8;
//	can_tx_msg.ExtId = id & 0xFF;
//	can_tx_msg.DLC = len;
//	memcpy(&can_tx_msg.Data, buf, len);
//
//	CAN_Transmit(CAN1, &can_tx_msg);

	//can_tx_msg.StdId = 0;
	//can_tx_msg.ExtId = 0x1234;
	//can_tx_msg.DLC = 1;
	//can_tx_msg.Data[0] = 0x55;

	CAN_Transmit(CAN1, &can_tx_msg);

	return 0;
}
