/*
 *   Copyright (C) 2021 by BG5HHP
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include "config.h"

#if defined(STM32F4) && defined(ENABLE_USART)
#include "usart.h"
#include "swo.h"

#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_misc.h"

void usart_init_int(struct usart_device * usart, int speed) {
    // USART1 - TXD PA9  - RXD PA10
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

    // Configure USART as alternate function
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //  Tx | Rx
    GPIO_InitStructure.GPIO_Speed = GPIO_Fast_Speed;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Configure USART baud rate
    USART_StructInit(&USART_InitStructure);
    USART_InitStructure.USART_BaudRate = speed;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);

    USART_Cmd(USART1, ENABLE);

#ifdef ENABLE_USART_IRQ
    // USART IRQ init
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
#endif

    TRACE("NewDrv: Init USART1");
}

const int TX_WAIT_TIMEOUT = 36000;
void usart_poll(struct usart_device * usart) {
    if (usart == 0)
        return;

    // Sending(polling mode)
    int txcount = 0;
    usart_fifo_t *txfifo = (usart_fifo_t*)usart->txfifo;

    while (!RB_EMPTY(*txfifo) && (USART1->SR & USART_FLAG_TXE) > 0) {
        // USART_SendData(USART1, RB_GET(*txfifo));
        USART1->DR = RB_GET(*txfifo);
        txcount++;
        // wait until send is complete
        int wait = 0;
        while(wait < TX_WAIT_TIMEOUT && (USART1->SR & USART_FLAG_TC) > 0){
            wait++;
        }

        if (wait == TX_WAIT_TIMEOUT)  // about 1ms wait time out.
            break;
    }

    if (txcount > 0 && usart->txcb) {
        usart->txcb(usart, txcount);
    }

    // Receiving
    int rxcount = 0;
    usart_fifo_t *rxfifo = (usart_fifo_t*)usart->rxfifo;
    while (!RB_FULL(*rxfifo) && (USART1->SR & USART_FLAG_RXNE) > 0) {
        RB_PUT(*rxfifo, USART1->DR);
        rxcount++;
    }

    if (rxcount > 0 && usart->rxcb) {
        usart->rxcb(usart, rxcount);
    }
}

#ifdef ENABLE_USART_IRQ
void USART1_IRQHandler()
{
    if (USART_GetITStatus(USART1, USART_IT_TXE))
    {
        if (!txFIFO1.empty())
        {
            USART_SendData(USART1, txFIFO1.get());
        }
        else
        { // if there's no more data to transmit then turn off TX interrupts
            USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
        }
    }

    if (USART_GetITStatus(USART1, USART_IT_RXNE))
    {
        rxFIFO1.put(USART_ReceiveData(USART1));
    }
}
#endif

#endif