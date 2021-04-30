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

#if defined(STM32F1) && defined(ENABLE_USART)
#include "usart.h"
#include "stm32.h"

#define USART_DIV(__PCLK__, __BAUD__)     (((__PCLK__)*25)/(4*(__BAUD__)))
#define USART_DIVMANT(__PCLK__, __BAUD__) (USART_DIV((__PCLK__), (__BAUD__))/100)
#define USART_DIVFRAQ(__PCLK__, __BAUD__) (((USART_DIV((__PCLK__), (__BAUD__)) - (USART_DIVMANT((__PCLK__), (__BAUD__)) * 100)) * 16 + 50) / 100)
#define USART_BRR(__PCLK__, __BAUD__)     ((USART_DIVMANT((__PCLK__), (__BAUD__)) << 4)|(USART_DIVFRAQ((__PCLK__), (__BAUD__)) & 0x0F))

typedef volatile uint32_t * const bitband_t;
/* periph bit band */
#define BITBAND_PERIPH(address, bit)   ((void *)(BITBAND_PERIPH_BASE + \
		(((uint32_t)address) - BITBAND_PERIPH_REF) * 32 + (bit) * 4))

#define PIN_USART1_TXD    9
#define PORT_USART1_TXD   GPIOA
#define PIN_USART1_RXD    10
#define PORT_USART1_RXD   GPIOA

static void GPIOConfigPin(GPIO_TypeDef *port_ptr, uint32_t pin, uint32_t mode_cnf_value)
{
	volatile uint32_t *cr_ptr;
	uint32_t cr_value;

	cr_ptr = &port_ptr->CRL;  // configuration of pins [0,7] is in CRL

	if (pin >= 8)			        // is pin in [8; 15]?
	{									        // configuration of pins [8,15] is in CRH
		cr_ptr++;               // advance to next struct element CRL -> CRH
		pin -= 8;               // crop the pin number
	}

	cr_value = *cr_ptr;			  // localize the CRL / CRH value

	cr_value &= ~(0xF << (pin * 4));	// clear the MODE and CNF fields (now that pin is an analog input)
	cr_value |= (mode_cnf_value << (pin * 4));	        // save new MODE and CNF value for desired pin

	*cr_ptr = cr_value;				// save localized value to CRL / CRL
}

void usart_init_int(struct usart_device * usart, int speed) {
    // gpio init, set GPIOA pins to input with pull-down
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	// GPIOA->CRL = 0x88888888;
	// GPIOA->CRH = 0x88888888;
	// GPIOA->ODR = 0;

    // usart pin config
    GPIOConfigPin(PORT_USART1_TXD, PIN_USART1_TXD, GPIO_CRL_MODE0_1|GPIO_CRL_CNF0_1);
    GPIOConfigPin(PORT_USART1_RXD, PIN_USART1_RXD, GPIO_CRL_CNF0_0);

    // usart init
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    USART1->BRR = USART_BRR(SYS_CLOCK /*48000000UL*/, speed);
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE |
                  USART_CR1_RE ; // Enable USART

    // NVIC_EnableIRQ(USART1_IRQn);
}

const int TX_WAIT_TIMEOUT = 36000;
void usart_poll(struct usart_device * usart) {
    if (usart == 0)
        return;

    // Sending(polling mode)
    int txcount = 0;
    usart_fifo_t *txfifo = (usart_fifo_t*)usart->txfifo;
    while (!RB_EMPTY(*txfifo) && (USART1->SR & USART_SR_TXE) > 0) {
        // USART_SendData(USART1, RB_GET(*txfifo));
        USART1->DR = RB_GET(*txfifo);
        txcount++;
        // wait until send is complete
        int wait = 0;
        while(wait < TX_WAIT_TIMEOUT && (USART1->SR & USART_SR_TC) > 0){
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
    while (!RB_FULL(*rxfifo) && (USART1->SR & USART_SR_RXNE) > 0) {
        RB_PUT(*rxfifo, USART1->DR);
        rxcount++;
    }

    if (rxcount > 0 && usart->rxcb) {
        usart->rxcb(usart, rxcount);
    }
}

#endif