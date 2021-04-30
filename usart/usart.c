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

#ifdef ENABLE_USART

#include "usart.h"
#include "swo.h"

#include <stdint.h>
#include <string.h>

// RingBuffer code, size should in power 2 bytes
#define RB_SIZE(rb) ((rb).size)    // size in bytes (power 2)
#define RB_MASK(rb) ((rb).size-1U) // buffer size mask for fast access

// Buffer read / write macros
#define RB_RESET(rb)         (rb).rdIdx = (rb).wrIdx = 0
#define RB_PUT(rb, dataIn)   (rb).data[RB_MASK(rb) & ((rb).wrIdx++)] = (dataIn)
#define RB_GET(rb)           ((rb).data[RB_MASK(rb) & ((rb).rdIdx++)])
#define RB_EMPTY(rb)         ((rb).rdIdx == (rb).wrIdx)
#define RB_FULL(rb)          (((rb).wrIdx > (rb).rdIdx) && (RB_MASK(rb) & (rb).rdIdx) == (RB_MASK(rb) & (rb).wrIdx))
#define RB_COUNT(rb)         (RB_MASK(rb) & ((rb).wrIdx - (rb).rdIdx))

// Declare the buffer type
#define DECL_RB_TYPE(name, _size)                           \
        typedef struct{                                     \
            uint32_t size;                                  \
            uint32_t wrIdx;                                 \
            uint32_t rdIdx;                                 \
            uint8_t data[_size];                            \
        }name##_t

#define USART_FIFO_SIZE 128
// USART FIFO type
DECL_RB_TYPE(usart_fifo, USART_FIFO_SIZE);
usart_fifo_t rxFIFO1 = {.size=USART_FIFO_SIZE};
usart_fifo_t txFIFO1 = {.size=USART_FIFO_SIZE};

void usart_init_int(struct usart_device * usart, int speed);

// USART Code
void usart_init(struct usart_device * usart, int speed, usart_callback rxcb, usart_callback txcb) {
    RB_RESET(rxFIFO1);
    RB_RESET(txFIFO1);

    memset(usart, 0, sizeof(struct usart_device));
    usart->rxfifo = &rxFIFO1;
    usart->txfifo = &txFIFO1;
    usart->rxcb = rxcb;
    usart->txcb = txcb;

    usart_init_int(usart, speed);
}

int usart_write(struct usart_device * usart, const char* bytes, int len) {
    if (bytes == NULL || len == 0)
        return 0;

    int count = 0;
    usart_fifo_t *txfifo = (usart_fifo_t*)usart->txfifo;
    while(!RB_FULL(*txfifo) && count < len){
        RB_PUT(*txfifo, bytes[count]);
        count++;
    }

    return count;
}

int usart_read(struct usart_device * usart, char* bytes, int len) {
    if (bytes == NULL || len == 0)
        return 0;

    int count = 0;
    usart_fifo_t *rxfifo = usart->rxfifo;
    while(!RB_EMPTY(*rxfifo) && count < len) {
        bytes[count] = RB_GET(*rxfifo);
        count++;
    }

    if(count > 0)
        RB_RESET(*rxfifo);

    return count;
}

int usart_rx_has_data(struct usart_device * usart) {
    usart_fifo_t *rxfifo = usart->rxfifo;
    return RB_COUNT(*rxfifo);
}

#if defined(STM32F1)
#include "usart_f1.c"
#endif

#if defined(STM32F4)
#include "usart_f4.c"
#endif

#endif