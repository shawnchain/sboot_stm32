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
#include "printf.h"

#include "usart.h"
#include "swo.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

usart_rx_fifo_t rxFIFO1 = {.size=USART_RX_FIFO_SIZE};
usart_tx_fifo_t txFIFO1 = {.size=USART_TX_FIFO_SIZE};

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

int usart_write(struct usart_device * usart, const unsigned char* bytes, int len) {
    if (bytes == NULL || len == 0)
        return 0;

    int count = 0;
    usart_tx_fifo_t *txfifo = usart->txfifo;
    while(!RB_FULL(*txfifo) && count < len){
        RB_PUT(*txfifo, bytes[count]);
        count++;
    }

    return count;
}

int usart_print(struct usart_device * usart, const char* str) {
    if (str == NULL)
        return 0;

    return usart_write(usart, (unsigned char*)str, strlen(str));
}

int usart_printf(struct usart_device * usart, const char* fmt, ...) {
#if defined(ENABLE_TINY_PRINTF)
    char buf[128];
    va_list va;
    va_start(va, fmt);
    int ret;
    ret = vsnprintf(buf, sizeof(buf), fmt, va);
    va_end(va);
    usart_write(usart, (unsigned char*)buf, ret);
    return ret;
#else
    // printf is disabled
    return usart_print(usart, fmt);
#endif
}

static int usart_read_(struct usart_device * usart, unsigned char* bytes, int len, bool force_read) {
    if (bytes == NULL || len == 0)
        return 0;

    int count = 0;
    usart_rx_fifo_t *rxfifo = usart->rxfifo;

    if (!force_read && RB_COUNT(*rxfifo) < len)
        return 0;

    while(!RB_EMPTY(*rxfifo) && count < len) {
        bytes[count] = RB_GET(*rxfifo);
        count++;
    }

    if(count > 0 && RB_EMPTY(*rxfifo))
        RB_RESET(*rxfifo);

    return count;
}

int usart_read(struct usart_device * usart, unsigned char* bytes, int len) {
    return usart_read_(usart, bytes, len, false);
}

int usart_read_force(struct usart_device * usart, unsigned char* bytes, int len) {
    return usart_read_(usart, bytes, len, true);
}

int usart_rx_has_data(struct usart_device * usart) {
    usart_rx_fifo_t *rxfifo = usart->rxfifo;
    return RB_COUNT(*rxfifo);
}

#endif