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

#ifndef USART_H
#define USART_H

#ifndef WEAK
#define WEAK __attribute__ ((weak))
#endif

struct usart_device;
typedef void (*usart_callback)(struct usart_device * usart, int count);
struct usart_device {
    void * rxfifo;
    void * txfifo;
    void * info;
    usart_callback rxcb;
    usart_callback txcb;
};

void usart_init(struct usart_device * usart, int speed, usart_callback rxcb, usart_callback txcb);

void usart_deinit(struct usart_device * usart);

void usart_enable(struct usart_device * usart);

void usart_disable(struct usart_device * usart);

int usart_read(struct usart_device * usart, char* bytes, int len);

int usart_rx_has_data(struct usart_device * usart);

int usart_write(struct usart_device * usart, const char* bytes, int len);

void usart_poll(struct usart_device * usart);

#endif