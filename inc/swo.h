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

#ifndef SWO_H
#define SWO_H

#if defined(ENABLE_TRACE) && (defined(STM32F1) || defined(STM32F4))

#include "config.h"

#ifndef WEAK
#define WEAK __attribute__ ((weak))
#endif

void WEAK swo_init(void);
int  WEAK swo_print(const char* s);
int  WEAK swo_printf(const char* fmt, ...);

#if defined(ENABLE_TRACE_PRINTF)
#define  TRACE(msg,...)  swo_printf(msg "\r\n", ## __VA_ARGS__)
#elif defined(ENABLE_TRACE)
#define  TRACE(msg,...)  swo_print(msg "\r\n", ## __VA_ARGS__)
#else
#define  TRACE(...)  do {} while(0)
#endif  /* #if defined(ENABLE_TRACE) */

#else
#define  TRACE(...)  do {} while(0)
#endif

#endif