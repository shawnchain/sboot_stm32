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

#ifndef RINGBUFFER_H
#define RINGBUFFER_H

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
#define DECL_RB_TYPE(_type_name, _size)                           \
        typedef struct{                                     \
            unsigned int size;                                  \
            unsigned int wrIdx;                                 \
            unsigned int rdIdx;                                 \
            unsigned char data[_size];                            \
        }_type_name

#endif