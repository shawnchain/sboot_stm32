/**
  * @file sha256.h
  * @brief SHA-256 (Secure Hash Algorithm 256)
  *
  * @section License
  *
  * SPDX-License-Identifier: GPL-2.0-or-later
  *
  * Copyright (C) 2010-2021 Oryx Embedded SARL. All rights reserved.
  *
  * This file is part of CycloneCRYPTO Open.
  *
  * This program is free software; you can redistribute it and/or
  * modify it under the terms of the GNU General Public License
  * as published by the Free Software Foundation; either version 2
  * of the License, or (at your option) any later version.
  *
  * This program is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  * GNU General Public License for more details.
  *
  * You should have received a copy of the GNU General Public License
  * along with this program; if not, write to the Free Software Foundation,
  * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
  *
  * @author Oryx Embedded SARL (www.oryx-embedded.com)
  * @version 2.0.4
  **/

#ifndef _SHA256_H
#define _SHA256_H

#include <stdint.h>

//SHA-256 block size
#define SHA256_BLOCK_SIZE 64
//SHA-256 digest size
#define SHA256_DIGEST_SIZE 32
//Minimum length of the padding string
#define SHA256_MIN_PAD_SIZE 9

//C++ guard
#ifdef __cplusplus
extern "C"
{
#endif

    //SHA-256 related functions
    int sha256_data(const void *data, int length, uint8_t *digest);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif