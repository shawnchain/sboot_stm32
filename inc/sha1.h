/**
  * @file sha1.h
  * @brief SHA-1 (Secure Hash Algorithm 1)
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

#ifndef _SHA1_H
#define _SHA1_H

#include "stdint.h"

//SHA-1 block size
#define SHA1_BLOCK_SIZE 64
//SHA-1 digest size
#define SHA1_DIGEST_SIZE 20
//Minimum length of the padding string
#define SHA1_MIN_PAD_SIZE 9
//SHA-1 algorithm object identifier
#define SHA1_OID sha1Oid
//Common interface for hash algorithms
#define SHA1_HASH_ALGO (&sha1HashAlgo)

//C++ guard
#ifdef __cplusplus
extern "C"
{
#endif

int sha1_data(const void *data, unsigned int len, uint8_t *digest);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif