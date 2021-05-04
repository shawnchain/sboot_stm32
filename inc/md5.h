/**
  * @file md5.h
  * @brief MD5 (Message-Digest Algorithm)
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

#ifndef _MD5_H
#define _MD5_H

#include <stdint.h>

//MD5 block size
#define MD5_BLOCK_SIZE 64
//MD5 digest size
#define MD5_DIGEST_SIZE 16
//Minimum length of the padding string
#define MD5_MIN_PAD_SIZE 9

//C++ guard
#ifdef __cplusplus
extern "C"
{
#endif

    //MD5 related functions
    int md5_data(const void *data, int length, uint8_t digest[16]);

//C++ guard
#ifdef __cplusplus
}
#endif

#endif