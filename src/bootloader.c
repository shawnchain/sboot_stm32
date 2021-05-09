/* This file is the part of the STM32 secure bootloader
 *
 * Copyright ©2016 Dmitry Filimonchuk <dmitrystu[at]gmail[dot]com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *   http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "config.h"
#include "stm32.h"
#include "usart.h"
#include "swo.h"
#include "usb.h"
#include "usb_dfu.h"
#include "descriptors.h"
#include "flash.h"
#include "crypto.h"
#include "checksum.h"
#include "misc.h"

#include "md5.h"
#include "sha1.h"
#include "sha256.h"

#include <string.h>

#define EOL "\r\n"

extern void System_Reset(void);

/* Checking for the EEPROM */
#if defined(DATA_EEPROM_BASE)
    #define _EE_START    DATA_EEPROM_BASE
    #define _EE_LENGTH   (DATA_EEPROM_END - DATA_EEPROM_BASE + 1)
#elif defined(FLASH_EEPROM_BASE)
    #define _EE_START    FLASH_EEPROM_BASE
    #define _EE_LENGTH   (FLASH_EEPROM_END - FLASH_EEPROM_BASE + 1 )
#endif

#if (DFU_INTF_EEPROM == _ENABLE) && !defined(_EE_START)
    #error No EEPROM found. Check config !!
#elif ((DFU_INTF_EEPROM == _AUTO) || (DFU_INTF_EEPROM == _ENABLE)) && defined(_EE_START)
    #define _EEPROM_ENABLED
#endif

/* Checking for application start address */
#if (DFU_APP_START == _AUTO)
    #define _APP_START  ((size_t)&__app_start)
#elif ((DFU_APP_START & 0x000007FF) == 0)
    #define _APP_START  DFU_APP_START
#else
    #error DFU_APP_START must be 2k aligned. Check config !!
#endif

/* Checking for application size */
#if (DFU_APP_SIZE == _AUTO)
    #define _APP_LENGTH ((size_t)&__romend - _APP_START)
#else
    #define _APP_LENGTH DFU_APP_SIZE
#endif

/* DFU request buffer size data + request header */
#define DFU_BUFSZ  ((DFU_BLOCKSZ + 3 + 8) >> 2)

extern uint8_t  __app_start;
extern uint8_t  __romend;

static uint32_t dfu_buffer[DFU_BUFSZ];
#if defined(ENABLE_USB)
static usbd_device dfu;

static struct dfu_data_s {
    uint8_t     (*flash)(void *romptr, const void *buf, size_t blksize);
    void        *dptr;
    size_t      remained;
    uint8_t     interface;
    uint8_t     bStatus;
    uint8_t     bState;
} dfu_data;

/** Processing DFU_SET_IDLE request */
static usbd_respond dfu_set_idle(void) {
    aes_init();
    dfu_data.bState = USB_DFU_STATE_DFU_IDLE;
    dfu_data.bStatus = USB_DFU_STATUS_OK;
    switch (dfu_data.interface){
#if defined(_EEPROM_ENABLED)
    case 1:
        dfu_data.dptr = (void*)_EE_START;
        dfu_data.remained = _EE_LENGTH;
        dfu_data.flash = program_eeprom;
        break;
#endif
    default:
        dfu_data.dptr = (void*)_APP_START;
        dfu_data.remained = _APP_LENGTH;
        dfu_data.flash = program_flash;
        break;
    }
    return usbd_ack;
}

static usbd_respond dfu_err_badreq(void) {
    dfu_data.bState  = USB_DFU_STATE_DFU_ERROR;
    dfu_data.bStatus = USB_DFU_STATUS_ERR_STALLEDPKT;
    return usbd_fail;
}

static usbd_respond dfu_upload(usbd_device *dev, size_t blksize) {
    switch (dfu_data.bState) {
#if (DFU_CAN_UPLOAD == _ENABLE)
    case USB_DFU_STATE_DFU_IDLE:
    case USB_DFU_STATE_DFU_UPLOADIDLE:
        if (dfu_data.remained == 0) {
            dev->status.data_count = 0;
            return dfu_set_idle();
        } else if (dfu_data.remained < DFU_BLOCKSZ) {
            blksize = dfu_data.remained;
        }
        aes_encrypt(dev->status.data_ptr, dfu_data.dptr, blksize);
        dev->status.data_count = blksize;
        dfu_data.remained -= blksize;
        dfu_data.dptr += blksize;
        return usbd_ack;
#endif
    default:
        return dfu_err_badreq();
    }
}

static usbd_respond dfu_dnload(void *buf, size_t blksize) {
    switch(dfu_data.bState) {
    case    USB_DFU_STATE_DFU_DNLOADIDLE:
    case    USB_DFU_STATE_DFU_DNLOADSYNC:
    case    USB_DFU_STATE_DFU_IDLE:
        if (blksize == 0) {
            dfu_data.bState = USB_DFU_STATE_DFU_MANIFESTSYNC;
            return usbd_ack;
        }
        if (blksize > dfu_data.remained) {
            dfu_data.bStatus = USB_DFU_STATUS_ERR_ADDRESS;
            dfu_data.bState = USB_DFU_STATE_DFU_ERROR;
            return usbd_ack;
        }
        aes_decrypt(buf, buf, blksize );
        dfu_data.bStatus = dfu_data.flash(dfu_data.dptr, buf, blksize);

        if (dfu_data.bStatus == USB_DFU_STATUS_OK) {
            dfu_data.dptr += blksize;
            dfu_data.remained -= blksize;
#if (DFU_DNLOAD_NOSYNC == _ENABLED)
            dfu_data.bState = USB_DFU_STATE_DFU_DNLOADIDLE;
#else
            dfu_data.bState = USB_DFU_STATE_DFU_DNLOADSYNC;
#endif
            return usbd_ack;
        } else {
            dfu_data.bState = USB_DFU_STATE_DFU_ERROR;
            return usbd_ack;
        }
    default:
        return dfu_err_badreq();
    }
}

static usbd_respond dfu_getstatus(void *buf) {
    /* make answer */
    struct usb_dfu_status *stat = buf;
    stat->bStatus = dfu_data.bStatus;
    stat->bState = dfu_data.bState;
    stat->bPollTimeout = (DFU_POLL_TIMEOUT & 0xFF);
    stat->wPollTimeout = (DFU_POLL_TIMEOUT >> 8);
    stat->iString = NO_DESCRIPTOR;

    switch (dfu_data.bState) {
    case USB_DFU_STATE_DFU_IDLE:
    case USB_DFU_STATE_DFU_DNLOADIDLE:
    case USB_DFU_STATE_DFU_UPLOADIDLE:
    case USB_DFU_STATE_DFU_ERROR:
        return usbd_ack;
    case USB_DFU_STATE_DFU_DNLOADSYNC:
        dfu_data.bState = USB_DFU_STATE_DFU_DNLOADIDLE;
        return usbd_ack;
    case USB_DFU_STATE_DFU_MANIFESTSYNC:
        return dfu_set_idle();
    default:
        return dfu_err_badreq();
    }
}

static usbd_respond dfu_getstate(uint8_t *buf) {
    *buf = dfu_data.bState;
    return usbd_ack;
}

static usbd_respond dfu_abort() {
    switch (dfu_data.bState) {
    case USB_DFU_STATE_DFU_IDLE:
    case USB_DFU_STATE_DFU_DNLOADSYNC:
    case USB_DFU_STATE_DFU_DNLOADIDLE:
    case USB_DFU_STATE_DFU_MANIFESTSYNC:
    case USB_DFU_STATE_DFU_UPLOADIDLE:
        return dfu_set_idle();
    default:
        return dfu_err_badreq();
    }
}

static usbd_respond dfu_clrstatus() {
    if (dfu_data.bState == USB_DFU_STATE_DFU_ERROR)  {
        return dfu_set_idle();
    } else {
        return dfu_err_badreq();
    }
}

static void dfu_reset(usbd_device *dev, uint8_t ev, uint8_t ep) {
    (void)dev;
    (void)ev;
    (void)ep;
    /** TODO : add firmware checkout */
    System_Reset();
}

static usbd_respond dfu_control (usbd_device *dev, usbd_ctlreq *req, usbd_rqc_callback *callback) {
    (void)callback;
    if ((req->bmRequestType  & (USB_REQ_TYPE | USB_REQ_RECIPIENT)) == (USB_REQ_STANDARD | USB_REQ_INTERFACE)) {
        switch (req->bRequest) {
        case USB_STD_SET_INTERFACE:
            if (req->wIndex != 0) return usbd_fail;
            switch (req->wValue) {
            case 0: break;
#if defined(_EEPROM_ENABLED)
            case 1: break;
#endif
            default:
                return usbd_fail;
            }
            dfu_data.interface = req->wValue;
            return dfu_set_idle();
        case USB_STD_GET_INTERFACE:
            req->data[0] = dfu_data.interface;
            return usbd_ack;
        default:
            return usbd_fail;
        }
    }
    if ((req->bmRequestType & (USB_REQ_TYPE | USB_REQ_RECIPIENT)) == (USB_REQ_CLASS | USB_REQ_INTERFACE)) {
        switch (req->bRequest) {
#if (DFU_DETACH == _ENABLE)
        case USB_DFU_DETACH:
            *callback = (usbd_rqc_callback)dfu_reset;
            return usbd_ack;
#endif
        case USB_DFU_DNLOAD:
            return dfu_dnload(req->data, req->wLength);
        case USB_DFU_UPLOAD:
            return dfu_upload(dev, req->wLength);
        case USB_DFU_GETSTATUS:
            return dfu_getstatus(req->data);
        case USB_DFU_CLRSTATUS:
            return dfu_clrstatus();
        case USB_DFU_GETSTATE:
            return dfu_getstate(req->data);
        case USB_DFU_ABORT:
            return dfu_abort();
        default:
            return dfu_err_badreq();
        }
    }
    return usbd_fail;
}

static usbd_respond dfu_config(usbd_device *dev, uint8_t config) {
    switch (config) {
    case 0:
        usbd_reg_event(dev, usbd_evt_reset, 0);
        break;
    case 1:
        usbd_reg_event(dev, usbd_evt_reset, dfu_reset);
        break;
    default:
        return usbd_fail;
    }
    return usbd_ack;
}

static void dfu_init (void) {
    dfu_set_idle();
    usbd_init(&dfu, &usbd_hw, DFU_EP0_SIZE, dfu_buffer, sizeof(dfu_buffer));
    usbd_reg_config(&dfu, dfu_config);
    usbd_reg_control(&dfu, dfu_control);
    usbd_reg_descr(&dfu, dfu_get_descriptor);
    usbd_enable(&dfu, 1);
    usbd_connect(&dfu, 1);
}
#endif

//
// DFU SERIAL CODE
//

#if defined(ENABLE_USART)

enum {
    DFU_SERIAL_RX_STATE_HEAD = 0,
    DFU_SERIAL_RX_STATE_LEN1 = 1,
    DFU_SERIAL_RX_STATE_LEN2 = 2,
    DFU_SERIAL_RX_STATE_BODY = 3,
};

static struct usart_device usart;
static struct dfu_serial_s {
    uint8_t     (*flash)(void *romptr, const void *buf, size_t blksize);
    void        *dptr;
    size_t      remained;
    uint8_t     bStatus;

    uint8_t     rxState;
    uint16_t    rxDataSize;
    uint8_t    *rxBuf;
    uint32_t    rxBufOffset;
    uint32_t    rxBufSize;
    uint32_t    resetTimer;

    // uint8_t     flash_blk_size;
    // uint16_t    flash_blk_count;
    uint32_t    flash_data_size;
    uint32_t    flash_data_crc;
} dfu_serial_;


#if defined(STM32F1)
const uint32_t SERIAL_READ_BLOCK_TIMEOUT = 10000;
const uint32_t SERIAL_RESET_TIMEOUT = 250000;
#else
const uint32_t SERIAL_READ_BLOCK_TIMEOUT = 20000;
const uint32_t SERIAL_RESET_TIMEOUT = 500000;
#endif

/**
 * @brief CRC32
 * 
 */
#define CRC_POLY 0xEDB88320UL
#define CRC_INIT 0xFFFFFFFFUL
static void crc32_init(uint32_t *checksum) {
    *checksum = CRC_INIT;
}

static void crc32_update(uint32_t *checksum, uint8_t data) {
    *checksum ^= data;
    for (int i =0; i < 8; i++) {
        if (*checksum & 0x01) {
            *checksum = (*checksum >> 1) ^ CRC_POLY;
        } else {
            *checksum = (*checksum >> 1);
        }
    }
}

static uint32_t crc32_calc(const void *data, int len) {
    uint32_t cs = 0;
    const uint8_t *buf = data;
    crc32_init(&cs);
    while(len-- > 0) {
        crc32_update(&cs, *buf);
        buf++;
    }

    return cs;
}

// static int crc32_append(void *data, int len) {
//     uint32_t cs;
//     cs = crc32_calc(data, len);

//     data[len] = (cs >> 24) & 0xff;
//     data[len + 1] = (cs >> 16) & 0xff;
//     data[len + 2] = (cs >> 8) & 0xff;
//     data[len + 3] = cs & 0xff;
//     return 0;
// }

static void dfu_serial_init(uint8_t* buf, uint32_t bufSize) {
    memset(&dfu_serial_, 0, sizeof(dfu_serial_));

    dfu_serial_.rxBuf = buf;
    dfu_serial_.rxBufSize = bufSize;

    dfu_serial_.rxState = DFU_SERIAL_RX_STATE_HEAD;
    dfu_serial_.bStatus = 0;
}

#define SERIAL_ACK 0
#define SERIAL_NAK 1
#define SERIAL_ERR 2
#define SERIAL_IGN 255

/**
 * @brief dfu serial ack
 * 
 * @param usart 
 * @param type 
 * @param error 
 * @param msg 
 */
static void dfu_serial_ack(struct usart_device *usart, uint8_t type, uint8_t error, const char* msg) {
    int msglen = 0;
    if (msg)
        msglen = strlen(msg);

    uint8_t buf[64];
    buf[0] = 0xFE;
    buf[1] = 0x00;
    buf[2] = 0;
    buf[3] = type;
    buf[4] = error;

    int i = 5;
    if (msglen > 0){
        const char *str = msg;
        while((i - 5) < msglen && (i - 5) < 32){
            buf[i++] = *str++;
        }
        buf[i++] = '\0';
    }

    uint32_t crc32 = crc32_calc(&buf[3], i - 3);
    crc32 = htobe32(crc32);

    buf[i++] = (crc32 >> 24 & 0xff);
    buf[i++] = (crc32 >> 16 & 0xff);
    buf[i++] = (crc32 >> 8 & 0xff);
    buf[i++] = (crc32 & 0xff);

    // final length
    buf[2] = i - 3;

    usart_write(usart, buf, buf[2] + 3);
}

/**
 * @brief DFU Serial Command: start flash procedure
 * 
 * @param data 
 * @param len 
 * @return int 
 */
static int dfu_serial_start_flash(unsigned char* data, int len) {
    if (len != 10)
        return SERIAL_NAK;   // NAK

    // update global flash parameters
    aes_init();
    // dfu_serial_.bStatus = USB_DFU_STATUS_OK;

    uint8_t dataType = data[0];
    switch(dataType) {
    case 0: // APP Flash
        dfu_serial_.dptr = (void*)_APP_START;
        dfu_serial_.remained = _APP_LENGTH;
        dfu_serial_.flash = program_flash;
        break;
#if defined(_EEPROM_ENABLED)
    case 1: // EEPROM
        dfu_serial_.dptr = (void*)_EE_START;
        dfu_serial_.remained = _EE_LENGTH;
        dfu_serial_.flash = program_eeprom;
        break;
#endif
    default:
        return SERIAL_NAK;
        break;
    }

    // dfu_serial_flash_block_size = data[1];

    // flash info
    dfu_serial_.flash_data_size  = (uint32_t)data[2] << 24 | (uint32_t)data[3] << 16 | (uint32_t)data[4] << 8 | (uint32_t)data[5];
    dfu_serial_.flash_data_crc   = (uint32_t)data[6] << 24 | (uint32_t)data[7] << 16 | (uint32_t)data[8] << 8 | (uint32_t)data[9];

    dfu_serial_.bStatus = 1;    // allows to flash

    return SERIAL_ACK;
}

static int dfu_serial_write_flash(unsigned char* buf, int blksize) {
    if (dfu_serial_.bStatus != 1)
        return SERIAL_NAK;

    if (blksize == 0) {
        // noop write
        return SERIAL_ACK;
    }

    if (blksize > dfu_serial_.remained) {
        // wrong size(for the last piece?)
        dfu_serial_.bStatus = 0;
        return SERIAL_NAK;
    }

    aes_decrypt(buf, buf, blksize);

    uint8_t ret;
    ret = dfu_serial_.flash(dfu_serial_.dptr, buf, blksize);
    if (ret == 0 /* flash write success */) {
        dfu_serial_.dptr += blksize;
        dfu_serial_.remained -= blksize;
        return SERIAL_ACK;
    }

    return SERIAL_NAK;
}

static int dfu_serial_verify_flash(uint8_t dataType) {
    uint8_t* dptr = 0;
    int   dlen = dfu_serial_.flash_data_size;

    switch(dataType) {
    case 0: // APP Flash
        dptr = (uint8_t*)_APP_START;
        break;
#if defined(_EEPROM_ENABLED)
    case 1: // EEPROM
        dptr = (uint8_t*)_EE_START;
        break;
#endif
    default:
        return SERIAL_NAK;
        break;
    }

    uint32_t cs;
    crc32_init(&cs);
    for(int i = 0; i < dlen; i++) {
        crc32_update(&cs, dptr[i]);
    }

    //TODO -  respond with 128bit hash or crc32 of the flashed data
    return (dfu_serial_.flash_data_crc == htobe32(cs)) ? SERIAL_ACK : SERIAL_NAK;
    // return SERIAL_ACK;
}

#if defined(STM32F1) || defined(STM32F4)
#define fnv1a32_turn(x, y) sub0001(x,y)
static uint32_t sub0001 (uint32_t fnv, uint32_t data ) {
    for (int i = 0; i < 4 ; i++) {
        fnv ^= (data & 0xFF);
        fnv *= 16777619;
        data >>= 8;
    }
    return fnv;
}
static uint16_t get_serialno_desc(char *buf) {
    char *str = buf;
    uint32_t fnv = 2166136261;
    fnv = fnv1a32_turn(fnv, *(uint32_t*)(UID_BASE + 0x00));
    fnv = fnv1a32_turn(fnv, *(uint32_t*)(UID_BASE + 0x04));
    fnv = fnv1a32_turn(fnv, *(uint32_t*)(UID_BASE + 0x08));
    for (int i = 28; i >= 0; i -= 4 ) {
        char c = (fnv >> i) & 0x0F;
        c += (c < 10) ? '0' : ('A' - 10);
        *str++ = c;
    }
    return 8;
}
#else
static uint16_t get_serialno_desc(char *buf) {
    char *str = buf;
    for (int i = 0; i < 8; i++) {
        *str++ = '0'
    }
    return 8;
}
#endif

/**
 * @brief DFU Serial Command: get device info
 * 
 * @param usart 
 * @return int 
 */
static int dfu_serial_get_info(struct usart_device *usart) {
    // FE | 00 0E | 80 | 00 | 31 33 33 30 31 44 41 30 | A2 C0 A4 83 

    uint8_t buf[32];
    buf[0] = 0xFE;
    buf[1] = 0x00;
    buf[2] = 14;                    // payload len: 2 + 8 + 4
    buf[3] = 0x80;                  // type
    buf[4] = 0x00;                  // ack

    int ret;
    ret = get_serialno_desc((char*)&buf[5]);    // read 8 chars of SN
    if (ret != 8)
        return SERIAL_NAK;

    uint8_t *data = &buf[3];
    uint32_t crc32 = crc32_calc(data, 10);
    crc32 = htobe32(crc32);                     // convert to big-endian 

    buf[13] = (crc32 >> 24) & 0xff;
    buf[14] = (crc32 >> 16) & 0xff;
    buf[15] = (crc32 >> 8)  & 0xff;
    buf[16] = (crc32 & 0xff);                   // total 17 bytes

    usart_write(usart, buf, 17);

    return SERIAL_IGN;
}

/**
 * @brief DFU Serial Command: do reboot
 * 
 * @return int 
 */
static int dfu_serial_reboot() {
    System_Reset();
    return SERIAL_IGN;
}

/**
 * @brief dfu serial data handler
 * 
 * @param usart 
 * @param data 
 * @param len 
 * @param offset 
 * @param total 
 */
static void dfu_serial_handle_data(struct usart_device *usart, unsigned char* data, int len, int offset, int total) {
    if (len < 5){
        #ifdef DEBUG
        dfu_serial_ack(usart, 0, SERIAL_ERR, NULL);
        #endif
        return;
    }

    // check crc
    uint32_t crc32 = crc32_calc(data, len);
    if (crc32 != 0){
        #ifdef DEBUG
        // crc32 = crc32_calc(data, len - 4);
        // usart_printf(usart, "data crc error, 0x%X" EOL, CPUTOBE32(crc32));      // at least 5 bytes: type(1) + crc(4)
        dfu_serial_ack(usart, data[0], SERIAL_NAK, "crc error");               // respond NAK
        #endif
        return ;
    }

    int ret = SERIAL_NAK;

    switch(data[0]) {
        case 0x00:          // initial flash
        if (offset == 0)
            ret = dfu_serial_start_flash(data + 1, len - 5);
        break;

        case 0x01:          // write flash
        if (offset == 0)
            ret = dfu_serial_write_flash(data + 1, len - 5);
        break;

        case 0x02:          // verify flash
        if (offset == 0)
            ret = dfu_serial_verify_flash(data[1]);
        break;

        case 0x03:          // read flash
        break;

        case 0x80:          // get info FE 00 05 80 52 93 45 C0
        if (offset == 0)
            ret = dfu_serial_get_info(usart);
        break;

        case 0x81:          // reboot/detach FE 00 05 81 C4 A3 42 B7
        if (offset == 0)
            ret = dfu_serial_reboot();
        break;

        default:
        break;
    }

    if (ret != SERIAL_IGN)
        dfu_serial_ack(usart, data[0], ret, NULL);

}

#define my_usart_rx_callback sub0000
static void sub0000(struct usart_device *usart, int len) {
    unsigned char c;
    int ret     = 0;

    dfu_serial_.resetTimer = 0;

    while(len > 0) {
        switch(dfu_serial_.rxState) {
            case DFU_SERIAL_RX_STATE_HEAD:                             // read 1 byte head 0xFE
            c = 0;
            ret = usart_read(usart, &c, 1);
            if (c == 0xFE){
                dfu_serial_.rxBufOffset = 0;
                dfu_serial_.rxDataSize  = 0;
                dfu_serial_.rxState     = DFU_SERIAL_RX_STATE_LEN1;
            }
            break;

            case DFU_SERIAL_RX_STATE_LEN1:                             // read 1st byte of length
            c = 0;
            ret = usart_read(usart, &c, 1);
            if (ret > 0) {
                dfu_serial_.rxDataSize = (((uint16_t)c) << 8);       // BIG-ENDIAN length
                dfu_serial_.rxState = DFU_SERIAL_RX_STATE_LEN2;
            }
            break;

            case DFU_SERIAL_RX_STATE_LEN2:                             // read 2nd byte of length
            c = 0;
            ret = usart_read(usart, &c, 1);
            if (ret > 0) {
                dfu_serial_.rxDataSize |= c;
                if (dfu_serial_.rxDataSize < 5) {                 // at least 5 bytes: type(1) + crc(4)
                    #ifdef DEBUG
                    dfu_serial_ack(usart, 0, SERIAL_ERR, NULL);
                    #endif
                    dfu_serial_.rxState = DFU_SERIAL_RX_STATE_HEAD;
                } else {
                    dfu_serial_.rxState = DFU_SERIAL_RX_STATE_BODY;
                }
            }
            break;

            case DFU_SERIAL_RX_STATE_BODY:                             // read data blocks
            {
                // NOTE - currently, it only supports single block transfer
                unsigned char* buf = dfu_serial_.rxBuf + dfu_serial_.rxBufOffset;
                int readlen = dfu_serial_.rxDataSize - dfu_serial_.rxBufOffset;
                if(readlen > (dfu_serial_.rxBufSize - dfu_serial_.rxBufOffset))       //<-- bug !!!
                    readlen = (dfu_serial_.rxBufSize - dfu_serial_.rxBufOffset);
                
                // for the last block, we should always read it out.
                ret = usart_read(usart, buf, readlen);      // read will return 0 if buffered data len < readlen
                // if (is_last_block) {
                //     ret = usart_read_force(usart, buf, readlen);    // force read the last data
                // }

                if (ret > 0) {
                    // 1 block read
                    dfu_serial_handle_data(usart, buf, ret, dfu_serial_.rxBufOffset, dfu_serial_.rxDataSize);

                    dfu_serial_.rxBufOffset += ret;
                    if (dfu_serial_.rxBufOffset == dfu_serial_.rxDataSize) {
                        // all data payload read OK, handle it and restore to initial state
                        dfu_serial_.rxBufOffset = 0;
                        dfu_serial_.rxState = DFU_SERIAL_RX_STATE_HEAD;
                    }
                }
            }
            break;

            default:
                return;         // should never be here
            break;
        }

        if (ret == 0)           // bail-out if usart_read() returns 0
            break;

        len -= ret;
    }
}

/**
 * @brief dfu serial state reset timer
 * 
 * @param usart 
 */
static void dfu_serial_poll() {
    // run the reset timer
    dfu_serial_.resetTimer++;
    if (dfu_serial_.resetTimer > SERIAL_RESET_TIMEOUT) {
        dfu_serial_.resetTimer = 0;
        if (dfu_serial_.rxState > DFU_SERIAL_RX_STATE_HEAD) {
            dfu_serial_.rxState = DFU_SERIAL_RX_STATE_HEAD;
            #ifdef DEBUG
            usart_print(&usart, "dfu rx state reset" EOL);
            #endif
        }
    }
}

#endif

int main (void) {

#if defined(ENABLE_USART)
    usart_init(&usart, 115200, my_usart_rx_callback, NULL);
    usart_printf(&usart, DFU_STR_PRODUCT EOL);

    dfu_serial_init((uint8_t*)dfu_buffer, sizeof(dfu_buffer));
#endif

#if defined(ENABLE_USB)
    dfu_init();
#endif

    while(1) {
#if defined(ENABLE_USART)
        usart_poll(&usart);
        dfu_serial_poll(/*&dfu_serial_*/);
#endif
#if defined(ENABLE_USB)
        usbd_poll(&dfu);
#endif
    }
}

#if defined(_EEPROM_ENABLED) && (defined(STM32F1) || defined(STM32F4))
__attribute__((long_call, section(".ram_func"))) uint8_t program_eeprom(void *romaddr, const void *buffer, size_t blksize) {
    return 0;
}
#endif
