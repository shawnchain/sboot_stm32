#include "config.h"

#if defined(ENABLE_TRACE) && (defined(STM32F1) || defined(STM32F4))

#include "swo.h"
#include "stm32.h"

#include <stdarg.h>
#include <stdio.h>

#ifndef ENABLE_SWO_INIT
#define ENABLE_SWO_INIT 0
#endif

void trace_init(void) {

#if ENABLE_SWO_INIT
    // CMSIS_CoreDebug Core Debug Registers (CoreDebug)
    CoreDebug->DEMCR = CoreDebug_DEMCR_TRCENA_Msk;

    // CMSIS_ITM Instrumentation Trace Macrocell (ITM)
    // ITM Lock Access Register, C5ACCE55 enables more write access to Control Register 0xE00 :: 0xFFC
    ITM->LAR = 0xC5ACCE55; 
    ITM->TCR = ITM_TCR_TraceBusID_Msk | ITM_TCR_SWOENA_Msk | ITM_TCR_SYNCENA_Msk | ITM_TCR_ITMENA_Msk;
    ITM->TPR = ITM_TPR_PRIVMASK_Msk;
    ITM->TER = 0xFUL;

    #if 1
    // CMSIS_TPI Trace Port Interface (TPI)
    uint32_t SWOSpeed = 115074; /* default 115200 baud rate */
    uint32_t SWOPrescaler = (SystemCoreClock / SWOSpeed) - 1; /* SWOSpeed in Hz, note that SystemCoreClock is expected to be match the CPU core clock */
    TPI->ACPR = SWOPrescaler; /* "Async Clock Prescaler Register". Scale the baud rate of the asynchronous output */
    // Selected PIN Protocol Register": Select which protocol to use for trace output (2: SWO NRZ, 1: SWO Manchester encoding)
    TPI->SPPR = 2UL; 
    TPI->FFCR = 0x00000100; /* Formatter and Flush Control Register */

    // CMSIS_DWT Data Watchpoint and Trace (DWT)
    DWT->CTRL = 0x400003FE; /* DWT_CTRL */
    #endif

#endif

}

const int PORT0 = 0;
static void swo_putc(int port, uint32_t ch) {
    if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
       ((ITM->TER & port               ) != 0UL)   )     /* ITM Port n enabled */
    {
        while (ITM->PORT[port].u32 == 0)
            ;

        ITM->PORT[port].u8 = (uint8_t) ch;
    }
}

int swo_print(const char* s) {
    int i;
    for(i = 0; s[i] !=0; i++) {
        swo_putc(PORT0, s[i]);
    }
    return i;
}

#if defined(ENABLE_TRACE_PRINTF)
static char swo_msg_buf[128];
int swo_printf(const char* fmt, ...) {
    va_list ap;
	va_start(ap, fmt);
	int len = vsnprintf(swo_msg_buf, sizeof(swo_msg_buf), fmt, ap);
	va_end(ap);
    swo_print(swo_msg_buf);

    return len;
}
#endif


#endif

