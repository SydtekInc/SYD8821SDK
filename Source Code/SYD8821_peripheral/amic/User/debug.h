#ifndef _DEBUG_H_
#define _DEBUG_H_

#include "ARMCM0.h"
#include "config.h"
#include "uart.h"

#ifdef CONFIG_UART_ENABLE
	#define DBGPRINTF(s)	 							dbg_printf s
	#define DBGHEXDUMP(title,buf,sz)	 				dbg_hexdump(title,buf,sz)
    #define DBG_UART_ID                                 0
    #define DBGTO(id, FMT, ARGS...)                     dbg_printf_sel(id, FMT, ##ARGS)
    #define DBGFROM(id, ptr)                            uart_read(id, ptr)
#elif defined(_SYD_RTT_DEBUG_)
	
#else
	#define DBGPRINTF(s)	 			
	#define DBGHEXDUMP(title,buf,sz)
    #define DBGTO(id, FMT, ARGS...)
    #define DBGTOFLUSH(id)
    #define DBGFROM(id, ptr)                            0
#endif
#define DBG(FMT, ARGS...)                           DBGTO(DBG_UART_ID, FMT, ##ARGS)
#define debug_printf_1(FMT, ARGS...)                DBGTO(DBG_UART_ID, FMT, ##ARGS)
#define debug_printf(FMT, ARGS...)                  DBGTO(DBG_UART_ID, FMT, ##ARGS)

#ifdef CONFIG_UART_ENABLE

#ifdef _DEBUG_C
#else
	extern void dbg_init(void);
	extern void dbg_printf(char *format,...) ;
	extern void dbg_hexdump(char *title, uint8_t *buf, uint16_t sz);
    extern void dbg_printf_sel(uint8_t id, char *format, ...);
#endif

#endif

#endif
