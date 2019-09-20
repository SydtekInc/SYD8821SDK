/**************************************************************************//**
 * @file     ARMCM0.h
 * @brief    CMSIS Core Peripheral Access Layer Header File for
 *           ARMCM0 Device Series
 * @version  V2.00
 * @date     17. Februar 2014
 *
 * @note
 *
 ******************************************************************************/
/* Copyright (c) 2011 - 2014 ARM LIMITED

   All rights reserved.
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
   - Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   - Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   - Neither the name of ARM nor the names of its contributors may be used
     to endorse or promote products derived from this software without
     specific prior written permission.
   *
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
   ---------------------------------------------------------------------------*/


#ifndef ARMCM0_H
#define ARMCM0_H

#ifdef __cplusplus
extern "C" {
#endif


/* -------------------------  Interrupt Number Definition  ------------------------ */
typedef enum IRQn
{
/* -------------------  Cortex-M0 Processor Exceptions Numbers  ------------------- */
	NonMaskableInt_IRQn	= -14,      /*!<  2 Non Maskable Interrupt          */
	HardFault_IRQn                = -13,      /*!<  3 HardFault Interrupt             */
	SVCall_IRQn                  	=  -5,      /*!< 11 SV Call Interrupt               */
	PendSV_IRQn                   	=  -2,      /*!< 14 Pend SV Interrupt               */
	SysTick_IRQn                  	=  -1,      /*!< 15 System Tick Interrupt           */
/* ----------------------  ARMCM0 Specific Interrupt Numbers  --------------------- */
	GPIO_IRQn                    	=   0,      /*!< GPIO Interrupt                    */
	LLC_IRQn                    	=   1,      /*!< LLC Interrupt                    */
	SPIM_IRQn				=   2,		
	SPIS_IRQn				=   3,
	I2CM0_IRQn				=   4,
	I2CM1_IRQn				=   5,
	UART0_IRQn                    	=   6,      /*!< UART0 Interrupt                    */
	UART2_IRQn                    	=   7,      /*!< UART1 Interrupt                    */
	I2CS_IRQn				=   8,
	TIMER0_IRQn                    	=   9,      /*!< TIMER0 Interrupt                    */
	TIMER1_IRQn                    	=   10, 	 /*!< TIMER1 Interrupt                    */
	TIMER2_IRQn                    	=   11,      /*!< TIMER2 Interrupt                    */
	TIMER3_IRQn                    	=   12,      /*!< TIMER3 Interrupt                    */
	TIMER4_IRQn                    	=   13,      /*!< TIMER4 Interrupt                    */
	WDT_IRQn				=   14,
	SW_IRQn				=   15,
	PWM1_IRQn				=   16,
	PWM2_IRQn				=   17,
	PWM3_IRQn				=   18,
	M2M_IRQn				=   19,
	RTC_IRQn				=   20,
	ISO7816_IRQn			=   21,
	IR_IRQn					=   22,
	TRNG_IRQn				=   23,
	GPADC_IRQn				=   24,
	UART1_IRQn				=   25,
	CAPDET_IRQn			=   26,
	PDM_IRQn				=   27,
	LLC2_IRQn				=   28,
	LLC3_IRQn				=   29,
	I2S_IRQn				=   30,
	INTCOMB_IRQn			=   31

} IRQn_Type;


/* ================================================================================ */
/* ================      Processor and Core Peripheral Section     ================ */
/* ================================================================================ */

/* --------  Configuration of the Cortex-M4 Processor and Core Peripherals  ------- */
#define __CM0_REV                 0x0000    /*!< Core revision r0p0                              */
#define __MPU_PRESENT             0         /*!< MPU present or not                              */
#define __NVIC_PRIO_BITS          2         /*!< Number of Bits used for Priority Levels         */
#define __Vendor_SysTickConfig    0         /*!< Set to 1 if different SysTick Config is used    */

#include <core_cm0.h>                       /* Processor and core peripherals                    */
#include "system_ARMCM0.h"                  /* System Header                                     */

/* -------------------  Start of section using anonymous unions  ------------------ */
#if defined(__CC_ARM)
  #pragma push
  #pragma anon_unions
#elif defined(__ICCARM__)
  #pragma language=extended
#elif defined(__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined(__TMS470__)
/* anonymous unions are enabled by default */
#elif defined(__TASKING__)
  #pragma warning 586
#elif defined ( __CSMC__ )		/* Cosmic */
/* anonymous unions are enabled by default */
#else
  #warning Not supported compiler type
#endif

//add types define
#ifndef bool
	typedef unsigned char			bool;
#endif

#ifndef true
	#define true		1
#endif

#ifndef false
	#define false		0
#endif
//add types define end

/* -------------------------------------------------------------------------------- */
/* ----------------------       SYD8821 Register Control       ---------------------- */
/* -------------------------------------------------------------------------------- */
#pragma pack(1)

#define U32BIT(s)			((uint32_t)1<<(s))
#define BIT(s)				((uint8_t)1<<(s))

#define BIT0					0x01
#define BIT1					0x02
#define BIT2					0x04
#define BIT3					0x08
#define BIT4					0x10
#define BIT5					0x20
#define BIT6					0x40
#define BIT7					0x80
#define BIT8					0x100
#define BIT9					0x200
#define BIT10					0x400
#define BIT11					0x800
#define BIT12					0x1000
#define BIT13					0x2000
#define BIT14					0x4000
#define BIT15					0x8000
#define BIT16					0x10000
#define BIT17					0x20000
#define BIT18 				    0x40000
#define BIT19					0x80000
#define BIT20					0x100000
#define BIT21					0x200000
#define BIT22					0x400000
#define BIT23					0x800000
#define BIT24					0x1000000
#define BIT25					0x2000000
#define BIT26					0x4000000
#define BIT27					0x8000000
#define BIT28					0x10000000
#define BIT29					0x20000000
#define BIT30					0x40000000
#define BIT31					0x80000000	
	
enum GPIO_DEFINE {
	GPIO_0		= 0x00,
	GPIO_1,
	GPIO_2,
	GPIO_3,
	GPIO_4,
	GPIO_5,
	GPIO_6,
	GPIO_7,
	GPIO_8,
	GPIO_9,
	GPIO_10,
	GPIO_11,
	GPIO_12,
	GPIO_13,
	GPIO_14,
	GPIO_15,
	GPIO_16,
	GPIO_17,
	GPIO_18,
	GPIO_19,
	GPIO_20,
	GPIO_21,
	GPIO_22,
	GPIO_23,
	GPIO_24,
	GPIO_25,
	GPIO_26,
	GPIO_27,
	GPIO_28,
	GPIO_29,
	GPIO_30,
	GPIO_31,
	GPIO_32,
	GPIO_33,
	GPIO_34,
};
/* ============================ SYSCONFIG =========================== */
typedef struct {
	/* 0x00 ~ 0x03*/	
	__IO  uint32_t  MEM_CONFIG:4;	
	__IO  uint32_t  FCLK_CONFIG:2;		
	__IO  uint32_t  PCLK_CONFIG:2;		
	__IO  uint32_t  BLE_MASTER_MODE:1;	
	__IO  uint32_t  WDT_RST_EN:1;	
    __IO  uint32_t  CLK_SPEED_SEL:1;
    __IO  uint32_t  WDT_RST_ALL_EN:1;
    __IO  uint32_t  CLK_EFUSE_EN:1;
    __IO  uint32_t  UART_CLK_SEL:3;
    __IO  uint32_t  RSVD00:16;
    /* 0x04 ~ 0x07*/
    __IO  uint32_t  CLK_ARB_EN:1;
    __IO  uint32_t  CLK_SEC_EN:1;
    __IO  uint32_t  CLK_TRNG_EN:1;
    __IO  uint32_t  CLK_GPADC_EN:1;
    __IO  uint32_t  CLK_CAPDET_EN:1;
    __IO  uint32_t  CLK_M2M_EN:1;
    __IO  uint32_t  CLK_I2S_EN:1;
    __IO  uint32_t  CLK_UART01_EN:1;
    __IO  uint32_t  CLK_PDM_EN:1;
    __IO  uint32_t  CLK_PWM1_EN:1;
    __IO  uint32_t  CLK_UART2_EN:1;
    __IO  uint32_t  CLK_IR_EN:1;
    __IO  uint32_t  CLK_MS_EN:1;
    __IO  uint32_t  CLK_KB_EN:1;
    __IO  uint32_t  CLK_SPIM_EN:1;
    __IO  uint32_t  CLK_SPIS_EN:1;
    __IO  uint32_t  CLK_I2CM0_EN:1;
    __IO  uint32_t  CLK_I2CM1_EN:1;
    __IO  uint32_t  CLK_I2CS_EN:1;
    __IO  uint32_t  CLK_SC_EN:1;
	__IO  uint32_t  CLK_SRAM_EN:1;
	__IO  uint32_t  CLK_QSPI_EN:1;
    __IO  uint32_t  CLK_CACHE_EN:1;
	__IO  uint32_t  CLK_BLE_EN:1;
	__IO  uint32_t  CLK_RTC_EN:1;
    __IO  uint32_t  CLK_TIMER0_EN:1;
    __IO  uint32_t  CLK_TIMER1_EN:1;
    __IO  uint32_t  CLK_TIMER2_EN:1;
    __IO  uint32_t  CLK_TIMER3_EN:1;
    __IO  uint32_t  CLK_TIMER4_EN:1;
    __IO  uint32_t  CLK_PWM_LED_EN:1;
	__IO  uint32_t  CLK_WDT_EN:1;
	/* 0x08 ~ 0x0B*/		
    __IO  uint32_t  RSVD01;
	/* 0x0C ~ 0x0F*/	
	__IO  uint32_t  FLASH_ADDR_OFFSET;		
}SYS_CONFIG_TYPE;


/* ============================ Cache control =========================== */

typedef struct {
	/* 0x00 ~ 0x03*/	
	__IO  uint32_t  LINE_SIZE:2;	
	__IO  uint32_t  RSVD00:30;		
	/* 0x04 ~ 0x07*/
	__IO  uint32_t  RANDOM_INIT:1;	
	__IO  uint32_t  RSVD04:31;	
	/* 0x08 ~ 0x0B*/
	__IO  uint32_t  BYPASS:1;	
	__IO  uint32_t  RSVD08:31;		
	/* 0x0C ~ 0x0F*/
	__IO  uint32_t  FLUSH:1;	
	__IO  uint32_t  RSVD0C:31;			
	/* 0x10 ~ 0x13*/
	__IO  uint32_t  CACHE_READY:1;	
	__IO  uint32_t  RSVD10:31;				
	/* 0x14 ~ 0x17*/	
	__IO  uint32_t  CACHE_HIT;	
	/* 0x18 ~ 0x1B*/	
	__IO  uint32_t  CACHE_MISS;		
	/* 0x1C ~ 0x1F*/	
	__IO  uint32_t  COUNTER_HIT_MISS_ENABLE:1;			
	__IO  uint32_t  RSVD1C:31;					
}CACHE_CTRL_TYPE;



typedef struct {
	/* 0x00 */
	__IO  uint8_t  SR_A:1;
	__IO  uint8_t  SR_P:1;
	__IO  uint8_t  RSVD00:6;
	/* 0x01 */
	/*__IO  uint8_t  TXINT:1;
	__IO  uint8_t  RXINT:1;
	__IO  uint8_t  ADVSCANEND:1;
	__IO  uint8_t  CONNSETUP:1;
	__IO  uint8_t  CONNLOST:1;
	__IO  uint8_t  CONNEVTEND:1;
	__IO  uint8_t  SLEEP:1;
	__IO  uint8_t  WAKEUP:1;*/
	__IO  uint8_t  INTST;
	/* 0x02 */
	union{
		struct{
			__IO  uint8_t  INTMSK;	
		};
		struct{
			__IO  uint8_t  TXINTMSK:1;
			__IO  uint8_t  RXINTMSK:1;
			__IO  uint8_t  ADVSCANENDMSK:1;
			__IO  uint8_t  CONNSETUPMSK:1;
			__IO  uint8_t  CONNLOSTMSK:1;
			__IO  uint8_t  CONNEVTENDMSK:1;
			__IO  uint8_t  SLEEPMSK:1;
			__IO  uint8_t  WAKEUPMSK:1; 

		};
	};
	/* 0x03 */
	__IO  uint8_t  TX_FULL:1;
	__IO  uint8_t  TX_EMPTY:1;
	__IO  uint8_t  RX_FULL:1;
	__IO  uint8_t  RX_EMPTY:1;
	__IO  uint8_t  RSVD03:4;	
	/* 0x04 */
	__IO  uint8_t  COE_ENABLE:1;
	__IO  uint8_t  COE_RXFF_R:1;
	__IO  uint8_t  RSVD04:6;
	/* 0x05 */
	__IO  uint8_t  EN_PRIVADR:1;
	__IO  uint8_t  RSVD05:7;	
	/* 0x06 */
	__IO  uint8_t  COE_STOP:1;
	__IO  uint8_t  LTCY_STOP:1;
	__IO  uint8_t  RSVD06:6;
	/* 0x07 */
	__IO  uint8_t  RATE_SEL:1;
	__IO  uint8_t  RSVD07:7;	
	/* 0x08 */
	__IO  uint8_t  ARD:4;
	__IO  uint8_t  RSVD08:4;
/*	
	__IO  uint8_t  DIS_CRC:1;
	__IO  uint8_t  WW_HALVED_EN:1;
	__IO  uint8_t  WW_RESERVE:1;
	__IO  uint8_t  RSVD08:1;
*/	
	/* 0x09 */
	__IO  uint8_t  WHITE_EN:1;
	__IO  uint8_t  CRC_ALLOW:1;
	__IO  uint8_t  RSVD090:2;
	__IO  uint8_t  CRC_PASS:1;
	__IO  uint8_t  SENDING_ACK:1;
	__IO  uint8_t  RSVD091:2;
	/* 0x0A */
	__IO  uint8_t  RSVD0A;
	/* 0x0B */
	__IO  uint8_t  T_TXSTR;
	/* 0x0C */
	__IO  uint8_t  T_R2TSTR;
	/* 0x0D */
	__IO  uint8_t  FIXED_CH;
	/* 0x0E */
	__IO  uint8_t  T_STBYLEN;
	/* 0x0F */
	__IO  uint8_t  FSM_STATE:6;
	__IO  uint8_t  RSVD0F:2;
	/* 0x10 */
	__IO  uint8_t  ADV_EN:1;
	__IO  uint8_t  ADV_DLY_EN:1;
	__IO  uint8_t  FILTERPOLICY:2;
	__IO  uint8_t  FIL_ADR_TYPE:1;
	__IO  uint8_t  FIL_ADVA_EN:1;
	__IO  uint8_t  COE_ADV_EN:1;	
	__IO  uint8_t  RSVD10:1;
	/* 0x11 */
	__IO  uint8_t  ADV_RND:7;
    __IO  uint8_t  RSVD11:1;
	/* 0x12 ~ 0x13 */
	__IO  uint16_t  ADV_ITV;
	/* 0x14 */
	__IO  uint8_t  ADV_PITV;
	/* 0x15 */
	__IO  uint8_t  ADV_CHIDX;
	/* 0x16 */
	__IO  uint8_t  FP_A_IND:1;	
	__IO  uint8_t  FP_A_DIR:1;		
	__IO  uint8_t  FP_A_NC:1;			
	__IO  uint8_t  FP_SCAN_REQ:1;			
	__IO  uint8_t  FP_SCAN_RSP:1;			
	__IO  uint8_t  FP_CONN_REQ:1;			
	__IO  uint8_t  FP_A_SCAN:1;			
	__IO  uint8_t  RSVD16:1;				
	/* 0x17 */
	__IO  uint8_t  RSVD17;
	/* 0x18 */
	__IO  uint8_t  A_SLPINT:1;	
	__IO  uint8_t  S_SLPINT:1;	
	__IO  uint8_t  A_WAKEINT:1;	
	__IO  uint8_t  S_WAKEINT:1;	
	__IO  uint8_t  COE_STOPINT:1;	
	__IO  uint8_t  RSVD18:3;
	/* 0x19 */
	__IO  uint8_t  A_SLPMSK:1;	
	__IO  uint8_t  S_SLPMSK:1;	
	__IO  uint8_t  A_WAKEMSK:1;	
	__IO  uint8_t  S_WAKEMSK:1;	
	__IO  uint8_t  COE_STOPMSK:1;	
	__IO  uint8_t  RSVD19:3;
	/* 0x1A */
	__IO  uint8_t  CONN_AUTOSLP_EN:1;	
	__IO  uint8_t  ADV_AUTOSLP_EN:1;	
	__IO  uint8_t  AUTOSLP_CLKG_EN:1;	
	__IO  uint8_t  LTCY_WW_EN:1;	
	__IO  uint8_t  RSVD1A:4;
	/* 0x1B */
	__I   uint8_t  RSVD1B;
	/* 0x1C */
	__IO  uint8_t  T_TXSTR_2M;
	/* 0x1D */
	__IO  uint8_t  T_R2TSTR_2M;
	/* 0x1E */
	__IO  uint8_t  T_STBYLEN_2M;
	/* 0x1F */
	__I   uint8_t  RSVD1F;
	/* 0x20 ~ 0x21 */
	__IO  uint16_t  CONN_EVTCNT;
	/* 0x22 */
	__IO  uint8_t  CONN_UPDATE:1;
	__IO  uint8_t  CHMAP_UPDATE:1;
	__IO  uint8_t  RSVD22:6;
	/* 0x23 */
	__IO  uint8_t  CONN_CRCERRTH:3;
	__IO  uint8_t  RSVD230:1;
	__IO  uint8_t  STOP_REPLY:1;
	__IO  uint8_t  RSVD231:3;
	/* 0x24 */
	__IO  uint8_t  RSVD24;
	/* 0x25 */
	__IO  uint8_t  HOP_INC;
	/* 0x26 ~ 0x27 */
	__IO  uint16_t  CONN_GO;
	/* 0x28 */
	__IO  uint8_t  CONN_WS;	
	/* 0x29 */
	__IO  uint8_t  RSVD29;	
	/* 0x2A ~ 0x2B */
	__IO  uint16_t  CONN_ITV;
	/* 0x2C ~ 0x2D */
	__IO  uint16_t  CONN_LTCY;
	/* 0x2E ~ 0x2F */
	__IO  uint16_t  CONN_STO;
	/* 0x30 ~ 0x31 */
	__IO  uint16_t  CONN_INS;
	/* 0x32 ~ 0x33 */
	__IO  uint16_t  RSVD32;
	/* 0x34 */
	__IO  uint8_t  RSVD34;	
	/* 0x35 */
	__IO  uint8_t  LOST_SUPER:1;	
	__IO  uint8_t  LOST_WWEXT:1;	
	__IO  uint8_t  LOST_6NC:1;	
	__IO  uint8_t  LOST_INS:1;	
	__IO  uint8_t  RSVD35:4;	
	/* 0x36 ~ 0x37 */
	__IO  uint16_t  CONN_WW_SETUP;	
	///* 0x37 */
	//__IO  uint8_t  RSVD37;		
	/* 0x38 ~ 0x39 */
	__IO  uint16_t  CONN_INS_CH;
	/* 0x3A ~ 0x3B */
	__IO  uint16_t  CONN_WW;	
	/* 0x3C ~ 0x3F */
	__IO  uint8_t  RSVD3C[4];
	/* 0x40 */
	__IO  uint8_t  DTM_EN:1;	
	__IO  uint8_t  DTM_MODE:1;	
	__IO  uint8_t  BER_TEST:1;	
	__IO  uint8_t  RSVD40:5;	
	/* 0x41 */
	__IO  uint8_t  BER_LEN;	
	/* 0x42 ~ 0x43 */
	__IO  uint8_t  RSVD42[2];
	/* 0x44 ~ 0x45 */
	__IO  uint16_t  DTM_RCNT;
	/* 0x46 ~ 0x47 */
	__IO  uint16_t  DTM_TCNT;
	/* 0x48 */
	__IO  uint8_t  RXPKT_LENGTH;	
	/* 0x49 ~ 0x4F */
	__IO  uint8_t  RSVD48[7];
	/* 0x50 */
	__IO  uint8_t  SEC_TX:1;	
	__IO  uint8_t  SEC_RX:1;	
	__IO  uint8_t  RSVD50:6;	
	/* 0x51 */
	__IO  uint8_t  SEC_MIC_OK:1;
	__IO  uint8_t  SEC_BUSY:1;
	__IO  uint8_t  RSVD51:6;
	/* 0x52 ~ 0x53 */
	__IO  uint8_t  RSVD52[2];
	/* 0x54 */
	__IO  uint8_t  SEC_AES_REQ:1;
	__IO  uint8_t  RSVD54:7;
	/* 0x55 */
	__IO  uint8_t  RAND_NUM;
	/* 0x56 ~ 0x5F */
	__IO  uint8_t  RSVD56[10];
	/* 0x60 */
	__IO  uint8_t  SCAN_EN:1;
	__IO  uint8_t  SCAN_TYPE:1;
	__IO  uint8_t  INIT_EN:1;
	__IO  uint8_t  COE_SCAN_EN:1;
	__IO  uint8_t  RSVD60:4;	
	/* 0x61 */
	__IO  uint8_t  SCAN_RND;
	/* 0x62 */
	__IO  uint8_t  CH_USER;
	/* 0x63 */
	__IO  uint8_t  RSVD63;	
	/* 0x64 ~ 0x65 */
	__IO  uint16_t  SCAN_WINDOW;
	/* 0x66 ~ 0x67 */
	__IO  uint16_t  SCAN_ITV;
	/* 0x68 ~ 0x6F */
	__IO  uint8_t  RSVD68[8];
	/* 0x70 */
	__IO  uint8_t  TH_LAST;
	/* 0x71 */
	__IO  uint8_t  TH_SLEEP;
	/* 0x72 */
	__IO  uint8_t  TH_COE;
	/* 0x73 */
	__IO  uint8_t  T_CLKWAKE;
	/* 0x74 */
	__IO  uint8_t  ADV_CH0;
	/* 0x75 */
	__IO  uint8_t  ADV_CH1;
	/* 0x76 */
	__IO  uint8_t  ADV_CH2;
	/* 0x77 */
	__IO  uint8_t  DIS_LOST_LINK:1;	
	__IO  uint8_t  EN_MORE_DATA:1;		
	__IO  uint8_t  RSVD77:2;		
	__IO  uint8_t  EN_FW_LTCY_CHG:1;		
	__IO  uint8_t  WS_OVER_HJALF_EN:1;	
	__IO  uint8_t  RSVD77_2:2;			
	/* 0x78 */
	__IO  uint8_t  WW_MINUS1_CNT:5;
	__IO  uint8_t  RSVD78:3;
	/* 0x79 */
	__IO  uint8_t  OVER_0P5:1;
	__IO  uint8_t  RSVD79:7;
	/* 0x7A */
	__IO  uint8_t  OVER_SLPTH:1;	
	__IO  uint8_t  OVER_COETH:1;		
	__IO  uint8_t  RSVD7A:6;		
	/* 0x7B ~ 0x7F */
	__IO  uint8_t  RSVD7B[5];
	/* 0x80 ~ 0x85*/
	__IO  uint8_t  MAC_ADDR[6];
	/* 0x86 ~ 0x87*/
	__IO  uint8_t  RSVD86[2];
	/* 0x88 ~ 0x8B*/
	__IO  uint8_t  ACC_CODE[4];
	/* 0x8C ~ 0x91*/
	__IO  uint8_t  WHITE_LIST[6];
	/* 0x92 ~ 0x93*/
	__IO  uint8_t  RSVD92[2];
	/* 0x94 ~ 0x96*/
	__IO  uint8_t  INIT_CRC[3];
	/* 0x97*/
	__IO  uint8_t  RSVD97;
	/* 0x98 ~ 0x9C*/
	__IO  uint8_t  CHMAP[5];
	/* 0x9D ~ 0x9F*/
	__IO  uint8_t  RSVD9D[3];	
	/* 0xA0 ~ 0xAF*/
	__IO  uint8_t  SEC_LTK[16];
	/* 0xB0 ~ 0xBF*/
	__IO  uint8_t  SEC_IV[16];
	/* 0xC0*/
	__IO  uint8_t  TXD0;
	/* 0xC1*/
	__IO  uint8_t  TXD1;
	/* 0xC2*/
	__IO  uint8_t  TXD0_COE;	
	/* 0xC3*/
	__IO  uint8_t  TXD1_COE;		
	/* 0xC4*/
	__IO  uint8_t  RXD;
	/* 0xC5~0xC7*/
	__IO  uint8_t  RSVDC4[3];
	/* 0xC8*/
	__IO  uint8_t  TXFF_DONE;
	/* 0xC9*/
	__IO  uint8_t  RXFF_DONE;
	/* 0xCA ~ 0xCE*/
	__IO  uint8_t  CHUPDATE[5];
	/* 0xCF*/
	__IO  uint8_t  RSVDCF;
	/* 0xD0 ~ 0xD3 */
	__IO  uint8_t  LLC_DBG[4];
	/* 0xD4 */
	__IO  uint8_t  RXD_COE;
	/* 0xD5 ~ 0xD7 */
	__IO  uint8_t  RSVDD5[3];
	/* 0xD8 */
	__IO  uint8_t  TXFF_DONE_COE;
	/* 0xD9 */
	__IO  uint8_t  RXFF_DONE_COE;	
	/* 0xDA ~ 0xE7 */
	__IO  uint8_t  RSVDDA[14];	
	/* 0xE8 ~ 0xEB*/
	__IO  uint32_t  SLDMA_MODE:1;	
	__IO  uint32_t  SLDMA_RX_REQ:1;	
	__IO  uint32_t  SLDMA_TX_REQ:1;		
	__IO  uint32_t  RSVDE8:29;
	/* 0xEC ~ 0xEF*/		
	__IO  uint32_t  SLDMA_RX_ADDR:18;	
	__IO  uint32_t  RSVDEE:6;			
	__IO  uint32_t  SLDMA_RX_LEN:7;			
	__IO  uint32_t  RSVDEF:1;			
	/* 0xF0 ~ 0xF3*/	
	__IO  uint32_t  SLDMA_TX_ADDR:18;	
	__IO  uint32_t  RSVDF2:6;			
	__IO  uint32_t  SLDMA_TX_LEN:7;			
	__IO  uint32_t  RSVDF3:1;				
} LL_CTRL_TYPE;

typedef struct {
	/* 0x00 */
	__IO  uint8_t  RAMP_UP_SPEED:2;
	__IO  uint8_t  TX_IRQ_SWAP:1;
	__IO  uint8_t  RSVD00:4;
	__IO  uint8_t  LOOP_TEST:1;
	/* 0x01 */
	__IO  uint8_t  TM_SEL:2;
	__IO  uint8_t  TN_EN:1;
	__IO  uint8_t  RSVD01:5;
	/* 0x02 */
	__IO  uint8_t  DAC_VAL_I;
	/* 0x03 */
	__IO  uint8_t  DAC_VAL_Q;
	/* 0x04 */
	__IO  uint8_t  FIXED_FREQ:6;
	__IO  uint8_t  RSVD04:2;
	/* 0x05 */
	__IO  uint8_t  TX_IF_FREQ;
	/* 0x06 */
	__IO  uint8_t  DEV_SEL:4;
	__IO  uint8_t  RSVD06:4;
	/* 0x07 */
	__IO  uint8_t  MUL_FACTOR;
	/* 0x08 */
	__IO  uint8_t  TX_PHY_MODE;
	/* 0x09 */
	__IO  uint8_t  ASYMM_LINK_OPT:1;
	__IO  uint8_t  STABLE_MOD_INDEX:1;	
	__IO  uint8_t  RSVD09:6;		
	/* 0x0A */
	__IO  uint8_t  RSVD0A:8;	
	/* 0x0B */
	__IO  uint8_t  RSVD0B:8;
	/* 0x0C */
	__IO  uint8_t  RSVD0C:8;
	/* 0x0D */
	__IO  uint8_t  RSVD0D:8;
	/* 0x0E */
	__IO  uint8_t  RSVD0E:8;
	/* 0x0F */
	__IO  uint8_t  RSVD0F:8;
	/* 0x10 */
	__IO  uint8_t  ADC_MODE:1;
	__IO  uint8_t  RSSI_ADC_M:1;
	__IO  uint8_t  RX_IQ_SWAP:1;
	__IO  uint8_t  IF_SEL:2;
	__IO  uint8_t  ADC_EDGE_SEL:1;
	__IO  uint8_t  BB_IQ_SWAP:1;
	__IO  uint8_t  FIX_BB_IQ_SWAP:1;
	/* 0x11 */
	__IO  uint8_t  WAIT1_CNT:4;
	__IO  uint8_t  WAIT2_CNT:4;
	/* 0x12 */
	__IO  uint8_t  MEASURE_CNT:4;
	__IO  uint8_t  POWER_CNT_TH:4;
	/* 0x13 */
	__IO  uint8_t  INA_SEL:3;
	__IO  uint8_t  FIX_LNA_SEL:1;
	__IO  uint8_t  BIT_SEL:1;
	__IO  uint8_t  FIX_BIT_SEL:1;
	__IO  uint8_t  BYPASS_ADC_Q:1;
	__IO  uint8_t  BYPASS_ADC_I:1;	
	/* 0x14 ~ 0x17*/
	__IO  uint32_t  FREQ_OFFSET_ROT:19;
	__IO  uint32_t	RSVD14:4;
	__IO  uint32_t  FIX_FREQ_OFFSET_ROT:1;	
	__IO  uint32_t  FREQ_OFFSET_SYNC:7;
	__IO  uint32_t  FIX_REQ_OFFSET_SYNC:1;
	/* 0x18 */
	__IO  uint8_t  PHDIN_TH:6;
	__IO  uint8_t  FIX_RSSI_IQ_SEL:1;
	__IO  uint8_t  FIX_RSSI_IQ:1;
	/* 0x19 */
	__IO  uint8_t  CORR_TH:7;
	__IO  uint8_t  RSVD19:1;
	/* 0x1A */
	__IO  uint8_t  SYNC_PASS:4;
	__IO  uint8_t  SYNC_PHASE:2;	
	__IO  uint8_t  RSVD1A:2;
	/* 0x1B */	
	__IO  uint8_t  CORR_TH2:2;	
	__IO  uint8_t  RSVD18:6;		
	/* 0x1C */	
	__IO  uint8_t  PKT_DET_TH:7;		
	__IO  uint8_t  RSVD1C:1;		
}MDM_CTRL_TYPE;

/* ============================ timer =========================== */
typedef struct {
	/* 0x00~ 0x03*/
	__IO  uint32_t	TIMER_ENABLE:1;
	__IO  uint32_t	RSVD00:31;	
	union{
		struct{
			/* 0x04~ 0x07*/
			__IO  uint32_t	TIMER_RELOAD_VAL:16;
			__IO  uint32_t	RSVD01:16;
			/* 0x08~ 0x0B*/	
			__IO  uint32_t	TIMER_COUNTER:16;
			__IO  uint32_t	RSVD02:16;
		};
		struct{
		/* 0x04~ 0x07*/
		__IO  uint32_t	TIMER4_RELOAD_VAL:24;
		__IO  uint32_t	RSVD011:8;
		/* 0x08~ 0x0B*/	
		__IO  uint32_t	TIMER4_COUNTER:24;
		__IO  uint32_t	RSVD021:8;
		};
  };
	/* 0x0C~ 0x17*/	
	__IO  uint32_t	RSVD03[3];
	/* 0x18~ 0x1B*/	
	__IO  uint32_t	INTERRUPT_OUT:1;
	__IO  uint32_t	RSVD04:31;	
	/* 0x1C~ 0x1F*/	
	__IO  uint32_t	TIMER_INTERRUPT_ENABLE:1;
	__IO  uint32_t	RSVD05:31;	
	/* 0x20~ 0x23*/	
	__IO  uint32_t	TIMER_INTERRUPT_DISABLE:1;
	__IO  uint32_t	RSVD06:31;	
}TIMER_CTRL_TYPE;

/* ============================ WDT =========================== */
typedef struct {
    /* 0x00 ~ 0x03 */
    __IO  uint32_t  TASK_START:1;
    __I   uint32_t  RSVD00:31;
    /* 0x04 ~ 0x07 */
    __IO  uint32_t  EVENTS_TIME_OUT:1;
    __I   uint32_t  RSVD01:31;
    /* 0x08 ~ 0x0B */
    __IO  uint32_t  WDT_INTERRUPT_ENABLE:1;
    __I   uint32_t  RSVD02:31;
    /* 0x0C ~ 0x0F */
    __IO  uint32_t  WDT_INTERRUPT_DISABLE:1;
    __I   uint32_t  RSVD03:31;
    /* 0x10 ~ 0x13 */
    __IO  uint32_t  RUN_STATUS_OF_WDT:1;
    __I   uint32_t  RSVD04:31;
    /* 0x14 ~ 0x17 */
    __IO  uint32_t  REQ_STATUS_OF_RR0:1;
    __IO  uint32_t  REQ_STATUS_OF_RR1:1;
    __IO  uint32_t  REQ_STATUS_OF_RR2:1;
    __IO  uint32_t  REQ_STATUS_OF_RR3:1;
    __IO  uint32_t  REQ_STATUS_OF_RR4:1;
    __IO  uint32_t  REQ_STATUS_OF_RR5:1;
    __IO  uint32_t  REQ_STATUS_OF_RR6:1;
    __IO  uint32_t  REQ_STATUS_OF_RR7:1;
    __I   uint32_t  RSVD05:24;
    /* 0x18 ~ 0x1B */
    __IO  uint32_t  COUNTER_RELOAD_VALUE;
    /* 0x1C ~ 0x1F */
    __IO  uint32_t  ENABLE_DISABLE_OF_RR0:1;
    __IO  uint32_t  ENABLE_DISABLE_OF_RR1:1;
    __IO  uint32_t  ENABLE_DISABLE_OF_RR2:1;
    __IO  uint32_t  ENABLE_DISABLE_OF_RR3:1;
    __IO  uint32_t  ENABLE_DISABLE_OF_RR4:1;
    __IO  uint32_t  ENABLE_DISABLE_OF_RR5:1;
    __IO  uint32_t  ENABLE_DISABLE_OF_RR6:1;
    __IO  uint32_t  ENABLE_DISABLE_OF_RR7:1;
    __I   uint32_t  RSVD06:24;
    /* 0x20 ~ 0x23 */
    __IO  uint32_t  CONFIGURATION_REGISTER;
    /* 0x24 ~ 0x27 */
    __IO  uint32_t  RR0_RELOAD_REGISTER;
    /* 0x28 ~ 0x2B */
    __IO  uint32_t  RR1_RELOAD_REGISTER;
    /* 0x2C ~ 0x2F */
    __IO  uint32_t  RR2_RELOAD_REGISTER;
    /* 0x30 ~ 0x33 */
    __IO  uint32_t  RR3_RELOAD_REGISTER;
    /* 0x34 ~ 0x37 */
    __IO  uint32_t  RR4_RELOAD_REGISTER;
    /* 0x38 ~ 0x3B */
    __IO  uint32_t  RR5_RELOAD_REGISTER;
    /* 0x3C ~ 0x3F */
    __IO  uint32_t  RR6_RELOAD_REGISTER;
    /* 0x40 ~ 0x43 */
    __IO  uint32_t  RR7_RELOAD_REGISTER;
} WDT_CTRL_TYPE;

#define    WDT_RR_NUM    8

typedef struct {
    /* 0x00 ~ 0x03 */
    __IO  uint32_t  TASK_START;
    /* 0x04 ~ 0x07 */
    __IO  uint32_t  EVENTS_TIME_OUT;
    /* 0x08 ~ 0x0B */
    __IO  uint32_t  INTENSET;
    /* 0x0C ~ 0x0F */
    __IO  uint32_t  INTENCLR;
    /* 0x10 ~ 0x13 */
    __I   uint32_t  RUN_STATUS;
    /* 0x14 ~ 0x17 */
    __I   uint32_t  REQ_STATUS;
    /* 0x18 ~ 0x1B */
    __IO  uint32_t  CRV;
    /* 0x1C ~ 0x1F */
    __IO  uint32_t  RR_EN;
    /* 0x20 ~ 0x23 */
    __IO  uint32_t  CONFIG;
    /* 0x24 ~ 0x43 */
    __IO  uint32_t  RR[WDT_RR_NUM];
} WDT_CTRL_TYPE2;

/* ============================ PAD control =============================== */
typedef struct {
    /* 0x00 ~ 0x03*/
    __IO  uint32_t  PAD_DS;
    /* 0x04 ~ 0x07*/
    __IO  uint32_t  PAD_PUP;
    /* 0x08 ~ 0x0B*/
    __IO  uint32_t  PAD_PDWN;
    /* 0x0C ~ 0x0F*/
    __IO  uint32_t  PAD_IE;
    /* 0x10 ~ 0x13*/
    __IO  uint32_t  SWCLK_DS:1;
    __IO  uint32_t  SWD_DS:1;
    __IO  uint32_t  FLASH_SCLK_DS:1;
    __IO  uint32_t  FLASH_CS_DS:1;
    __IO  uint32_t  FLASH_SIO_DS:4;
    __IO  uint32_t  SWCLK_PUP:1;
    __IO  uint32_t  SWD_PUP:1;
    __IO  uint32_t  FLASH_SCLK_PUP:1;
    __IO  uint32_t  FLASH_CS_PUP:1;
    __IO  uint32_t  FLASH_SIO_PUP:4;
    __IO  uint32_t  SWCLK_PDWN:1;
    __IO  uint32_t  SWD_PDWN:1;
    __IO  uint32_t  FLASH_SCLK_PDWN:1;
    __IO  uint32_t  FLASH_CS_PDWN:1;
    __IO  uint32_t  FLASH_SIO_PDWN:4;
    __IO  uint32_t  SWCLK_IE:1;
    __IO  uint32_t  SWD_IE:1;
    __IO  uint32_t  FLASH_SCLK_IE:1;
    __IO  uint32_t  FLASH_CS_IE:1;
    __IO  uint32_t  FLASH_SIO_IE:4;
    /* 0x14 ~ 0x17*/
    __IO  uint32_t  PAD3_DS:8;
    __IO  uint32_t  PAD3_PUP:8;
    __IO  uint32_t  PAD3_PDWN:8;
    __IO  uint32_t  PAD3_IE:8;
    /* 0x18 ~ 0x1B*/
    __IO  uint32_t  INT_ANA_MASK_REG:1;
    __IO  uint32_t  INT_KB_MASK_REG:1;
    __IO  uint32_t  INT_MS_MASK_REG:1;
    __IO  uint32_t  INT_AAR_MASK_REG:1;
    __IO  uint32_t  INT_CMAC_MASK_REG:1;
    __IO  uint32_t  INT_ECDH_MASK_REG:1;
    __IO  uint32_t  RSVD01:2;
    __IO  uint32_t  INT_ANA_RAW_STAT:1;
    __IO  uint32_t  INT_KB_RAW_STAT:1;
    __IO  uint32_t  INT_MS_RAW_STAT:1;
    __IO  uint32_t  INT_AAR_RAW_STAT:1;
    __IO  uint32_t  INT_CMAC_RAW_STAT:1;
    __IO  uint32_t  INT_ECDH_RAW_STAT:1;
    __IO  uint32_t  RSVD02:2;
    __IO  uint32_t  INT_ANA_MASKED_STAT:1;
    __IO  uint32_t  INT_KB_MASKED_STAT:1;
    __IO  uint32_t  INT_MS_MASKED_STAT:1;
    __IO  uint32_t  INT_AAR_MASKED_STAT:1;
    __IO  uint32_t  INT_CMAC_MASKED_STAT:1;
    __IO  uint32_t  INT_ECDH_MASKED_STAT:1;
    __IO  uint32_t  RSVD03:2;
    __IO  uint32_t  RSVD04:8;
    /* 0x1C ~ 0x1F*/
    __IO  uint32_t  DEBUG_BUS_SEL:8;
    __IO  uint32_t  RSVD05:24;
} PAD_CTRL_TYPE;

/* ============================ PMU =========================== */
typedef struct {
	/* 0x00 ~ 0x03*/
	__IO  uint32_t  WAKEUP_PIN_SEL;
    /* 0x04 ~ 0x07*/
	__IO  uint32_t  WAKEUP_PIN_POL;
    /* 0x08 ~ 0x0B*/
    union {
        __IO  uint8_t   PMU_WAKEUP_EN;
        struct {
            __IO  uint32_t  PIN_WAKE_EN:1;
            __IO  uint32_t  TIMER_WAKE_EN:1;
            __IO  uint32_t  FSM_SLEEP_EN:1;
            __IO  uint32_t  ANA_WAKE_EN:1;
            __IO  uint32_t  RTC_WAKE_EN:1;
            __IO  uint32_t  WDT_WAKE_EN:1;
            __IO  uint32_t  CAPDET_WAKE_EN:1;
            __IO  uint32_t  RSVD00:1;
        };
    };
    __IO  uint32_t  RFMODE_MANUAL:5;
    __IO  uint32_t  RSVD01:2;
    __IO  uint32_t  RFMODE_MANUAL_EN:1;
    __IO  uint32_t  BLE_DISABLE:1;
    __IO  uint32_t  PER_DISABLE:1;
    __IO  uint32_t  DSLEEP_LPO_EN:1;
    __IO  uint32_t  SLEEP_RC_EN:1;
    __IO  uint32_t  MRR_MANU_ON:1;
    __IO  uint32_t  MRR_MANU_OFF:1;
    __IO  uint32_t  UART_EN:1;
    __IO  uint32_t  CAPDET_EN:1;
    __IO  uint32_t  LPS06_MANU_EN:1;
    __IO  uint32_t  LPS06_MANU_ON:1;
    __IO  uint32_t  LPS09_MANU_EN:1;
    __IO  uint32_t  LPS09_MANU_ON:1;
    __IO  uint32_t  MAN_PMU_EN:1;
    __IO  uint32_t  MAN_PMU_SLEEP:1;
    __IO  uint32_t  MAN_PMU_DSLEEP:1;
    __IO  uint32_t  FLASH_LDO_EN:1;
    /* 0x0C ~ 0F */
    __IO  uint32_t  ENGINE_PWR_UP_WAIT:12;
    __IO  uint32_t  RSVD02:4;
    __IO  uint32_t  BLE_PWR_UP_WAIT:12;
    __IO  uint32_t  RSVD03:4;
    /* 0x10 ~ 13 */
    __IO  uint32_t  PER_PWR_UP_WAIT:12;
    __IO  uint32_t  RSVD04:4;
    __IO  uint32_t  PMU_STATE:4;
    __IO  uint32_t  BLE_PWR_EN:1;
    __IO  uint32_t  BLE_ISO_EN:1;
    __IO  uint32_t  PER_PWR_EN:1;
    __IO  uint32_t  PER_ISO_EN:1;
    __IO  uint32_t  RSVD05:8;
    /* 0x14 ~ 17 */
    __IO  uint32_t  SRAM_B0_OFF:1;
    __IO  uint32_t  SRAM_B1_OFF:1;
    __IO  uint32_t  SRAM_B2_OFF:1;
    __IO  uint32_t  SRAM_B3_OFF:1;
    __IO  uint32_t  SRAM_B4_OFF:1;
    __IO  uint32_t  SRAM_B5_OFF:1;
    __IO  uint32_t  SRAM_B6_OFF:1;
    __IO  uint32_t  SRAM_B7_OFF:1;
    __IO  uint32_t  SRAM_B8_OFF:1;
    __IO  uint32_t  SRAM_B9_OFF:1;
    __IO  uint32_t  SRAM_B10_OFF:1;
    __IO  uint32_t  SRAM_B11_OFF:1;
    __IO  uint32_t  RSVD06:20;
    /* 0x18 ~ 1B */
    __IO  uint8_t   WAKEUP_PIN2_SEL;
    __IO  uint8_t   WAKEUP_PIN2_POL;
    __IO  uint8_t   RSVD07[2];
}PMU_CTRL_TYPE;


typedef struct {
	/* 0x08 ~ 0x09*/	
	__IO  uint16_t  SW_INT_SET:8;	
	__IO  uint16_t  SW_INT_CLR:8;	
}SW_INT_CTRL_TYPE;

/* ============================ Serial Interface Common =========================== */
typedef struct {
    /* 0x00 ~ 0x03 */
    __IO  uint32_t  B_DEF:8;
    __IO  uint32_t  B_ORC:8;
    __IO  uint32_t  B_I2CS_DEBOUNCE_THR:4;
    __IO  uint32_t  B_SPIM_SPEED:3;
    __IO  uint32_t  B_SPIM_RESYNC:1;
    __IO  uint32_t  B_CPOL:1;
    __IO  uint32_t  B_CPHA:1;
    __IO  uint32_t  UART_SWRST:1;
    __IO  uint32_t  I2CM1_SWRST:1;
    __IO  uint32_t  I2CM0_SWRST:1;
    __IO  uint32_t  I2CS_SWRST:1;
    __IO  uint32_t  SPIM_SWRST:1;
    __IO  uint32_t  SPIS_SWRST:1;
} SERIAL_IF_CTRL_TYPE;
/* ============================ SPI Slave =========================== */
typedef struct {
    /* 0x00 ~ 0x03 */
    __IO  uint32_t  SPIS_ENABLE:1;
    __IO  uint32_t  RSVD00:7;
    __IO  uint32_t  MCU_TRIG_TXDMA:1;
    __IO  uint32_t  RSVD01:7;
    __IO  uint32_t  SPIS_TX_MAXCNT:6;
    __IO  uint32_t  RSVD02:2;
    __IO  uint32_t  SPIS_RX_MAXCNT:6;
    __IO  uint32_t  RSVD03:2;
    /* 0x04 ~ 0x07 */
    __IO  uint32_t  SPIS_INT_MASK:1;
    __IO  uint32_t  RSVD04:7;
    __IO  uint32_t  SPIS_INT_CLEAR:1;
    __IO  uint32_t  RSVD05:7;
    __I   uint32_t  INT_SPIS_ENDTR:1;
    __I   uint32_t  INT_SPIS_OVERRD:1;
    __I   uint32_t  INT_SPIS_OVERWR:1;
    __IO  uint32_t  RSVD06:5;
    __I   uint32_t  STATUS_TRX_LENGTH:6;
    __IO  uint32_t  RSVD07:2;
    /* 0x08 ~ 0x0B */
    __IO  uint32_t  SPIS_RX_PTR:18;
    __IO  uint32_t  RSVD08:14;
    /* 0x0c ~ 0x0F */
    __IO  uint32_t  SPIS_TX_PTR:18;
    __IO  uint32_t  RSVD09:14;
} SPI_SLAVE_CTRL_TYPE;
/* ============================ SPI Master =========================== */
typedef struct {
    /* 0x00 ~ 0x03 */
    __IO  uint32_t  SPIM_ENABLE:1;
    __IO  uint32_t  RSVD00:7;
    __IO  uint32_t  MCU_TRIG_TRX:1;
    __IO  uint32_t  RSVD01:7;
    __IO  uint32_t  SPIM_TX_MAXCNT:6;
    __IO  uint32_t  RSVD02:2;
    __IO  uint32_t  SPIM_RX_MAXCNT:6;
    __IO  uint32_t  RSVD03:2;
    /* 0x04 ~ 0x07 */
    __IO  uint32_t  SPIM_INT_MASK:1;
    __IO  uint32_t  RSVD04:7;
    __IO  uint32_t  SPIM_INT_CLEAR:1;
    __IO  uint32_t  RSVD05:7;
    __I   uint32_t  INT_SPIM_ENDTR:1;
    __IO  uint32_t  RSVD07:15;
    /* 0x08 ~ 0x0B */
    __IO  uint32_t  SPIM_RX_PTR:18;
    __IO  uint32_t  RSVD028:14;
    /* 0x0c ~ 0x0F */
    __IO  uint32_t  SPIM_TX_PTR:18;
    __IO  uint32_t  RSVD029:14;
} SPI_MASTER_CTRL_TYPE;
/* ============================ I2C Slave =========================== */
typedef struct {
    /* 0x00 ~ 0x03 */
    __IO  uint32_t  I2CS_ENABLE:1;
    __I   uint32_t  RSVD00:7;
    __IO  uint32_t  B_I2CS_ADDR_MODE:2;
    __I   uint32_t  RSVD01:6;
    __IO  uint32_t  I2CS_SLAVE_ID1:7;
    __I   uint32_t  RSVD02:1;
    __IO  uint32_t  I2CS_SLAVE_ID2:7;
    __I   uint32_t  RSVD03:1;
    /* 0x04 ~ 0x07 */
    __IO  uint32_t  I2CS_INT_MASK:1;
    __I   uint32_t  RSVD04:7;
    __IO  uint32_t  I2CS_INT_CLEAR:1;
    __I   uint32_t  RSVD05:7;
    union {
        __I   uint8_t   INT_STATUS;
        struct {
            __I   uint32_t  INT_I2CS_STOPPED:1;
            __I   uint32_t  INT_I2CS_READ:1;
            __I   uint32_t  INT_I2CS_WRITE:1;
            __I   uint32_t  RSVD06:5;
        };
    };
    __I   uint32_t  RSVD07:8;
    /* 0x08 ~ 0x0F */
    union {
        struct {
            __IO  uint32_t  RX_PTR;
            __IO  uint32_t  TX_PTR;
        };
        struct {
            __IO  uint32_t  I2CS_RX_PTR:18;
            __I   uint32_t  RSVD08:14;
            __IO  uint32_t  I2CS_TX_PTR:18;
            __I   uint32_t  RSVD09:14;
        };
    };
    /* 0x10 ~ 0x5F */
    __I   uint32_t  RSVD0A[20];
    /* 0x60 ~ 0x63 */
    __IO  uint32_t  I2CS_TX_MAXCNT:16;
    __IO  uint32_t  I2CS_RX_MAXCNT:16;
} I2C_SLAVE_CTRL_TYPE;
/* ============================ I2C Master =========================== */
typedef struct {
    /* 0x00 ~ 0x03 */
    __IO  uint32_t  I2CM_ENABLE:1;
    __I   uint32_t  RSVD00:7;
    __IO  uint32_t  I2CM_MCU_TRIG_RD:1;
    __IO  uint32_t  I2CM_MCU_TRIG_WR:1;
    __I   uint32_t  RSVD01:4;
    __IO  uint32_t  I2CM_TX_MAXCNT:9;
    __IO  uint32_t  I2CM_RX_MAXCNT:9;
    /* 0x04 ~ 0x07 */
    __IO  uint32_t  I2CM_INT_MASK:1;
    __I   uint32_t  RSVD02:7;
    __IO  uint32_t  I2CM_INT_CLEAR:1;
    __I   uint32_t  RSVD03:7;
    __I   uint32_t  INT_I2CM_STOPPED:1;
    __I   uint32_t  I2CM_NAK_FLAG:1;
    __I   uint32_t  RSVD04:5;
    __I   uint32_t  INT_I2CM_TRX_LENGTH:9;
    /* 0x08 ~ 0x0B */
    union {
        __IO  uint32_t  RX_PTR;
        struct {
            __IO  uint32_t  I2CM_RX_PTR:18;
            __I   uint32_t  RSVD05:14;
        };
    };
    /* 0x0C ~ 0x0F */
    union {
        __IO  uint32_t  TX_PTR;
        struct {
            __IO  uint32_t  I2CM_TX_PTR:18;
            __I   uint32_t  RSVD06:14;
        };
    };
    /* 0x10 ~ 0x13 */
    __IO  uint32_t  B_I2CM_DEV_WRADDR:16;
    __IO  uint32_t  B_I2CM_DEV_INST:7;
    __IO  uint32_t  B_I2CM_ADDR1B_MODE:1;
    __IO  uint32_t  B_I2CM_SPEED:3;
    __IO  uint32_t  B_I2CM_CURR_ADDR:1;
    __IO  uint32_t  RSVD7:4;
} I2C_MASTER_CTRL_TYPE;
/* ============================ UART =========================== */
typedef struct {
    /* 0x00 ~ 0x03 */
    union {
        __IO  uint8_t   INT_STATUS;
        struct {
            __IO  uint32_t  BAUD_SEL:4;
            __IO  uint32_t  RXD_INT:1;
            __IO  uint32_t  TXD_INT:1;
            __IO  uint32_t  INT_RX_MASK:1;
            __IO  uint32_t  FLOWCTRL_EN:1;
        };
    };
    __IO  uint32_t  TX_DATA:8;
    __IO  uint32_t  BAUD_CLK_SEL:1;
    __IO  uint32_t  PARITY_FAIL_STATUS:1;
    __IO  uint32_t  PARITY_FAIL_MASK:1;
    __IO  uint32_t  UART_ENABLE:1;
    __IO  uint32_t  RX_PARITY_ENABLE:1;
    __IO  uint32_t  RX_PARITY_EVEN:1;
    __IO  uint32_t  TX_PARITY_ENABLE:1;
    __IO  uint32_t  TX_PARITY_EVEN:1;
    __I   uint32_t  RXFF_EMPTY:1;
    __I   uint32_t  RXFF_FULL:1;
    __I   uint32_t  TXFF_EMPTY:1;
    __I   uint32_t  TXFF_FULL:1;
    __IO  uint32_t  INT_TX_MASK:1;
    __I   uint32_t  RSVD01:3;
    /* 0x04 ~ 0x07 */
    __IO  uint32_t  RX_DATA:8;
    __I   uint32_t  RSVD02:24;
} UART_CTRL_TYPE;
/* ============================ Flash Controler =========================== */
typedef struct {	
	/* 0x00~ 0x03*/
	__IO  uint32_t	TASK_START:1;	
	__IO  uint32_t	RSVD0:31;	
	/* 0x04~ 0x07*/
	__IO  uint32_t	TRANS_TX_LEN:4;
	__IO  uint32_t	TRANS_RX_LEN:4;	
	__IO  uint32_t	RSVD4:24;
    union {
        struct {
	        /* 0x08~ 0x0B*/
	        __IO  uint32_t	TRANS_ADDR:24;	
	        __IO  uint32_t	TRANS_CMD:8;
	        /* 0x0C~ 0x0F*/
	        __IO  uint32_t	TRANS_DUMMY;
        };
        struct {
            __IO  uint32_t  U32BUF[2];
        };
    };
	/* 0x10~ 0x13*/
    union {
        struct {
            __IO  uint8_t   QSPI_W_MODE:8;
            __IO  uint32_t  QSPI_R_MODE:1;
            __I   uint32_t  RSVD10_1:23;
        };
        struct {
	__IO  uint32_t	QSPI_MODE:9;
	__IO  uint32_t	RSVD10:23;
        };
    };
	/* 0x14~ 0x17*/
	__IO  uint32_t	READ_DELAY_HALF_CYCLE:1;
	__IO  uint32_t	READ_DELAY_ONE_CYCLE:1;	
	__IO  uint32_t	RSVD14:30;	
	/* 0x18~ 0x1B*/
	__IO  uint32_t	READ_STATUS;
	/* 0x1C~ 0x1F*/
	__IO  uint32_t	TASK_FINISH:1;
	__IO  uint32_t	RSVD1C:31;
	/* 0x20~ 0x23*/
	__IO  uint32_t	ENHANCE_MODE:1;
	__IO  uint32_t	RSVD20:31;
	/* 0x24~ 0x27*/
	__IO  uint32_t	APB_CTRL:1;
	__IO  uint32_t	RSVD24:31;
	/* 0x28~ 0x3B*/
	__IO  uint32_t	RSVD28[5];
	/* 0x3C~ 0x3F*/
	__IO  uint32_t	CONTINUE_WORDS:7;
	__IO  uint32_t	RSVD3C:25;
	/* 0x40~ 0x03*/
	__IO  uint32_t	HOLD_MANUAL_EN:1;	
	__IO  uint32_t	RSVD400:3;	
	__IO  uint32_t	HOLD_MAN_VAL:1;	
	__IO  uint32_t	RSVD401:3;
	__IO  uint32_t	WP_MANUAL_EN:1;	
	__IO  uint32_t	RSVD402:3;	
	__IO  uint32_t	WP_MAN_VAL:1;	
	__IO  uint32_t	RSVD403:3;
	__IO  uint32_t	CS_MANUAL_EN:1;	
	__IO  uint32_t	RSVD404:3;	
	__IO  uint32_t	CS_MAN_VAL:1;	
	__IO  uint32_t	RSVD405:11;
}FLASH_CTRL_TYPE;

/* ============================ ECDH =========================== */
typedef struct {
	/* 0x00 */
	__IO  uint8_t  ECDH_WRSTN:1;
	__IO  uint8_t  RSVD00:7;	
	/* 0x01 */
	__IO  uint8_t  ECDH_X_START:1;
	__IO  uint8_t  ECDH_Y_START:1;
	__IO  uint8_t  RSVD1:6;
	/* 0x02 ~ 0x03*/
	__IO  uint16_t RSVD02;
	/* 0x04 ~ 0x07*/
	__IO  uint32_t  ECDH_POINTER:18; 
	__IO  uint32_t  RSVD04:5;
	__IO  uint32_t  ECDH_BSTLMT_EN:1;	
	__IO  uint32_t  ECDH_MAX_BST_LEN:8;
	/* 0x08 ~ 0x0B */
	__IO  uint8_t  RSVD07[4];
	/* 0x0C ~ 0x0D*/
	__IO  uint16_t R_SKEY_M:9;
	__IO  uint16_t RSVD0C:7;	
	/* 0x0E ~ 0x0F*/
	__IO  uint16_t RSVD0E;
	/* 0x10 ~ 2F*/
	__IO  uint32_t SKEY[8];
	/* 0x30 ~ 4F*/
	__IO  uint32_t PKEY_X[8];
	/* 0x50 ~ 6F*/
	__IO  uint32_t PKEY_Y[8];
	/* 0x70 ~ 8F*/
	__IO  uint32_t DH_KEY[8];
	/* 0x90*/
	__IO  uint8_t ECDH_DONE_INT_EN:1;
	__IO  uint8_t RSVD90:7;	
	/* 0x91*/
	__IO  uint8_t ECDH_DONE_STS:1;
	__IO  uint8_t RSVD91:7;	

} ECDH_CTRL_TYPE;


/* ============================ CMAC =========================== */
typedef struct {
	/* 0x100 */
	__IO  uint8_t  CMAC_WRSTN:1;
	__IO  uint8_t  RSVD100:7;
	/* 0x101 */
	__IO  uint8_t  CMAC_START:1;
	__IO  uint8_t  RSVD101:7;
	/* 0x102 ~ 0x103 */
	__IO  uint16_t  RSVD102;
	/* 0x104 ~ 0x107*/
	__IO  uint32_t  CMAC_POINTER:18;		
	__IO  uint32_t  RSVD105:5;		
	__IO  uint32_t  CMAC_BSTLMT_EN:1;			
	__IO  uint32_t  CMAC_MAX_BST_LEN:8;	
	/* 0x108 ~ 0x109*/
	__IO  uint16_t  CMAC_M_LEN:10;			
	__IO  uint16_t  RSVD108:6;
	/* 0x10A ~ 0x10F*/
	__IO  uint8_t RSVD10A[6];
	/* 0x110 ~ 0x11F*/
	__IO  uint32_t CMAC_KEY[4];
	/* 0x120 ~ 0x12F*/
	__IO  uint32_t CMAC_MAC[4];	
	/* 0x130 */
	__IO  uint8_t  CMAC_DONE_INT_EN:1;			
	__IO  uint8_t  RSVD130:7;				
	/* 0x131 */
	__IO  uint8_t  CMAC_DONE_STS:1;			
	__IO  uint8_t  RSVD131:7;			

}CMAC_CTRL_TYPE;

/* ============================ AAR =========================== */
typedef struct {
	/* 0x200 */

	__IO  uint8_t  AAR_WRSTN:1; 			
	__IO  uint8_t  RSVD200:7; 			
	/* 0x201 */
	__IO  uint8_t  AAR_START:1; 			
	__IO  uint8_t  RSVD201:7; 			
	/* 0x202 ~ 203 */
	__IO  uint16_t  RSVD202; 			
	/* 0x204 ~ 207*/	
	__IO  uint32_t  AAR_POINTER:18; 			
	__IO  uint32_t  RSVD204:5; 			
	__IO  uint32_t  AAR_BSTLMT_EN:1; 			
	__IO  uint32_t  AAR_MAX_BST_LEN:8; 	
	/* 0x208 */	
	__IO  uint8_t  AAR_IRK_LEN:5; 					
	__IO  uint8_t  RSVD208:3; 		
	/* 0x209 ~ 20F*/	
	__IO  uint8_t RSVD209[7]; 			
	/* 0x210 ~ 215*/	
	__IO  uint8_t  AAR_PRAND[3]; 			
	__IO  uint8_t  AAR_HASH[3]; 			
	/* 0x216 ~ 21F*/	
	__IO  uint8_t  RSVD216[10]; 			
	/* 0x220*/	
	__IO  uint8_t AAR_MIRK:5; 			
	__IO  uint8_t RSVD220:2; 			
	__IO  uint8_t AAR_RESOLVED:1;	
	/* 0x221 ~ 22F*/	
	__IO  uint8_t  RSVD221[15]; 				
	/* 0x230*/	
	__IO  uint8_t AAR_DONE_INT_EN:1; 			
	__IO  uint8_t RSVD230:7; 			
	/* 0x231*/	
	__IO  uint8_t AAR_DONE_STS:1; 			
	__IO  uint8_t RSVD231:7; 			

}AAR_CTRL_TYPE;

/* ============================ PAD define =========================== */
typedef struct {
	/* 0x00 ~ 0x0F */	
	__IO  uint32_t  PAD0_MUX_SEL:4;
	__IO  uint32_t  PAD1_MUX_SEL:4;	
	__IO  uint32_t  PAD2_MUX_SEL:4;
	__IO  uint32_t  PAD3_MUX_SEL:4;	
	__IO  uint32_t  PAD4_MUX_SEL:4;
	__IO  uint32_t  PAD5_MUX_SEL:4;		
	__IO  uint32_t  PAD6_MUX_SEL:4;
	__IO  uint32_t  PAD7_MUX_SEL:4;	
	__IO  uint32_t  PAD8_MUX_SEL:4;
	__IO  uint32_t  PAD9_MUX_SEL:4;	
	__IO  uint32_t  PAD10_MUX_SEL:4;
	__IO  uint32_t  PAD11_MUX_SEL:4;	
	__IO  uint32_t  PAD12_MUX_SEL:4;
	__IO  uint32_t  PAD13_MUX_SEL:4;	
	__IO  uint32_t  PAD14_MUX_SEL:4;
	__IO  uint32_t  PAD15_MUX_SEL:4;	
	__IO  uint32_t  PAD16_MUX_SEL:4;
	__IO  uint32_t  PAD17_MUX_SEL:4;	
	__IO  uint32_t  PAD18_MUX_SEL:4;
	__IO  uint32_t  PAD19_MUX_SEL:4;	
	__IO  uint32_t  PAD20_MUX_SEL:4;
	__IO  uint32_t  PAD21_MUX_SEL:4;	
	__IO  uint32_t  PAD22_MUX_SEL:4;
	__IO  uint32_t  PAD23_MUX_SEL:4;	
	__IO  uint32_t  PAD24_MUX_SEL:4;
	__IO  uint32_t  PAD25_MUX_SEL:4;	
	__IO  uint32_t  PAD26_MUX_SEL:4;	
	__IO  uint32_t  PAD27_MUX_SEL:4;
	__IO  uint32_t  PAD28_MUX_SEL:4;	
	__IO  uint32_t  PAD29_MUX_SEL:4;
	__IO  uint32_t  PAD30_MUX_SEL:4;	
	__IO  uint32_t  PAD31_MUX_SEL:4;	
    /* 0x10 ~ 0x2B */
    __I   uint32_t  RSVD00[7];
    /* 0x2C ~ 0x2F */
    __IO  uint32_t  PAD32_MUX_SEL:4;
    __IO  uint32_t  PAD33_MUX_SEL:4;
    __IO  uint32_t  PAD34_MUX_SEL:4;
    __IO  uint32_t  PAD35_MUX_SEL:4;
    __IO  uint32_t  PAD36_MUX_SEL:4;
    __IO  uint32_t  PAD37_MUX_SEL:4;
    __IO  uint32_t  PAD38_MUX_SEL:4;
    __IO  uint32_t  PAD39_MUX_SEL:4;
}PAD_MUX_SEL;

/* ============================ GPIO =========================== */
typedef struct {
	/* 0x00 */
	__IO  uint32_t  GPIO_OUT;
	/* 0x04 */
	__IO  uint32_t  GPIO_OUT_SET;
	/* 0x08 */
	__IO  uint32_t  GPIO_OUT_CLR;
	/* 0x0C */
	__IO  uint32_t  GPIO_OUT_TOG;
	/* 0x10 */
	__IO  uint32_t  GPIO_DIR;
	/* 0x14 */
	__IO  uint32_t  GPIO_DIR_SET;		
	/* 0x18 */
	__IO  uint32_t  GPIO_DIR_CLR;
	/* 0x1C */
	__IO  uint32_t  GPIO_DIR_TOG;
	/* 0x20 */
	__IO  uint32_t  GPIOINT_ENABLE;
	/* 0x24 */
	__IO  uint32_t  GPIOINT_ENABLE_SET;
	/* 0x28 */
	__IO  uint32_t  GPIOINT_ENABLE_CLR;	
	/* 0x2C */
	__IO  uint32_t  GPIOINT_ENABLE_TOG;
	/* 0x30 */
	__IO  uint32_t  GPIOINT_POL;	
	/* 0x34 */
	__IO  uint32_t  GPIOINT_POL_SET;	
	/* 0x38 */
	__IO  uint32_t  GPIOINT_POL_CLR;	
	/* 0x3C */
	__IO  uint32_t  GPIOINT_POL_TOG;	
	/* 0x40 */
	__IO  uint32_t  GPIOINT_TYPE;	
	/* 0x44 */
	__IO  uint32_t  GPIOINT_TYPE_SET;	
	/* 0x48 */
	__IO  uint32_t  GPIOINT_TYPE_CLR;	
	/* 0x4C */
	__IO  uint32_t  GPIOINT_TYPE_TOG;	
	/* 0x50 */
	__IO  uint32_t  GPIO_IN;	
	/* 0x54 */
	__IO  uint32_t  GPIOINT_STATUS;	
} GPIO_CTRL_TYPE;

/* ============================ M2M =========================== */
typedef struct {
	/* 0x000 */
	__IO  uint8_t  M2M_WRSTN:1; 			
	__IO  uint8_t  RSVD00:7; 			
	/* 0x001 */
	__IO  uint8_t  M2M_START:1; 			
	__IO  uint8_t  M2M_F_END:1;
	__IO  uint8_t  RSVD01:6; 			
	/* 0x002 ~ 0x003 */
	__IO  uint16_t  RSVD02; 			
	/* 0x004 ~ 0x007*/
	__IO  uint32_t  M2M_RD_POINTER:18;	
	__IO  uint32_t  RSVD04:5;
	__IO  uint32_t  M2M_RD_BSTLMT_EN:1;
	__IO  uint32_t  RSVD07:8;
	/* 0x008 ~ 0x00B*/
	__IO  uint32_t  M2M_WR_POINTER:18;	
	__IO  uint32_t  RSVD0A:5;
	__IO  uint32_t  M2M_WR_BSTLMT_EN:1;
	__IO  uint32_t  M2M_MAX_BST_LEN:8;			
	/* 0x00C ~ 0x00D*/
	__IO  uint16_t  M2M_D_LEN; 			
	/* 0x00E ~ 0x00F*/
	__IO  uint16_t  RSVD0E;
	/* 0x010*/
	__IO  uint8_t  M2M_END_INT_EN:1;
	__IO  uint8_t  RSVD10:7;
	/* 0x011*/
	__IO  uint8_t  M2M_END_STS:1;
	__IO  uint8_t  RSVD11:7;

}M2M_CTRL_TYPE;


/* ============================ RF control =========================== */
typedef struct {
	/* 0x00 ~ 0x03 */	
	__IO  uint32_t  RF_ADDR:8;
	__IO  uint32_t  RF_WDATA:8;
	__IO  uint32_t  RF_RDY:1;
	__IO  uint32_t  RSVD00:15;
	/* 0x04 ~ 0x07*/
	__IO  uint32_t  RF_RDATA:8;
	__IO  uint32_t  RSVD04:24;
}RF_CTRL_TYPE;

/* ============================ eFuse   ================================ */
typedef struct {
    /* 0x00 ~ 0x03*/
    __IO  uint32_t  EFUSE_MAN_EN:1;
    __IO  uint32_t  RSVD00:31;
    /* 0x04 ~ 0x07*/
    __IO  uint32_t  EFUSE_MAN_VAL;
    /* 0x08 ~ 0x0B*/
    __IO  uint32_t  EFUSE_OUT_VAL;
}EFUSE_CTRL_TYPE;

/* ============================ Arbiter control ================================ */
typedef struct {
    /* 0x00 ~ 0x03*/
    __IO  uint32_t  REQ_MAP_EN:1;
    __IO  uint32_t  BURST_EN:1;
    __IO  uint32_t  RR_RULE_SEL:2;
    __IO  uint32_t  RR_EN:1;
    __IO  uint32_t  RSVD00:27;
    union {
        __IO  uint16_t  REQ_PRI[16];
        struct {
            /* 0x04 ~ 0x07*/
            __IO  uint32_t  REQ0_PRI:16;
            __IO  uint32_t  REQ1_PRI:16;
            /* 0x08 ~ 0x0B*/
            __IO  uint32_t  REQ2_PRI:16;
            __IO  uint32_t  REQ3_PRI:16;
            /* 0x0C ~ 0x0F*/
            __IO  uint32_t  REQ4_PRI:16;
            __IO  uint32_t  REQ5_PRI:16;
            /* 0x10 ~ 0x13*/
            __IO  uint32_t  REQ6_PRI:16;
            __IO  uint32_t  REQ7_PRI:16;
            /* 0x14 ~ 0x17*/
            __IO  uint32_t  REQ8_PRI:16;
            __IO  uint32_t  REQ9_PRI:16;
            /* 0x18 ~ 0x1B*/
            __IO  uint32_t  REQ10_PRI:16;
            __IO  uint32_t  REQ11_PRI:16;
            /* 0x1C ~ 0x1F*/
            __IO  uint32_t  REQ12_PRI:16;
            __IO  uint32_t  REQ13_PRI:16;
            /* 0x20 ~ 0x23*/
            __IO  uint32_t  REQ14_PRI:16;
            __IO  uint32_t  REQ15_PRI:16;
        };
    };
} ARBITER_CTRL_TYPE;

/* ============================ UART2 =========================== */
#pragma pack(push, 4)
typedef struct {
    /* 0x00 ~ 0x03 */
    union{
        __O   uint32_t  THR;
        __I   uint32_t  RBR;
        __IO  uint32_t  DLL;
    };
    /* 0x04 ~ 0x07 */
    union{
        struct {
            __IO  uint32_t  ERBFI:1;
            __IO  uint32_t  ETBEI:1;
            __IO  uint32_t  ELSI:1;
            __IO  uint32_t  EDSSI:1;
            __I   uint32_t  RSVD00:3;
            __IO  uint32_t  PTIME:1;
            __I   uint32_t  RSVD01:24;
        };
        struct {
            __IO  uint32_t  DLH:8;
            __I   uint32_t  RSVD02:24;
        };
    };
    /* 0x08 ~ 0x0B */
    union {
        struct {
            __I   uint32_t  INTERRUPT_ID:4;
            __I   uint32_t  RSVD03:2;
            __I   uint32_t  FIFOS_ENABLED:2;
            __I   uint32_t  RSVD04:24;
        };
        __O   uint32_t  FCR;   // FIFO Control
    };
    /* 0x0C ~ 0x0F */
    __IO  uint32_t  DLS:2;
    __IO  uint32_t  STOP:1;
    __IO  uint32_t  PEN:1;
    __IO  uint32_t  EPS:1;
    __IO  uint32_t  SP:1;
    __IO  uint32_t  BC:1;
    __IO  uint32_t  DLAB:1;
    __I   uint32_t  RSVD05:24;
    /* 0x10 ~ 0x13 */
    __IO  uint32_t  DTR:1;
    __IO  uint32_t  RTS:1;
    __IO  uint32_t  OUT1:1;
    __IO  uint32_t  OUT2:1;
    __IO  uint32_t  LB:1;
    __IO  uint32_t  AFCE:1;
    __IO  uint32_t  SIRE:1;
    __I   uint32_t  RSVD06:25;
    /* 0x14 ~ 0x17 */
    __I   uint32_t  LSR;
    /* 0x18 ~ 0x1B */
    __IO  uint32_t  DCTS:1;
    __IO  uint32_t  DDSR:1;
    __IO  uint32_t  TERI:1;
    __IO  uint32_t  DDCD:1;
    __IO  uint32_t  CTS:1;
    __IO  uint32_t  DSR:1;
    __IO  uint32_t  RI:1;
    __IO  uint32_t  DCD:1;
    __I   uint32_t  RSVD07:24;
    /* 0x1C ~ 0x1F */
    __IO  uint32_t  SCRATCHPAD:8;
    __I   uint32_t  RSVD08:24;
    /* 0x20 ~ 0x23 */
    __IO  uint32_t  LPDLL:8;
    __I   uint32_t  RSVD09:24;
    /* 0x24 ~ 0x27 */
    __IO  uint32_t  LPDLH:8;
    __I   uint32_t  RSVD0A:24;
    /* 0x28 ~ 0x2B */
    __I   uint32_t  RSVD0B;
    /* 0x2C ~ 0x2F */
    __IO  uint32_t  RSVD0C;
    /* 0x30 ~ 0x33 */
    __IO  uint32_t  SHADOW_RECEIVE_BUFFER_REGISTER:9;
    __I   uint32_t  RSVD0D:23;
    /* 0x34 ~ 0x6F */
    __I   uint32_t  RSVD0E[15];
    /* 0x70 ~ 0x73 */
    __IO  uint32_t  FIFO_ACCESS_REGISTER:1;
    __IO  uint32_t  RSVD0F:31;
    /* 0x74 ~ 0x77 */
    __IO  uint32_t  TRANSMIT_FIFO_READ:8;
    __I   uint32_t  RSVD10:24;
    /* 0x78 ~ 0x7B */
    __IO  uint32_t  RFWD:8;
    __IO  uint32_t  RFPE:1;
    __IO  uint32_t  RFFE:1;
    __I   uint32_t  RSVD11:22;
    /* 0x7C ~ 0x7F */
    __IO  uint32_t  BUSY:1;
    __IO  uint32_t  TFNF:1;
    __IO  uint32_t  TFE:1;
    __IO  uint32_t  RFNE:1;
    __IO  uint32_t  RFF:1;
    __I   uint32_t  RSVD12:27;
    /* 0x80 ~ 0x83 */
    __IO  uint32_t  TRANSMIT_FIFO_LEVEL:6;
    __I   uint32_t  RSVD13:26;
    /* 0x84 ~ 0x87 */
    __IO  uint32_t  RECEIVE_FIFO_LEVEL:6;
    __I   uint32_t  RSVD14:26;
    /* 0x88 ~ 0x8B */
    __IO  uint32_t  UR:1;
    __IO  uint32_t  EFR:1;
    __IO  uint32_t  XFR:1;
    __I   uint32_t  RSVD15:29;
    /* 0x8C ~ 0x8F */
    __IO  uint32_t  SRTS:1;
    __I   uint32_t  RSVD16:31;
    /* 0x90 ~ 0x93 */
    __IO  uint32_t  SBCR:1;
    __I   uint32_t  RSVD17:31;
    /* 0x94 ~ 0x97 */
    __IO  uint32_t  SDMAM:1;
    __I   uint32_t  RSVD18:31;
    /* 0x98 ~ 0x9B */
    __IO  uint32_t  SFE:1;
    __I   uint32_t  RSVD19:31;
    /* 0x9C ~ 0x9F */
    __IO  uint32_t  SRT:1;
    __I   uint32_t  RSVD1A:31;
    /* 0xA0 ~ 0xA3 */
    __IO  uint32_t  STET:2;
    __I   uint32_t  RSVD1B:30;
    /* 0xA4 ~ 0xA7 */
    __IO  uint32_t  HTX:1;
    __I   uint32_t  RSVD1C:31;
    /* 0xA8 ~ 0xAB */
    __IO  uint32_t  DMASA:1;
    __I   uint32_t  RSVD1D:31;
    /* 0xAC ~ 0xBF */
    __I   uint32_t  RSVD1E[5];
    /* 0xC0 ~ 0xC3 */
    __IO  uint32_t  DLF:4;
    __I   uint32_t  RSVD1F:28;
    /* 0xC4 ~ 0xC7 */
    __IO  uint32_t  RAR:8;
    __I   uint32_t  RSVD20:24;
    /* 0xC8 ~ 0xCB */
    __IO  uint32_t  TAR:8;
    __I   uint32_t  RSVD21:24;
    /* 0xCC ~ 0xCF */
    __IO  uint32_t  DLS_E:1;
    __IO  uint32_t  ADDR_MATCH:1;
    __IO  uint32_t  SEND_ADDR:1;
    __IO  uint32_t  TRANSMIT_MODE:1;
    __I   uint32_t  RSVD22:28;
} UART2_CTRL_TYPE;
#pragma pack(pop)

/* ============================ HID Controler =========================== */
typedef struct {
    /* 0x00 ~ 0x03 */
    __IO  uint32_t  R_HID_SEN_EN:1;
    __IO  uint32_t  R_HID_BTN_EN:1;
    __IO  uint32_t  RSVD00:6;
    union {
        struct {
            __IO  uint32_t  BTN_INIT:1;
            __IO  uint32_t  POLL_REQ:1;
            __IO  uint32_t  HID_REQ:1;
            __IO  uint32_t  HID_WRITE:1;
            __IO  uint32_t  HID_QBmode:1;
            __IO  uint32_t  RSVD01:3;
        };
        __IO  uint8_t  REQ;
    };
    __I   uint32_t  BTN_RPT_RAW:13;
    __I   uint32_t  RSVD02:2;
    __IO  uint32_t  MS_SWRST:1;
    /* 0x04 ~ 0x07 */
    __IO  uint32_t  R_STABLE_THL:6;
    __IO  uint32_t  RSVD03:2;
    __IO  uint32_t  R_STABLE_THS:4;
    __IO  uint32_t  R_BTN_ACC_MAX:4;
    __IO  uint32_t  R_STABLE_TH4MOT:5;
    __IO  uint32_t  R_ZBTN_DIV:2;
    __IO  uint32_t  RSVD04:1;
    __IO  uint32_t  R_TW_RPTIME:8;
    /* 0x08 ~ 0x0B */
    __IO  uint32_t  R_SPI_BURST_CNT:3;
    __IO  uint32_t  R_SPI_BURST_MODE:1;
    __IO  uint32_t  RSVD05:4;
    __IO  uint32_t  R_SPI_SPEED:3;
    __IO  uint32_t  SPI_RESYNC:1;
    __IO  uint32_t  RSVD06:4;
    __IO  uint32_t  R_CPOL:1;
    __IO  uint32_t  R_CPHA:1;
    __IO  uint32_t  R_CS_TEST:1;
    __IO  uint32_t  RSVD07:5;
    __I   uint32_t  HID_EOT:1;
    __IO  uint32_t  RSVD08:7;
    /* 0x0C ~ 0x13 */
    __IO  uint8_t   R_MOUSE_ADDR[7];
    __IO  uint8_t   R_SPI_A2D_TH;
    /* 0x14 ~ 0x1B */
    __IO  uint8_t   R_MOUSE_WDATA[7];
    __IO  uint8_t   R_SPI_CS2SCK_TH;
    /* 0x1C ~ 0x23 */
    __I   uint8_t   HID_RDATA[7];
    __IO  uint8_t   R_SPI_SCK2CS_TH;
    /* 0x24 ~ 0x27 */
    __I   uint32_t  BTN_RPT:5; // B5,B4,BM,BR,BL
    __I   uint32_t  RSVD10:3;
    __I   uint32_t  DX:12;
    __I   uint32_t  DY:12;
    /* 0x28 ~ 0x2B */
    __I   uint32_t  ZBTN_RPT:5;
    __I   uint32_t  RSVD11:3;
    __I   uint32_t  TW2:1;
    __I   uint32_t  TW1:1;
    __I   uint32_t  RSVD12:22;
    /* 0x2C ~ 0x2F */
    __I   uint32_t  RSVD13;
    /* 0x30 ~ 0x33 */
    __IO  uint32_t  B_KEYSCAN_ENH:1;
    __IO  uint32_t  B_KB_SCAN_LVL:1;
    __I   uint32_t  KEYSCAN_DONE:1;
    __I   uint32_t  KEYSCAN_UPDATE:1;
    __I   uint32_t  RSVD14:4;
    __IO  uint32_t  B_KEYSCAN_ACTIVE_PERIOD:8;
    __IO  uint32_t  B_KEYSCAN_TRANS_PERIOD:8;
    __IO  uint32_t  B_KEYSCAN_SCANNUM:5;
    __I   uint32_t  RSVD15:2;
    __IO  uint32_t  KB_SWRST:1;
    /* 0x34 ~ 0x47 */
    __I   uint8_t   KEYSCAN_DATAI[20];
    /* 0x48 ~ 0x4F */
    __I   uint32_t  RSVD16[2];
    /* 0x50 ~ 0x53 */
    __IO  uint32_t  DEBUG_SW:2;
    __I   uint32_t  RSVD17:30;
} HID_CTRL_TYPE;

/* ============================ PWMLED =========================== */
typedef struct {
    /* 0x00 ~ 0x03 */
    __IO  uint8_t   T1;
    __IO  uint8_t   T2;
    __IO  uint16_t  T3;
    /* 0x04 ~ 0x07 */
    __IO  uint32_t  N1:7;
    __IO  uint32_t  RP_N1:1;
    __IO  uint32_t  N2:7;
    __IO  uint32_t  RP_N2:1;
    __I   uint32_t  RSVD00:16;
    /* 0x08 ~ 0x0B */
    __IO  uint32_t  POL:1;
    __I   uint32_t  RSVD01:3;
    __IO  uint32_t  MODE:2;
    __IO  uint32_t  LED_EN:1;
    __IO  uint32_t  LED_RESETB:1;
    __I   uint32_t  RSVD02:24;
    /* 0x0C ~ 0x0F */
    __IO  uint32_t  T4:16;
    __IO  uint8_t   BR_SP;
    __I   uint32_t  RSVD03:8;
    /* 0x10 ~ 0x13 */
    __IO  uint32_t  BR_TH_MAX:16;
    __IO  uint32_t  BR_TH_MIN:16;
    /* 0x14 ~ 0x17 */
    __IO  uint32_t  PWM_M:16;
    __IO  uint32_t  PWM_N:16;
} PWM_LED_CTRL_TYPE;

/* ============================ RTC =========================== */
typedef struct {
    /* 0x00 ~ 0x03 */
    __IO  uint32_t  TASK_START:1;
    __I   uint32_t  RSVD00:31;
    /* 0x04 ~ 0x07 */
    __IO  uint32_t  TASK_STOP:1;
    __I   uint32_t  RSVD01:31;
    /* 0x08 ~ 0x0B */
    __IO  uint32_t  TASK_CLEAR:1;
    __I   uint32_t  RSVD02:31;
    /* 0x0C ~ 0x0F */
    __IO  uint32_t  INTERRUPT_COMPARE0:1;
    __IO  uint32_t  INTERRUPT_COMPARE1:1;
    __IO  uint32_t  INTERRUPT_TICK:1;
    __I   uint32_t  RSVD03:29;
    /* 0x10 ~ 0x13 */
    __IO  uint32_t  COMPARE0_INTERRUPT_ENABLE:1;
    __IO  uint32_t  COMPARE1_INTERRUPT_ENABLE:1;
    __IO  uint32_t  TICK_INTERRUPT_ENABLE:1;
    __I   uint32_t  RSVD04:29;
    /* 0x14 ~ 0x17 */
    __IO  uint32_t  COMPARE0_INTERRUPT_DISABLE:1;
    __IO  uint32_t  COMPARE1_INTERRUPT_DISABLE:1;
    __IO  uint32_t  TICK_INTERRUPT_DISABLE:1;
    __I   uint32_t  RSVD05:29;
    /* 0x18 ~ 0x1B */
    __IO  uint32_t  COUNTER:24;
    __I   uint32_t  RSVD06:8;
    /* 0x1C ~ 0x1F */
    __IO  uint32_t  PRESCALER:13;
    __I   uint32_t  RSVD07:19;
    /* 0x20 ~ 0x23 */
    __IO  uint32_t  COMPARE0_SECONDS_0_9:4;
    __IO  uint32_t  COMPARE0_SECONDS_0_5:4;
    __IO  uint32_t  COMPARE0_MINUTE_0_9:4;
    __IO  uint32_t  COMPARE0_MINUTE_0_5:4;
    __IO  uint32_t  COMPARE0_HOURS_0_9:4;
    __IO  uint32_t  COMPARE0_HOURS_0_5:4;
    __I   uint32_t  RSVD08:8;
    /* 0x24 ~ 0x27 */
    __IO  uint32_t  COMPARE1_SECONDS_0_9:4;
    __IO  uint32_t  COMPARE1_SECONDS_0_5:4;
    __IO  uint32_t  COMPARE1_MINUTE_0_9:4;
    __IO  uint32_t  COMPARE1_MINUTE_0_5:4;
    __IO  uint32_t  COMPARE1_HOURS_0_9:4;
    __IO  uint32_t  COMPARE1_HOURS_0_5:4;
    __I   uint32_t  RSVD09:8;
    /* 0x28 ~ 0x2B */
    __IO  uint32_t  SECONDS_BIT:4;
    __I   uint32_t  RSVD0A:28;
    /* 0x2C ~ 0x2F */
    __IO  uint32_t  CALENDAR_SECONDS_0_9:4;
    __IO  uint32_t  CALENDAR_SECONDS_0_5:4;
    __IO  uint32_t  CALENDAR_MINUTE_0_9:4;
    __IO  uint32_t  CALENDAR_MINUTE_0_5:4;
    __IO  uint32_t  CALENDAR_HOURS_0_9:4;
    __IO  uint32_t  CALENDAR_HOURS_0_5:4;
    __I   uint32_t  RSVD0B:8;
} RTC_CTRL_TYPE;

typedef struct {
    __IO  uint32_t  TASK_START;
    __IO  uint32_t  TASK_STOP;
    __IO  uint32_t  TASK_CLEAR;
    __IO  uint32_t  EVENTS;
    __IO  uint32_t  INTENSET;
    __IO  uint32_t  INTENCLR;
    __IO  uint32_t  COUNTER;
    __IO  uint32_t  PRESCALER;
    __IO  uint32_t  COMPARE0;
    __IO  uint32_t  COMPARE1;
    __IO  uint32_t  SECONDS_BIT;
    __IO  uint32_t  CALENDAR;
} RTC_CTRL_TYPE2;

/* ============================ TRNG control =========================== */
typedef struct {
    /* 0x00 ~ 0x03 */
    __IO  uint32_t  TASK_TRNG_START:1;
    __IO  uint32_t  RSVD00:31;
    /* 0x04 ~ 0x07*/
    __IO  uint32_t  TASK_PRNG_START:1;
    __IO  uint32_t  RSVD01:31;
    /* 0x08 ~ 0x0B*/
    __IO  uint32_t  TRNG_DATA_VALUE:8;
    __IO  uint32_t  RSVD02:24;
    /* 0x0C ~ 0x0F*/
    __IO  uint32_t  PRNG_DATA_VALUE:8;
    __IO  uint32_t  RSVD03:24;
    /* 0x10 ~ 0x13 */
    __IO  uint32_t  TRNG_DONE_INTERRUPT:1;
    __IO  uint32_t  RSVD04:31;
    /* 0x14 ~ 0x17*/
    __IO  uint32_t  ENABLE_INTERRUPT:1;
    __IO  uint32_t  RSVD05:31;
    /* 0x18 ~ 0x1B*/
    __IO  uint32_t  DISABLE_INTERRUPT:1;
    __IO  uint32_t  RSVD06:31;
    /* 0x1C ~ 0x1F*/
    __IO  uint32_t  TAP_ARRAY:8;
    __IO  uint32_t  RSVD07:24;
    /* 0x20 ~ 0x23*/
    __IO  uint32_t  INIT_VALUE:8;
    __IO  uint32_t  RSVD08:24;
    /* 0x24 ~ 0x27*/
    __IO  uint32_t  SEED_UPDATE_N:4;
    __IO  uint32_t  RSVD09:28;
    /* 0x28 ~ 0x2B*/
    __IO  uint32_t  TAP_UPDATE_N:4;
    __IO  uint32_t  RSVD0A:28;
    /* 0x2C ~ 0x2F*/
    __IO  uint32_t  T_OR_P:1;
    __IO  uint32_t  RSVD0B:31;
}TRNG_CTRL_TYPE;

/* ============================  PWM control  =========================== */
#define    PWM_SEQ_NUM    2

typedef struct {
    __IO  uint32_t  SEQ_PTR;
    __IO  uint32_t  SEQ_CNT;
    __IO  uint32_t  RSVD00;
} PWM_SEQ_SETTING_TYPE;

typedef struct {
    /* 0x00 ~ 0x03 */
    __IO  uint32_t  TASK_STOP; /* write 1 to stop pwm wave generation, auto clear */
    /* 0x04 ~ 0x0F */
    __IO  uint32_t  TASKS_SEQSTART[PWM_SEQ_NUM]; /* wirte 1 to start seq pwm waveform, auto clear */
    __IO  uint32_t  RSVD00;
    /* 0x10 ~ 0x13 */
    __IO  uint32_t  INT_STATUS;
    /* 0x14 ~ 0x17 */
    __IO  uint32_t  INTENSET;
    /* 0x18 ~ 0x1B */
    __IO  uint32_t  INTENCLR;
    /* 0x1C ~ 0x1F */
    __IO  uint32_t  MODE;
    /* 0x20 ~ 0x23 */
    __IO  uint32_t  COUNTERTOP;
    /* 0x24 ~ 0x27 */
    __IO  uint32_t  PRESCALER;
    /* 0x28 ~ 0x2B */
    __IO  uint32_t  DECODER;
    /* 0x2C ~ 0x2F */
    __IO  uint32_t  LOOP_CNT;
    /* 0x30 ~ 0x47 */
    __IO  PWM_SEQ_SETTING_TYPE  PWM_SEQ[PWM_SEQ_NUM];
    /* 0x48 ~ 0x4B */
    __IO  uint32_t  SEQ_MODE;
} PWM1_CTRL_TYPE;

/* ============================  PDM control  =========================== */
typedef struct {
    /* 0x00 ~ 0x03 */
    __IO  uint32_t  R_PDM_EN_RD_ONLY:1;
    __I   uint32_t  RSVD00:7;
    __IO  uint32_t  R_PDM_SAMPLE_RATE:1;
    __IO  uint32_t  R_PDM_SAMPLE_WIDTH:1;
    __IO  uint32_t  R_PDM_MODE_LR:1;
    __IO  uint32_t  R_PDM_MODE_CLK:1;
    __IO  uint32_t  R_PDMCLKCTRL:2;
    __I   uint32_t  RSVD01:2;
    union {
        __IO  uint8_t   INT_MASK;
        struct {
            __I   uint32_t  RSVD02:5;
            __IO  uint32_t  R_AMICI2S_INT_MASK:1;
            __IO  uint32_t  R_PDM_INT_MASK:1;
            __IO  uint32_t  R_PDM_HALF_INT_MASK:1;
        };
    };
    union {
        __IO  uint8_t   INT_STATUS;
        struct {
            __I   uint32_t  RSVD03:5;
            __IO  uint32_t  R_AMICI2S_INT:1;
            __IO  uint32_t  R_PDM_HALF_INT:1;
            __IO  uint32_t  R_PDM_INT:1;
        };
    };
    /* 0x04 ~ 0x07 */
    __IO  uint32_t  R_PDM_GAINR:8;
    __IO  uint32_t  R_PDM_GAINL:8;
    __I   uint32_t  RSVD04:16;
    /* 0x08 ~ 0x0B */
    __IO  uint32_t  R_PDM_PTR:18;
    __I   uint32_t  RSVD05:14;
    /* 0x0C ~ 0x0F */
    __IO  uint32_t  R_PDM_MAXCNT:17;
    __IO  uint32_t  R_DC_PERIOD:4;
    __IO  uint32_t  R_DC_MODE:1;
    __IO  uint32_t  R_DC_ONOFF:1;
    __I   uint32_t  RSVD06:9;
    /* 0x10 ~ 0x13 */
    __IO  uint32_t  R_PDM_EN:1;
    __I   uint32_t  RSVD07:31;
    /* 0x14 ~ 0x17 */
    __I   uint32_t  CURRENT_RX_PTR:18;
    __I   uint32_t  RSVD08:14;
    /* 0x18 ~ 0x4F */
    __I   uint32_t  RSVD09[14];
    /* 0x50 ~ 0x53 */
    __IO  uint32_t  R_AMIC_INIT_CNTVAL:20;
    __I   uint32_t  RSVD0A:9;
    __IO  uint32_t  R_AMIC_FPGA_TEST_MODE:1;
    __IO  uint32_t  R_AMIC_ADC_BIST_EN:1;
    __IO  uint32_t  R_AMIC_MODE_EN:1;
    /* 0x54 ~ 0x57 */
    __IO  uint32_t  R_AMIC_PGA_GAIN:5;
    __I   uint32_t  RSVD0B:3;
    __IO  uint32_t  R_AMIC_TEST:4;
    __I   uint32_t  RSVD0C:19;
    __IO  uint32_t  R_AMIC48K_EN:1;
    /* 0x58 ~ 0x5B */
    __I   uint32_t  AD_AMIC_DOUT:12;
    __I   uint32_t  AD_AMIC_DOUT_OVF:1;
    __I   uint32_t  AD_AMIC_CKOUT:1;
    __I   uint32_t  RSVD0D:18;
    /* 0x5C ~ 0x5F */
    __I   uint32_t  RSVD0E;
    /* 0x60 ~ 0x63 */
    __I   uint32_t  AMIC_DATA:16;
    __I   uint32_t  SINE_PAT:16;
} PDM_CTRL_TYPE;

/* ============================  I2S control  =========================== */
typedef struct {
    /* 0x00 ~ 0x03 */
    __IO  uint32_t  MFREQ:4;
    __IO  uint32_t  SRATIO:4;
    __IO  uint32_t  SWIDTH:2;
    __IO  uint32_t  CHANNEL:2;
    __IO  uint32_t  LRSWAP:1;
    __IO  uint32_t  FORMAT:1;
    __IO  uint32_t  LSB_T:1;
    __IO  uint32_t  LSB_R:1;
    __I   uint32_t  RSVD00:16;
    /* 0x04 ~ 0x07 */
    __IO  uint32_t  I2S_TXEN:1;
    __I   uint32_t  RSVD01:31;
    /* 0x08 ~ 0x0B */
    __IO  uint32_t  I2S_RXEN:1;
    __I   uint32_t  RSVD02:31;
    /* 0x0C ~ 0x0F */
    __IO  uint32_t  DMA_TX_PTR:16;
    __IO  uint32_t  DMA_RX_PTR:16;
    /* 0x10 ~ 0x13 */
    __IO  uint32_t  DMA_TX_MAXLEN:16;
    __IO  uint32_t  DMA_RX_MAXLEN:16;
    /* 0x14 ~ 0x17 */
    union {
        __IO  uint32_t  INT_ST;
        struct {
            __IO  uint32_t  I2S_T_DMA_HALF_DONE_INT:1;
            __IO  uint32_t  I2S_R_DMA_HALF_DONE_INT:1;
            __I   uint32_t  I2S_BUSY:1;
            __I   uint32_t  RSVD03:1;
            __IO  uint32_t  I2S_T_DMA_HALF_DONE_INT_MASK:1;
            __IO  uint32_t  I2S_R_DMA_HALF_DONE_INT_MASK:1;
            __I   uint32_t  RSVD04:2;
            __IO  uint32_t  I2S_T_DMA_DONE_INT:1;
            __IO  uint32_t  I2S_R_DMA_DONE_INT:1;
            __I   uint32_t  RSVD05:2;
            __IO  uint32_t  I2S_T_DMA_DONE_INT_MASK:1;
            __IO  uint32_t  I2S_R_DMA_DONE_INT_MASK:1;
            __I   uint32_t  RSVD06:18;
        };
    };
} I2S_CTRL_TYPE;

/* ============================  GPADC control  =========================== */
typedef struct {
    /* 0x00 ~ 0x03 */
    __IO  uint32_t  TASK_STOP;
    /* 0x04 ~ 0x07 */
    __IO  uint32_t  TASK_START;
    /* 0x08 ~ 0x0B */
    __IO  uint32_t  CHANNEL_SEL;
    /* 0x0C ~ 0x0F */
    __IO  uint32_t  DATA_PTR;
    /* 0x10 ~ 0x13 */
    union {
        __IO  uint32_t  INT_ST;
        struct {
            __IO  uint32_t  TASK_DONE_INT_ST:1;
            __IO  uint32_t  HALF_MEM_DONE_INT_ST:1;
            __IO  uint32_t  FULL_MEM_DONE_INT_ST:1;
            __I   uint32_t  RSVD10:29;
        };
    };
    /* 0x14 ~ 0x17 */
    __IO  uint32_t  CONTINUE_SCAN;
    /* 0x18 ~ 0x1B */
    __I   uint32_t  RSVD18;
    /* 0x1C ~ 0x1F */
    union {
        __IO  uint32_t  INT_EN_SET;
        struct {
            __IO  uint32_t  TASK_DONE_INT_EN_SET:1;
            __IO  uint32_t  HALF_MEM_DONE_INT_EN_SET:1;
            __IO  uint32_t  FULL_MEM_DONE_INT_EN_SET:1;
            __I   uint32_t  RSVD1C:29;
        };
    };
    /* 0x20 ~ 0x23 */
    union {
        __IO  uint32_t  INT_EN_CLR;
        struct {
            __IO  uint32_t  TASK_DONE_INT_EN_CLR:1;
            __IO  uint32_t  HALF_MEM_DONE_INT_EN_CLR:1;
            __IO  uint32_t  FULL_MEM_DONE_INT_EN_CLR:1;
            __I   uint32_t  RSVD20:29;
        };
    };
    /* 0x24 ~ 0x27 */
    /* number of channel switch */
    __IO  uint32_t  SCAN_COUNT;
    /* 0x28 ~ 0x2B */
    /* sample number for channel is equal to DATA_LENGTH + 1 */
    __IO  uint32_t  DATA_LENGTH;
    /* 0x2C ~ 0x3B */
    __I   uint32_t  RSVD2C[4];
    /* 0x3C ~ 0x3F */
    __IO  uint32_t  START_SETTLE;
    /* 0x40 ~ 0x43 */
    __IO  uint32_t  CHANNEL_SETTLE;
    /* 0x44 ~ 0x47 */
    /* 1 - no average
       3 - sum of 2 samples
       5 - sum of 4 samples
       7 - sum of 8 samples */
    __IO  uint32_t  AVERAGE;
    /* 0x48 ~ 0x4B */
    /* number of channels configure in SET */
    __IO  uint32_t  CHANNEL_SET_NUM;
    /* 0x4C ~ 0x4F */
    __IO  uint32_t  DA_ADCGP_EN;
    /* 0x50 ~ 0x53 */
    __IO  uint32_t  DA_ADCGP_TRIM_EN;
    /* 0x54 ~ 0x57 */
    __IO  uint32_t  DA_ADCGP_BIST_EN;
    /* 0x58 ~ 0x5B */
    __IO  uint32_t  DA_ADCGP_BIST_SHORT_EN;
    /* 0x5C ~ 0x5F */
    /* Sample rate = FCLK / ((CLKRATE + 1) * 2) */
    __IO  uint32_t  CLKRATE;
    /* 0x60 ~ 0x67 */
    union {
        __IO  uint32_t  CHANEL_SET[2];
        struct {
            __IO  uint32_t  CH_CANDIDATE0:4;
            __IO  uint32_t  CH_CANDIDATE1:4;
            __IO  uint32_t  CH_CANDIDATE2:4;
            __IO  uint32_t  CH_CANDIDATE3:4;
            __IO  uint32_t  CH_CANDIDATE4:4;
            __IO  uint32_t  CH_CANDIDATE5:4;
            __IO  uint32_t  CH_CANDIDATE6:4;
            __IO  uint32_t  CH_CANDIDATE7:4;
            __IO  uint32_t  CH_CANDIDATE8:4;
            __IO  uint32_t  CH_CANDIDATE9:4;
            __IO  uint32_t  CH_CANDIDATEA:4;
            __I   uint32_t  RSVD60:20;
        };
    };
    /* 0x68 ~ 0x6B */
    /* unit(Bytes / 4) */
    __IO  uint32_t  GPADC_MEM_MAXCNT;
    /* 0x6C ~ 0x6F */
    __I   uint32_t  ADC_DATA_HCLK;
    /* 0x70 ~ 0x73 */
    __I   uint32_t  GPADC_FSM_CS;
    /* 0x74 ~ 0x77 */
    __IO  uint32_t  BIST_SHORT_HIGH_SETTLE;
    /* 0x78 ~ 0x7B */
    __IO  uint32_t  BIST_SHORT_LOW_SETTLE;
    /* 0x7C ~ 0x7F */
    __IO  uint32_t  DEBUG_O_SEL;
    /* 0x80 ~ 0x83 */
    __IO  uint32_t  ADC_DATA_SUM2DMA_HCLK;
    /* 0x84 ~ 0xB3 */
    /* BIST related, skip */
} GPADC_CTRL_TYPE;

#pragma pack(push, 4)
/* ============================ SC Reader Controler =========================== */
typedef struct {	
	/* CTRL : 0x00~ 0x03*/
	__IO  uint32_t	MODE:1;                 /* 7816 = 0, UART = 1 */
    __IO  uint32_t	PARITY_CHECK_EN:1;      /* No parity = 0, One parity = 1 */
    __IO  uint32_t	PARITY_CHECK_MODE:1;    /* Even = 0, Odd = 1 */
    __IO  uint32_t	PROTOCOL_SEL:1;         /* T0 = 0, T1 = 1 */
    __IO  uint32_t	STOP_LEN:1;             /* 0 = 1bit, 1 = 2bit */
    __IO  uint32_t	RX_RETRY_CNT:3;
    __IO  uint32_t	TX_RETRY_CNT:3;
    __IO  uint32_t	CLK_STOP_VAL:1;
    __IO  uint32_t	CLK_STOP_EN:1;
    __IO  uint32_t	DETECT_LVL:1;           /* detect low = 0, hight = 1 */
    __IO  uint32_t	AUTO_ACTIVATION_EN:1;
    __IO  uint32_t	EXTRA_GUARD_TIME:8;     /* Extra guard time between TXs, also affect RX decode */
    __IO  uint32_t	GUARD_TIME:8;           /* this will only affect delay between first TX & last RX */
	__IO  uint32_t	ACTIVE_VCC_LEVEL:1;	    /* Setup VCC pin level when activated */
	/* CTRL2 : 0x04~ 0x07*/
    __IO  uint32_t	RX_FIFO_THRESHOLD:4;
    __IO  uint32_t	TX_FIFO_THRESHOLD:4;
    __IO  uint32_t	RX_TIMEOUT_DELAY:16;
	__IO  uint32_t	RX_TIMEOUT_DETECT_EN:1;
	__IO  uint32_t	RSVD4:7;	
	/* DIV : 0x08~ 0x0B*/
	__IO  uint32_t	SC_CLK_DIV:16;          /* Output clock divider */
	__IO  uint32_t	BAUD_CLK_DIV:16;        /* F/D */
	/* ENA : 0x0C~ 0x0F*/
	__IO  uint32_t	RX_ENABLE:1;
    __IO  uint32_t	TX_ENABLE:1;
    __IO  uint32_t	RX_RETRY_ENABLE:1;
    __IO  uint32_t	TX_RETRY_ENABLE:1;
    __IO  uint32_t  TX_FORCE_PARITY_ERROR:1;
    __IO  uint32_t  RX_FORCE_PARITY_ERROR:1;
    __IO  uint32_t	RSVDC:26;
	/* ACTION : 0x10~ 0x13*/
	__IO  uint32_t	TRIGGER_ACTIVATE:1;
    __IO  uint32_t	TRIGGER_DEACTIVATE:1;
    __IO  uint32_t	TRIGGER_WARM_RESET:1;
	__IO  uint32_t	RSVD10:29;
	/* ACCTIME : 0x14~ 0x17*/
	__IO  uint32_t	ACTIVATE_DURATION:16;
	__IO  uint32_t	RSVD14:16;	
	/* RSTTIME : 0x18~ 0x1B*/
    __IO  uint32_t	RESET_DURATION:16;
	__IO  uint32_t	RSVD18:16;
	/* ATRWAIT : 0x1C~ 0x1F*/
	__IO  uint32_t	ATR_WAIT_DURATION:16;
	__IO  uint32_t	RSVD1C:16;
	/* WAITTIME : 0x20~ 0x23*/
	__IO  uint32_t	WAITING_TIME:16;
	__IO  uint32_t	RSVD20:16;
	/* INTEN : 0x24~ 0x27*/
    __IO  uint32_t	INTS_EN:18;
	__IO  uint32_t	RSVD24:14;
    /* INTSTAT : 0x28~ 0x2B*/
    __IO  uint32_t	INTS_STATE:18;
	__IO  uint32_t	RSVD28:14;
    /* CARDSTAT : 0x2C~ 0x2F*/
    __IO  uint32_t	INVERSE_CONVENTION:1;
    __IO  uint32_t	RX_IN_ACTIVE:1;
    __IO  uint32_t	TX_IN_ACTIVE:1;
	__IO  uint32_t	RSVD2C:29;
    /* FIFOSTAT : 0x30~ 0x33*/
    __IO  uint32_t	TX_FIFO_EMPTY:1;
    __IO  uint32_t	TX_FIFO_FULL:1;
    __IO  uint32_t	TX_FIFO_OVERFLOW:1;
    __IO  uint32_t	TX_FIFO_COUNT:4;
    __IO  uint32_t	RX_FIFO_EMPTY:1;
    __IO  uint32_t	RX_FIFO_FULL:1;
    __IO  uint32_t	RX_FIFO_OVERFLOW:1;
    __IO  uint32_t	RX_FIFO_COUNT:4;
	__IO  uint32_t	RSVD30:18;
    /* DATA : 0x34~ 0x37*/
    __IO  uint32_t	DATA:8;
    __IO  uint32_t	DATA_PARITY_CHECK_ERROR:1;
	__IO  uint32_t	RSVD34:23;
    /* FSM : 0x38~ 0x3B*/
    __IO  uint32_t	SC_STATE:3;
    __IO  uint32_t	ACT_STATE:3;
    __IO  uint32_t	ATR_STATE:3;
    __IO  uint32_t	ATR_SEQ_STATE:3;
	__IO  uint32_t	RSVD38:20;
} SC_READER_CTRL_TYPE;
#pragma pack(pop)

#pragma pack()
/* ============================ IR =========================== */
typedef struct {
	/* 0x0000 */
	__IO  uint32_t	CARRIER_PERIOD:11;	//"Defines the carrier period in ir_clk cycles" 
	__IO  uint32_t	CARRIER_INVTIME:11; //"Invert output level at invtime"
	__IO  uint32_t	CARRIER_PWMFMT:1;	//"0:from high to low 1: from low to high"	
	__IO  uint32_t	RSVD00:9;		
	
	/* 0x0004 */
	__IO  uint32_t	LOGICBIT_ONE_FH:8;	//"Defines the first half duration of logic one in carrier clock cycles. Must be >0"	
	__IO  uint32_t	LOGICBIT_ONE_LH:8;	//"Defines the last half duration of logic one in carrier clock cycles. Must be >0"
	__IO  uint32_t	LOGICBIT_ZERO_FH:8; //"Defines the first half duration of logic zero in carrier clock cycles. Must be >0 ""
	__IO  uint32_t	LOGICBIT_ZERO_LH:8; 	//"Defines the last half duration of logic zero in carrier clock"	
	/* 0x0008 */
	__IO  uint32_t	CTRL_IR_EN	:1; 		//"enable ir transmition"
	__IO  uint32_t	CTRL_REPEAT_EN 	:1	;	//"enable repeat code transmition"
	__IO  uint32_t	CTRL_MODE :1	;			//"0:IR mode1:PWM mode"
	__IO  uint32_t	CTRL_IR_INV_OUT:1 ; 	//"0:not invert output level 1:invert output level"
	__IO  uint32_t	CTRL_ONE_FMT:1 ;			//"0:mark first 1:space first"
	__IO  uint32_t	CTRL_ZERO_FMT:1 ;		//"0:mark first 1:space first"
	__IO  uint32_t	CTRL_IRIE:1 ;			//"0:disable interrupt when transmit done 1:enable interrupt when transmit done"
	__IO  uint32_t	CTRL_CMDDONEIE:1;		//"interrupt at each command transmit done 0:disable 1:enable"
	__IO  uint32_t	CTRL_REPEAT_TIME :13;	//"repeat interval,in carrier clock cycles"
	__IO  uint32_t	RSVD01:11;	
	/* 0x000C */
	__IO  uint32_t	FIFOCLR_CCMD_FIFOCLR	:1; 		//"clear common comand fifo"
	__IO  uint32_t	FIFOCLR_RCMD_FIFOCLR	:1; 		//"clear repeat comand fifo"
	__IO  uint32_t	RSVD02:30;
	/* 0x0010*/
	__IO  uint32_t	STATUS_CCMD_FIFO_CNT	:4	;		//"count in common command fifo"
	__IO  uint32_t	STATUS_RCMD_FIFO_CNT	:4	;		//"count in repeat command fifo"
	__IO  uint32_t	STATUS_IR_BUSY	:1	;			//"ir busy"
	__IO  uint32_t	RSVD03:23;
	/* 0x0014*/
	__IO  uint32_t	INTSTATE_TRANS_DONE:1;			//"interrupt when each cmd transmit done"
	__IO  uint32_t	INTSTATE_CMD_DONE:1;				//"interrupt when ir transmit done"
	__IO  uint32_t	RSVD04:30;		
	/* 0x0018*/
	__IO  uint32_t	DR;
	/* 0x001C*/
	__IO  uint32_t	REPCMD;


}IR_CTRL_TYPE;
/* --------------------  End of section using anonymous unions  ------------------- */
#if defined(__CC_ARM)
  #pragma pop
#elif defined(__ICCARM__)
  /* leave anonymous unions enabled */
#elif defined(__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined(__TMS470__)
  /* anonymous unions are enabled by default */
#elif defined(__TASKING__)
  #pragma warning restore
#elif defined ( __CSMC__ )		/* Cosmic */
/* anonymous unions are enabled by default */
#else
  #warning Not supported compiler type
#endif

/* ================================================================================ */
/* ==============================             Memory Base             ============================== */
/* ================================================================================ */
#define ROM_BASE				0x00000000UL
#define PRAM_BASE				0x00100000UL
#define CACHE_BASE				0x10000000UL
#define DRAM_BASE				0x20000000UL
#define SYSTEMROM_BASE			0xf0000000UL
/* ================================================================================ */
/* ==========================              Peripheral memory map             ========================== */
/* ================================================================================ */
#define GPIO_CTRL_BASE			0x50000000UL
#define GPIO32_CTRL_BASE        (GPIO_CTRL_BASE + 0x80UL)
#define SYS_CTRL_BASE			0x50001000UL
#define SW_INT_CTRL_BASE		(SYS_CTRL_BASE + 0x08UL)
#define EFUSE0_CTRL_BASE        (SYS_CTRL_BASE + 0x10UL)
#define EFUSE1_CTRL_BASE        (SYS_CTRL_BASE + 0x20UL)
#define PAD_CTRL_BASE				(SYS_CTRL_BASE + 0x40UL)
#define PAD_SEL_BASE				(SYS_CTRL_BASE + 0x60UL)
#define PMU_CTRL_BASE				(SYS_CTRL_BASE + 0x70UL)
#define PAD2_SEL_BASE				(SYS_CTRL_BASE + 0x8CUL)
#define CACHE_CTRL_BASE		0x50004000UL
#define FLASH_CTRL_BASE		0x50005000UL
#define RTC_CTRL_BASE			0x50007000UL
#define WDT_CTRL_BASE			0x50008000UL
#define TIMER0_CTRL_BASE		0x5000A000UL
#define TIMER1_CTRL_BASE		0x5000B000UL
#define TIMER2_CTRL_BASE		0x5000C000UL
#define TIMER3_CTRL_BASE		0x5000D000UL
#define TIMER4_CTRL_BASE		0x5000E000UL
#define PWM_LED0_CTRL_BASE	0x5000F000UL
#define PWM_LED1_CTRL_BASE	0x5000F100UL
#define PWM_LED2_CTRL_BASE	0x5000F200UL
#define PWM_LED3_CTRL_BASE	0x5000F300UL
#define PWM_LED4_CTRL_BASE	0x5000F400UL
#define PWM_LED5_CTRL_BASE	0x5000F500UL
#define LLC_CTRL_BASE			0x50010000UL
#define MODEM_CTRL_BASE		0x50010100UL
#define RFSPI_CTRL_BASE			0x50010200UL
#define TRNG_CTRL_BASE			0x50020000UL
#define GPADC_CTRL_BASE		0x50021000UL
#define PWM1_CTRL_BASE			0x50022000UL
#define CAPDET_CTRL_BASE		0x50025000UL
#define PDM_CTRL_BASE			0x50026000UL
#define I2S_CTRL_BASE			0x50027000UL
#define ARBITER_CTRL_BASE		0x50028000UL
#define ECDH_CTRL_BASE			0x50029000UL
#define CMAC_CTRL_BASE			0x50029100UL
#define AAR_CTRL_BASE			0x50029200UL
#define M2M_CTRL_BASE			0x5002A000UL
#define SERIAL_IF_CTRL_BASE		0x5002B000UL
#define SPI_SLAVE_CTRL_BASE		(SERIAL_IF_CTRL_BASE + 0x10UL)
#define SPI_MASTER_CTRL_BASE		(SERIAL_IF_CTRL_BASE + 0x20UL)
#define I2C_SLAVE_CTRL_BASE		(SERIAL_IF_CTRL_BASE + 0x30UL)
#define I2C_MASTER0_CTRL_BASE		(SERIAL_IF_CTRL_BASE + 0x40UL)
#define I2C_MASTER1_CTRL_BASE		(SERIAL_IF_CTRL_BASE + 0x54UL)
#define UART0_CTRL_BASE			(SERIAL_IF_CTRL_BASE + 0x70UL)
#define UART1_CTRL_BASE			(SERIAL_IF_CTRL_BASE + 0x78UL)
#define HID_CTRL_BASE			0x5002C000UL
#define UART2_CTRL_BASE		0x5002D000UL
#define IR_CTRL_BASE			0x5002E000UL
#define SC_7816_CTRL_BASE		0x5002F000UL
#define BLEMASTERREG_CTRL_BASE	0x50100000UL
#define BLEMASTERCS_CTRL_BASE	0x50110000UL
#define MISC_BASE					0x40000000UL

#if 0
/* ================================================================================ */
/* ==========================              Peripheral memory map             ========================== */
/* ================================================================================ */
#define GPIO_CTRL_BASE			0x40000000UL
#define PMU_CTRL_BASE			0x40001070UL
#define SYS_CTRL_BASE			0x40001000UL
#define ECDH_CTRL_BASE			0x50001000UL
#define CMAC_CTRL_BASE			0x50001100UL
#define AAR_CTRL_BASE			0x50001200UL
#define DMA_M2M_CTRL_BASE		0x50002000UL
#define SERIAL_CTRL_BASE		0x50003000UL
#define UART_0_CTRL_BASE		0x50003008UL
#define UART_1_CTRL_BASE		0x5000300CUL
#define SPI_SLAVE_CTRL_BASE		0x50003010UL
#define SPI_MASTER_CTRL_BASE	0x50003020UL
#define I2C_SLAVE_CTRL_BASE		0x50003030UL
#define I2C_MASTER0_CTRL_BASE	0x50003040UL
#define I2C_MASTER1_CTRL_BASE	0x50003054UL
#define CACHE_CTRL_BASE			0x50004000UL
#define FLASH_CTRL_BASE			0x50005000UL
#define TIMER_CTRL_BASE			0x50006000UL
#define RTC_CTRL_BASE			0x50007000UL
#define WDT_CTRL_BASE			0x50008000UL
#define HID_CTRL_BASE			0x5000E000UL
#define PWM_CTRL_BASE			0x50022000UL
#define SW_INT_BASE				0x40001008UL
#define PAD_SEL_BASE			0x40001060UL
#define MEM_CONFIG_BASE			0x40001000UL


/* ================================================================================ */
/* ==============================             Memory map             ============================== */
/* ================================================================================ */
#define ROM_BASE				0x00000000
#define CACHE_BASE				0x10000000
#define RAM_BASE				0x20000000
#define MISC_BASE				0x40000000
#define INTF_BASE				0x40010000
#define LL_BASE	  				0x50010000
#define MDM_BASE				0x50010100
#define RF_BASE					0x50010200
#define PWRON_BASE				0x50001000
#define FW_CODE_0_BASE			0x10001000
#endif

#define PMU_CTRL  ((PMU_CTRL_TYPE *) PMU_CTRL_BASE)
#define LL_CTRL  ((LL_CTRL_TYPE *) LLC_CTRL_BASE)


#ifdef __cplusplus
}
#endif

#endif  /* ARMCM0_H */
