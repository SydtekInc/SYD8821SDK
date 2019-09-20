#ifndef _IR_H_
#define _IR_H_

#include "basic_types.h"
#include "config.h"
#include "stdbool.h"
#include "console.h"
#include "ARMCM0.h"

/* Macro Definition */
#define IR_IRQ_NUMBER			(9)
//#define SYSTEM_M_FREQUENCY	  (13)
#define SYSTEM_M_FREQUENCY		(15)
#define SYSTEM_MAIN_FREQUENCY		(15000000)

/* Bit Operation */

#define BIT_SET(data, bit)		((data) |= (bit))
#define BIT_RESET(data, bit)	((data) &= ~(bit))

#define  PPR250_FPGA_RC    30000000
#define  FCLK			   (PPR250_FPGA_RC / (1 << SYS_CTRL->FCLK_CONFIG))
#define  PCLK			   (FCLK / (1 << (SYS_CTRL->PCLK_CONFIG + 1)))
#define  SCLK			   32000000
#define SYS_CTRL	((SYS_CONFIG_TYPE *) SYS_CTRL_BASE)


#define IR_NEC_K_FREQUENCY		(38)		// 38KHz
#define IR_RC5_K_FREQUENCY		(36)		// 36KHz
#define IR_NEC_PERIOD			(26.315)	// 1/38KHz
#define IR_RC5_PERIOD			(27.777)	// 1/36KHz
#define IR_NEC_CARRIER_SPACE	(3)
#define IR_RC5_CARRIER_SPACE	(3)
#define IR_NEC_ONE_FH_TIME		(0.56)		// 560us
#define IR_RC5_ONE_FH_TIME		(0.889) 	// 889us
#define IR_NEC_ONE_SPACE		(3)
#define IR_RC5_ONE_SPACE		(1)

/* Register Definition */
#define PIR_BASE				(0x5002E000)



/* IR controller (PIR) */
#define IRPORT_CARRIER		   (*(volatile uint32_t *)(PIR_BASE + 0x00)) //Carrier control
#define IRPORT_LOGICBIT 	   (*(volatile uint32_t *)(PIR_BASE + 0x04)) //Logical bit
#define IRPORT_CTRL 		   (*(volatile uint32_t *)(PIR_BASE + 0x08)) //Format and interrupt control
#define IRPORT_FIFOCLR		   (*(volatile uint32_t *)(PIR_BASE + 0x0c)) //Fifo clear
#define IRPORT_STATUS		   (*(volatile uint32_t *)(PIR_BASE + 0x10)) //Status 
#define IRPORT_INTSTAT		   (*(volatile uint32_t *)(PIR_BASE + 0x14)) //Interrupt status
#define IRPORT_DR			   (*(volatile uint32_t *)(PIR_BASE + 0x18)) //DR: Data to transmit, write to fifo
#define IRPORT_REPCMD		   (*(volatile uint32_t *)(PIR_BASE + 0x1c)) //RCMD: Repeat command(data) to transmit, write to fifo

/* Bit Definition */
#define PERIOD_MASK 			(0x7ff) 		// [10:0]
#define INVTIME_MASK			(0x7ff << 11)	// [21:11]
#define PWMFMT_MASK 			(0x001 << 22)	// [22]

#define ONE_FH_MASK 			(0xff)			// [7:0]
#define ONE_LH_MASK 			(0xff << 8) 	// [15:8]
#define ZERO_FH_MASK			(0xff << 16)	// [23:16]
#define ZERO_LH_MASK			(0xff << 24)	// [31:24]

#define CTX_IREN				BIT(0)
#define CTX_REPEN				BIT(1)
#define CTX_MODE				BIT(2)
#define CTX_IRINVOUT			BIT(3)
#define CTX_ONEFMT				BIT(4)
#define CTX_ZEROFMT 			BIT(5)
#define CTX_IRIE				BIT(6)
#define CTX_CMDDONEIE			BIT(7)
#define CTX_REPTIME_MASK		(0x1fff << 8)	// [20:8]

#define CCMD_FIFO_CLR			BIT(0)
#define RCMD_FIFO_CLR			BIT(1)

#define CCMD_FIFO_CNT_MASK		(0xf)			// [3:0]
#define RCMD_FIFO_CNT_MASK		(0xf << 4)		// [7:4]
#define IRBUSY_MASK 			(0x1 << 8)		// [8]

#define INTSTAT_TRANSDONE		BIT(0)
#define INTSTAT_CMD_DONE		BIT(1)

// cmd type 0
#define DR0_BIT_SEQ_MASK		(0xffff)		// [15:0]
#define DR0_BIT_NUL_MASK		(0xf << 16) 	// [19:16]
#define DR0_CMD_TYPE_MASK		(0x1 << 25) 	// [25]

// cmd type 1
#define DR1_FH_TIME_MASK		(0xfff) 		// [11:0]
#define DR1_LH_TIME_MASK		(0xfff << 12)	// [23:12]
#define DR1_SPACE_FIRST_MASK	(0x1 << 24) 	// [24]
#define DR1_CMD_TYPE_MASK		(0x1 << 25) 	// [25]

/* IR Controller Related Macro Definition */
#define IR_CMDDONE_INT_DISABLE()	(BIT_RESET(IRPORT_CTRL, CTX_CMDDONEIE))
#define IR_CMDDONE_INT_ENABLE() 	(BIT_SET(IRPORT_CTRL, CTX_CMDDONEIE))
#define IR_INT_DISABLE()			(BIT_RESET(IRPORT_CTRL, CTX_IRIE))
#define IR_INT_ENABLE() 			(BIT_SET(IRPORT_CTRL, CTX_IRIE))
#define IR_ZEROFMT_MARK()			(BIT_RESET(IRPORT_CTRL, CTX_ZEROFMT))
#define IR_ZEROFMT_SPACE()			(BIT_SET(IRPORT_CTRL, CTX_ZEROFMT))
#define IR_ONEFMT_MARK()			(BIT_RESET(IRPORT_CTRL, CTX_ONEFMT))
#define IR_ONEFMT_SPACE()			(BIT_SET(IRPORT_CTRL, CTX_ONEFMT))
#define IR_IRINVOUT_DISABLE()		(BIT_RESET(IRPORT_CTRL, CTX_IRINVOUT))
#define IR_IRINVOUT_ENABLE()		(BIT_SET(IRPORT_CTRL, CTX_IRINVOUT))
#define IR_SET_MODE_IR()			(BIT_RESET(IRPORT_CTRL, CTX_MODE))
#define IR_SET_MODE_PWM()			(BIT_SET(IRPORT_CTRL, CTX_MODE))
#define IR_REP_TX_ENABLE()			(BIT_SET(IRPORT_CTRL, CTX_REPEN))
#define IR_REP_TX_DISABLE() 		(BIT_RESET(IRPORT_CTRL, CTX_REPEN))
#define IR_TX_ENABLE()				(BIT_SET(IRPORT_CTRL, CTX_IREN))
#define IR_TX_DISABLE() 			(BIT_RESET(IRPORT_CTRL, CTX_IREN))

#define IR_CCMD_FIFO_CLR()			(BIT_SET(IRPORT_FIFOCLR, CCMD_FIFO_CLR))
#define IR_RCMD_FIFO_CLR()			(BIT_SET(IRPORT_FIFOCLR, RCMD_FIFO_CLR))

#define IR_TRANSDONE_INT_CLR()		(BIT_SET(IRPORT_INTSTAT, INTSTAT_TRANSDONE))
#define IR_CMD_DONE_INT_CLR()		(BIT_SET(IRPORT_INTSTAT, INTSTAT_CMD_DONE))
#define SET_IR_CARRIER(period,invtime,pwmfmt)  do { \
	IRPORT_CARRIER = ( \
		(period & PERIOD_MASK) | \
		((invtime << 11) & INVTIME_MASK) | \
		((pwmfmt << 22) & PWMFMT_MASK) \
		); \
	} while (0)

#define SET_IR_LOGICBIT(one_fh,one_lh,zero_fh,zero_lh)	do { \
	IRPORT_LOGICBIT = ( \
		(one_fh & ONE_FH_MASK) | \
		((one_lh << 8) & ONE_LH_MASK) | \
		((zero_fh << 16) & ZERO_FH_MASK) | \
		((zero_lh << 24) & ZERO_LH_MASK) \
		); \
	} while (0)

#define SET_IR_REPTIME(reptime)  do { \
	IRPORT_CTRL = (IRPORT_CTRL & (uint32_t)(~CTX_REPTIME_MASK & 0xFFFFFFFF)) \
	| ((reptime << 8) & CTX_REPTIME_MASK); \
	} while (0)


/* NEC  Declaration */
extern void ir_nec_init(uint16_t carrier_hz);
extern void ir_nec_tx_data(uint16_t custom_code_t, uint16_t data_code_t);
extern void ir_nec_tx_reverse_data(uint8_t custom_code_t, uint8_t data_code_t);
extern void ir_nec_tx_repeat(uint16_t custom_code_t, uint16_t data_code_t);
/* RC5	Declaration */
extern void ir_rc5_init(uint16_t carrier_kHz);
extern void ir_rc5_test_hard(void);

extern void ir_tx(uint32_t dr);
extern void ir_stop_tx(void);
extern uint32_t ir_rc5_tx_data(uint8_t custom_code, uint8_t data_code,uint8_t toggle);
extern void ir_rc5_repeat_test(uint8_t custom_code, uint8_t data_code,uint8_t toggle);

extern uint8_t bit_reverse(uint8_t b);



#endif //_PDM_H_
