#ifndef __CAPDET_ALG_LIB_H__
#define __CAPDET_ALG_LIB_H__

#include <stdint.h>
#include <stdbool.h>

#define CELL_MAX_NUM 16
#define HEADER_PATTERN0 0x57
#define HEADER_PATTERN1 0x58
#define HEADER_PATTERN2 0x59
#define HEADER_PATTERN3 0x5A

typedef struct {
	uint8_t		header[4];							/* header for UART communication */
	uint32_t	timestamp;
	int16_t		frm[CELL_MAX_NUM];					/* output frame buffer for AP */
	int16_t		debug_frm[CELL_MAX_NUM];			/* addtional output frame buffer for AP */		
	uint16_t	btn;								/* output maximum of 16 button info for AP in each bit, 1: pressed,  0: released. */	
	uint8_t		btn_pkg_idx;	
	uint8_t		frm_pkg_idx;
	uint8_t		op_mode;							/* current operation mode: run = 0, idle1 = 1, idle2 = 2, sleep= 3 */
} CAPDET_ALG_CONTEXT_TYPE;

typedef struct {
	uint8_t		cell_num;					/* user register: set active cell number (maximum 16 cells) */
	uint8_t		cycle_num;					/* user register: set scan cycle for SNR */
	uint8_t		channel_map[CELL_MAX_NUM];	/* user register: channel mapping for each cell position */
	uint16_t	scale_frm[CELL_MAX_NUM];	/* user register: set gain for each cells (Unit gain = 1000) */	
	uint8_t		frm_iir;					/* user register: frame based IIR filter (time domain) */
	uint8_t		frm_converge_cnt;			/* user register: set #frame for background frame convergence */
	bool		frm_show;					/* user register: show frame in frm[] */
	uint8_t		debug_frm_show;				/* user register: show debug frame in debug_frm[] buffer.  
												1: raw, 
												2: normalized raw, 
												3: background frame buffer
												4: delta frame, 
												5: scaled delta frame (after process of config_scale_frm[]), 														
											*/	
	uint16_t	neg_comp_frm;				/* user register: compensate the background cell if negative cell present time > neg_comp_frm &  */
	int16_t		neg_comp_val;				/*   												nagative cell value <= neg_comp_val	  */			
	uint16_t	pos_comp_frm;				/* user register: compensate the background cell if positive cell present time > pos_comp_frm &  */
	int16_t		pos_comp_val;				/*   												positive cell value > pos_comp_val	 & */	
	uint8_t		pos_comp_num;				/*   												positive cell num > config_pos_comp_num	  */	
	uint8_t		max_btn_allow;				/* user register: set how many buttons can be pressed simuntaniously */
	int16_t		touch_force;				/* user register: touch force for button detect */
	int16_t		release_force;				/* user register: release force for button detect */		
	
	uint8_t		fps_active;					/* user register: frame rate at active mode */	
	uint8_t		fps_idle1;					/* user register: frame rate at idle 1 mode */	
	uint8_t		fps_idle2;					/* user register: frame rate at idle 2 mode */	
	uint8_t		fps_sleep;					/* user register: frame rate at sleep mode */	
	int16_t		fps_idle1_cnt;				/* user register: frame count from active to idle 1 */	
	int16_t		fps_idle2_cnt;				/* user register: frame count from idle 1 to idle 2 */	
	int16_t		fps_sleep_cnt;				/* user register: frame count from idle 2 to sleep mode */	
	uint16_t	wake_up_mask;				/* user register: wake up cell mask at sleep mode */
	uint16_t	wake_up_th;					/* user register: wake up HW threshold at sleep mode,  set 0: auto calculate */
	uint16_t	settle_cycle;				/* user register: HW settle cycle */
	uint32_t	each_time;					/* user register: HW timeout setting */												
	uint8_t		capdet_ibias;				/* user register: HW ibias */
	uint8_t		full_cycle_cnt_en;			/* user register: HW cycle count method */	
	uint8_t		report_latency;				/* user register: Any cell value which had been >= touch_force && continued > report_latency will be treated as a button press. */	
	uint8_t		cap_detect_deglitch;		/* user register: HW deglitch margin.  */	
} CAPDET_ALG_CONFIG_TYPE;


void capdet_init(void);
bool capdet_process(CAPDET_ALG_CONTEXT_TYPE *context);		/* polling rate >= frame rate. return true if state change new btn_info or new frame buffer */
void capdet_int_handle(uint32_t timestamp);
void capdet_stop(void);
void capdet_start(void);
void capdet_sysclk_change(int32_t from, int32_t to);

/* communicate with AP */
void capdet_reg_read_all(CAPDET_ALG_CONFIG_TYPE *reg);
void capdet_reg_write_all(const CAPDET_ALG_CONFIG_TYPE *reg);

bool capdet_reg_read(const uint16_t addr, uint8_t *val);	 	/* return false if AP access the wrong address */
bool capdet_reg_write(const uint16_t addr, const uint8_t val);	/* return false if AP access the wrong address */

#endif
