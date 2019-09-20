#include "sc_reader.h"
#include "debug.h"
#include "tpdu.h"

#define    BUFF_MAX_LENGTH     (512)
#define    BUFF_LENGTH         (BUFF_MAX_LENGTH + 1)
#define    SC_CTRL             ((SC_READER_CTRL_TYPE *)SC_7816_CTRL_BASE)

static int count;
static int set_count;
static bool stress_act_trigger;
static bool reseted = false;

extern void sc_reader_warm_reset(void);
extern void sc_reader_config_auto(bool);
extern void sc_reader_deactivate(void);
extern void sc_reader_activate(void);
extern void sc_reader_dump_info(void);
extern void sc_reader_config_clock_stoppable(bool en);
extern void atr_decoder_config_default_FD(uint8_t fd);

//void sc_test_init(void)
//{
//    pad_mux_write(25, 11); //VCCCTL
//    pad_mux_write(26, 11); //DET
//    pad_mux_write(27, 11); //CLK
//    pad_mux_write(28, 11); //IO
//    pad_mux_write(29, 11); //RST
//    sc_reader_enable();
//}

//void main()
//{
//	sc_test_init();
//	while(1) {
//		sc_reader_task();
//	}
//}

