#include <string.h>
#include "atr_decoder.h"
#include "debug.h"
#include "queue.h"
#include "sc_reader.h"
#include "tpdu.h"

#define    SC_CTRL    ((SC_READER_CTRL_TYPE *)SC_7816_CTRL_BASE)
#define    SYS_CTRL   ((SYS_CONFIG_TYPE *) SYS_CTRL_BASE)

// bit mask for deferred task
enum SC_DEFERRED_TASK_MASK {
    CARD_DET_TASK = U32BIT(0),
    ATR_DONE_TASK = U32BIT(1),
    ATR_FAIL_TASK = U32BIT(2),
    DEACTIVATION_TASK = U32BIT(3),
};

// TPDU status
typedef enum {
    NOT_READY,
    PPS_REQUESTED,
    ACTIVATED,
} TASK_STATE;

// Module interrupt mask
enum SC_INT_MASK {
    TX_DONE = U32BIT(0),
    TX_FIFO_EMPTY = U32BIT(1),
    TX_FIFO_THRESHOLD = U32BIT(2),
    TX_PARITY_ERROR = U32BIT(3),
    TX_RETRY = U32BIT(4),
    TX_RETRY_OVER_LIMIT = U32BIT(5),
    RX_FIFO_NOT_EMPTY = U32BIT(6),
    RX_FIFO_THRESHALL = U32BIT(7),
    RX_TIMEOUT = U32BIT(8),
    RX_PARITY_ERROR = U32BIT(9),
    RX_RETRY = U32BIT(10),
    RX_RETRY_OVER_LIMIT = U32BIT(11),
    WAITING_TIMEOUT = U32BIT(12),
    ATR_DONE = U32BIT(13),
    ATR_FAIL = U32BIT(14),
    CARD_DETECT = U32BIT(15),
    CARD_REMOVE = U32BIT(16),
    DEACTIVATE = U32BIT(17),
    SC_INT_ALL = 0x3FFFF,
};

/* Critical section for SC */
#define SC_CS_IN()                                      \
            do {                                        \
                old_primask = __get_PRIMASK();          \
                __disable_irq();                        \
            } while(false)
#define SC_CS_OUT()                                     \
            do {                                        \
                __set_PRIMASK(old_primask);             \
            } while(false)

/* Define which interrupts are used for TX/RX */
#define RX_FIFO_INT     (RX_FIFO_THRESHALL | RX_TIMEOUT)
#define TX_FIFO_INT     (TX_FIFO_EMPTY)

            
static const uint32_t F_Table[16] = {372, 372, 558, 744, 1116, 1488, 1860, 0,
                                    0, 512, 768, 1024, 1536, 2048};
static const uint32_t D_Table[16] = {0, 1, 2, 4, 8, 16, 32, 64, 12, 20};

static volatile uint32_t old_primask;
static volatile uint32_t task_flag;
static volatile bool tx_finished;
static bool card_in = false;
static SC_ATR ATR;
static SC_PPS PPS;
static bool card_activated = false;
static bool card_clock_stoppable = false;
static CALLBACK callback[CALL_BACK_NUM];
static PPS_CALLBACK pps_cb[PPS_CALL_BACK_NUM];
static bool enable_retry = true;
static uint8_t buffer[264];
static QUEUE sc_queue = {0, 0, sizeof(buffer), buffer};

static void sc_reader_set_FD(uint8_t fd);
static void sc_reader_spec_default(void);
static void sc_reader_sendPPS(SC_PPS *pps);
static bool sc_reader_pps_verification(SC_PPS *pps);
__STATIC_INLINE bool get_byte(uint8_t *data);
__STATIC_INLINE void sc_reader_enable_retry(void);
void sc_reader_send(uint8_t *buf, int length);
bool sc_reader_get(uint8_t *buf, int length);
void sc_reader_deactivate(void);
void sc_reader_dump_info(void);




/* Configure module auto activation when card being inserted */
void sc_reader_config_auto(bool en)
{
    SC_CTRL->AUTO_ACTIVATION_EN = (en != 0);
}

/* Configure vcc output when activated */
void sc_reader_config_vcc_level(bool en)
{
    SC_CTRL->ACTIVE_VCC_LEVEL = (en != 0);
}

/* Configure retry enable for each transaction */
void sc_reader_config_retry(bool en)
{
    enable_retry = (en != 0);
}

/* Configure module settings */
void sc_reader_enable(void)
{
    SYS_CTRL->CLK_SC_EN = 1;
    
    /* Configure RESET & ACTIVATE timeout to maximum */
    SC_CTRL->RESET_DURATION = 400;
    SC_CTRL->ACTIVATE_DURATION = 30;
    
    /* Configure FIFO threshold, FIFO size is 8, set half to inform*/
    SC_CTRL->RX_FIFO_THRESHOLD = 0x4;
    SC_CTRL->TX_FIFO_THRESHOLD = 0x4;
    
    /* Retry configuration */
    SC_CTRL->TX_RETRY_CNT = 3;
    SC_CTRL->RX_RETRY_CNT = 3;
    
    /* Enable RX FIFO Timeout */
    SC_CTRL->RX_TIMEOUT_DETECT_EN = true;
    
    sc_reader_spec_default();
    
    /* Enable TX/RX */
    SC_CTRL->TX_ENABLE = true;
    SC_CTRL->RX_ENABLE = true;
}

void sc_reader_disable(void)
{
    sc_reader_deactivate();
    SYS_CTRL->CLK_SC_EN = 0;
}

void sc_reader_deactivate(void)
{
    if (!card_in)
        return;

    SC_CTRL->TRIGGER_DEACTIVATE = true;
}

void sc_reader_activate(void)
{
    if (!card_in)
        return;

    SC_CTRL->TRIGGER_ACTIVATE = true;
}

void sc_reader_warm_reset(void)
{
    if (!card_in || !card_activated)
        return;

    sc_reader_spec_default();
    card_activated = false;
    SC_CTRL->TRIGGER_WARM_RESET = true;
}

/* Interrupt Handler */
void ISO_7816_IRQHandler(void)
{
    uint32_t mask = SC_CTRL->INTS_STATE & SC_CTRL->INTS_EN;

    // Optimize empty interrupt might occur when access FIFO
    if (!mask)
        return;
    
    if (mask & DEACTIVATE) {
        task_flag |= DEACTIVATION_TASK;
    }
    
    if (mask & (CARD_DETECT | CARD_REMOVE)) {
        card_in = ((mask & CARD_DETECT) != 0);
        task_flag |= CARD_DET_TASK;
    }

    // RX fifo data available
    if (mask & RX_FIFO_INT) {

		//dbg_printf("RX_FIFO_INT--SC_CTRL->DATA:%x\r\n",SC_CTRL->DATA);
	
        while (SC_CTRL->RX_FIFO_COUNT) {
            enqueue(&sc_queue, SC_CTRL->DATA);
        }
    }
    
    // TX fifo available
    if (mask & TX_FIFO_INT) {
        uint8_t data;
        
        // fill data until full or no data
        while (!SC_CTRL->TX_FIFO_FULL && dequeue(&sc_queue, &data)) {
            SC_CTRL->DATA = data;
        }
        
        // TX Done
        if (is_queue_empty(&sc_queue) && SC_CTRL->TX_FIFO_EMPTY) {
            // Disable TX FIFO available notification
            SC_CTRL->INTS_EN &= ~TX_FIFO_INT;
            SC_CTRL->INTS_EN |= RX_FIFO_INT;
            tx_finished = true;
        }
    }
    
    // ATR failed
    if (mask & ATR_FAIL) {
        // Consumes RX FIFO
        while (SC_CTRL->RX_FIFO_COUNT) {
            enqueue(&sc_queue, SC_CTRL->DATA);
        }
        task_flag |= ATR_FAIL_TASK;
       //dbg_printf("ATR_FAIL, INT_ST = 0x%X\r\n", SC_CTRL->INTS_STATE);
    }

    if (mask & ATR_DONE) {

		//dbg_printf("ATR_DONE--SC_CTRL->DATA:%x\r\n",SC_CTRL->DATA);
	
        // Consume RX FIFO data
        while (SC_CTRL->RX_FIFO_COUNT) {
            enqueue(&sc_queue, SC_CTRL->DATA);
        }
        task_flag |= ATR_DONE_TASK;
    }
    
    if (mask & TX_RETRY_OVER_LIMIT) {
        if (SC_CTRL->TX_FORCE_PARITY_ERROR)
            SC_CTRL->TX_FORCE_PARITY_ERROR = false;
        else
            SC_CTRL->TX_RETRY_ENABLE = false;
    }
    
    if (mask & RX_RETRY_OVER_LIMIT) {
        if (SC_CTRL->RX_FORCE_PARITY_ERROR)
            SC_CTRL->RX_FORCE_PARITY_ERROR = false;
        else
            SC_CTRL->RX_RETRY_ENABLE = false;
    }
    
    // Clear interrupt
    SC_CTRL->INTS_STATE = mask;
}

/* 
 * Perform deferred tasks
 * Do TPDU request & PPS verification
 */


//uint8_t Get_Challenge_COM[5] = {0x00, 0x84, 0x00, 0x00,0x04};


void get_challenge_test_handle(TPDU_COMMAND *command)
{

	uint8_t i =0;
	uint8_t *buf;
	
	dbg_printf("get_challenge_test_handle\r\n");
	
	dbg_printf("command->header: ");
	for (i = 0; i < HEADER_SIZE ; i++)
        dbg_printf("%02X ", command->header[i]);

    dbg_printf("\r\n");

	

	dbg_printf("command->writeCommand:%x\r\n",command->writeCommand);


	dbg_printf("command->sw: ");
	for (i = 0; i < SW_SIZE ; i++)
        dbg_printf("%02X ", command->sw[i]);

    dbg_printf("\r\n");

	if((command->sw[0] == 0x90) &&(command->sw[1] == 0x00))
	{
		dbg_printf("command->data: ");
		for (i = 0; i < command->header[4] ; i++)
       		 dbg_printf("%02X ", command->data[i]);
	
    	dbg_printf("\r\n");
	}
	else
	{
		dbg_printf("command->sw is error\r\n");
	}
}


TPDU_COMMAND get_challenge_test = {0}; 


void test_tpdu(void)
{
	
	get_challenge_test.header[0] = 0x00;
	get_challenge_test.header[1] = 0x84;
	get_challenge_test.header[2] = 0x00;
	get_challenge_test.header[3] = 0x00;
	get_challenge_test.header[4] = 0x04;
	get_challenge_test.writeCommand = 0;
	
	tpdu_request(&get_challenge_test,get_challenge_test_handle);
}




void sc_reader_task(void)
{

	static	uint8_t send_test = 0;
    uint32_t flag;
    static TASK_STATE state;
    
    SC_CS_IN();
    flag = task_flag;
    task_flag = 0;
    SC_CS_OUT();
    
    // Handle ISR deferred tasks

	//dbg_printf("sc_reader_task_task_flag:%x\r\n",task_flag);

    if (flag) 
	{
        if (flag & CARD_DET_TASK) {
            card_activated = false;
            state = NOT_READY;
            dbg_printf(card_in ? "CARD DETECTED\r\n" : "CARD REMOVED\r\n");
        }
        
        // ATR Received
        if (flag & ATR_DONE_TASK) 
		{
            uint8_t *buf = (uint8_t *)&ATR;

            state = NOT_READY;
            // Get ATR
            memset(buf, 0, sizeof(ATR));
            while(true) {
                if (!dequeue(&sc_queue, buf++))
                    break;
                ATR.TotalLength++;
            }
            
            if (atr_decode(&ATR)) //check error
			{ 
                sc_reader_deactivate();
            }
            else 
			{
                bool clock_lvl;
                // Timing configuration
                SC_CTRL->EXTRA_GUARD_TIME = atr_decoder_get_extra_guard_time();
                
                // Clock stop
                card_clock_stoppable = atr_decoder_get_clock_stop(&clock_lvl);
                SC_CTRL->CLK_STOP_VAL = clock_lvl;
                
                // Setup PPS
                PPS.PPSS = 0xFF;
                PPS.PPS[0] = 0x10; // T0
                PPS.PPS[1] = atr_decoder_get_FD();

				dbg_printf("ATR_DONE_TASK\r\n");

                if (!atr_decoder_allow_pps() &&
                   (pps_cb[PPS_REQUEST_CB] == NULL ||
                    pps_cb[PPS_REQUEST_CB](&PPS)))
                {
                	dbg_printf("send pps\r\n");
                    // send pps
                    sc_reader_sendPPS(&PPS);
                    state = PPS_REQUESTED;
                    card_activated = true;
                }
                else if (!atr_decoder_get_protocol()) 
				{ // T0 specific mode
					sc_reader_dump_info();
					
					dbg_printf("atr_decoder_get_protocol  T0 specific mode\r\n");
                    // set fd
                    //sc_reader_set_FD(DEFAULT_FD);
                    tpdu_reset();
                    state = ACTIVATED;
                    card_activated = true;
                } // T1 and specific mode but allow to change to negotiable mode
                else if (atr_decoder_allow_switch_mode()) {
					dbg_printf("atr_decoder_allow_switch_mode\r\n"); 
                    sc_reader_warm_reset();
                }
                // Not support
                else {
                    sc_reader_deactivate();
                }
				
            }
        }
        
        if (flag & ATR_FAIL_TASK) 
		{
            uint8_t *buf = (uint8_t *)&ATR;
            state = NOT_READY;
            // Get ATR
            memset(buf, 0, sizeof(ATR));
            while(true) {
                if (!dequeue(&sc_queue, buf++))
                    break;
                ATR.TotalLength++;
            }
        }
            
        if (flag & DEACTIVATION_TASK) 
		{
            card_activated = false;
            SC_CS_IN();
            queue_reset(&sc_queue);
            sc_reader_spec_default();
            SC_CS_OUT();
            state = NOT_READY;
            dbg_printf("DEACTIVATED\r\n");
			
			sc_reader_dump_info();
			
            if (callback[DACT_CB]) {
                callback[DACT_CB]();
            }
        }
    }
    
    /* If not in activation, then no TPDU or PPS will be performed */
    if (!card_in || !card_activated) 
	{
        tpdu_reset();
        return;
    }

    // SC reader state machine
    if (state == ACTIVATED) 
	{
        static TPDU_TASK_STATE tpdu_state;
        tpdu_state = tpdu_task(tx_finished);
		//
		
        if (tpdu_state == TT_IDLE) {
			
			
            if (card_clock_stoppable){
                SC_CTRL->CLK_STOP_EN = true;
            }

			if(send_test == 0)	{
				send_test = 1;
				dbg_printf("state == ACTIVATED,test_tpdu()\r\n");
				test_tpdu();
			}
			
            if (callback[ACT_CB])
                callback[ACT_CB]();
        }
        else if (tpdu_state == TT_ERROR) {
            dbg_printf("TPDU error\r\n");
            sc_reader_deactivate();
            state = NOT_READY;
        }
    }
    else if (state == PPS_REQUESTED) 
	{
        while (!tx_finished);
        if (sc_reader_pps_verification(&PPS)) {
            state = ACTIVATED;
            dbg_printf("PPS Done\r\n");
			
			sc_reader_dump_info();
			{
				//uint8_t Get_Challenge_COM[5] = {0x00, 0x84, 0x00, 0x00,0x04};
				//sc_reader_send( Get_Challenge_COM,sizeof( Get_Challenge_COM));
			}
			
        }
        else {
            if (pps_cb[PPS_FAIL_CB])
                pps_cb[PPS_FAIL_CB](&PPS);
            sc_reader_deactivate();
            state = NOT_READY;
        }
    }
}

/* TX helper function */
void sc_reader_send(uint8_t *buf, int length)
{
    int i;
    
    SC_CS_IN();
    queue_reset(&sc_queue);
    SC_CS_OUT();

	
    for (i = 0; i < length; i++) {
        enqueue(&sc_queue, buf[i]);
    }
	
    tx_finished = false;
    sc_reader_enable_retry();
    SC_CTRL->CLK_STOP_EN = false;
    SC_CTRL->INTS_EN &= ~RX_FIFO_INT;
    SC_CTRL->INTS_EN |= TX_FIFO_INT;
    SC_CTRL->INTS_STATE = WAITING_TIMEOUT;
}

/* RX helper function */
bool sc_reader_get(uint8_t *buf, int length)
{
    int i;
    
    if (queue_size(&sc_queue) < length) 
	{
        // Timeout and data not enough => deactivate
        if (SC_CTRL->INTS_STATE & WAITING_TIMEOUT) {
            //dbg_printf("RX queue size = %d\r\n", queue_size(&sc_queue));
            sc_reader_deactivate();
        }
        return false;
    }
    SC_CS_IN();
    for (i = 0; i < length; i++, buf++) {
        dequeue(&sc_queue, buf);
    }
    SC_CS_OUT();
    
    return true;
}

void sc_reader_config_clock_stoppable(bool en)
{
    card_clock_stoppable = en;
}

/* Configure module output clock */
void sc_reader_config_clock_div(uint16_t div)
{	
    if (div <= 1)
        return;
    /* if div is 3, duty cyle will be 2 : 1, which violates spec */
    if (div == 4)
        div = 5;
		
    SC_CTRL->SC_CLK_DIV = div - 1;
}

/* Debug Information */
void sc_reader_dump_info(void)
{
    int i;
    uint8_t *buf;
    
    dbg_printf("card      : %s\r\n", card_in ? "in" : "out");
    dbg_printf("auto      : %d\r\n", SC_CTRL->AUTO_ACTIVATION_EN);
    dbg_printf("vcc       : %d\r\n", SC_CTRL->ACTIVE_VCC_LEVEL);
    dbg_printf("activated : %d\r\n", card_activated);
    dbg_printf("stoppable : %d\r\n", card_clock_stoppable);
    dbg_printf("retry     : %d\r\n", enable_retry);
    
    // ATR sequence
    dbg_printf("ATR       : ");
    for (i = 0, buf = (uint8_t *)&ATR; i < ATR.TotalLength; i++)
        dbg_printf("%02X ", *buf++);
    dbg_printf("\r\n");
    
    // PPS
    dbg_printf("PPS       : ");
    for (i = 0, buf = (uint8_t *)&PPS; i < sizeof(PPS); i++)
        dbg_printf("%02X ", *buf++);
    dbg_printf("\r\n");
}

/* Callback registration for ACT/DACT */
void sc_reader_add_callback(CALLBACK c, SC_CB type)
{
    if (type >= CALL_BACK_NUM)
        return;
    callback[type] = c;
}

/* Callback registration for PPS releated events */
void sc_reader_add_PPS_callback(PPS_CALLBACK c, SC_PPS_CB type)
{
    if (type >= PPS_CALL_BACK_NUM)
        return;
    pps_cb[type] = c;
}

/* Reset module configuration */
static void sc_reader_spec_default(void)
{
    NVIC_DisableIRQ(ISO7816_IRQn);
    atr_reset();
    
    SC_CTRL->EXTRA_GUARD_TIME = 0;
    sc_reader_set_FD(0x01);
    SC_CTRL->CLK_STOP_EN = false;
    card_clock_stoppable = false;
    
    /* Reset RX queue for ATR*/
    queue_reset(&sc_queue);

    sc_reader_enable_retry();

    /* Enable SC Interrupt */
    SC_CTRL->INTS_EN = RX_FIFO_INT | ATR_DONE |
                       ATR_FAIL | CARD_DETECT |
                       CARD_REMOVE | DEACTIVATE |
                       TX_RETRY_OVER_LIMIT | RX_RETRY_OVER_LIMIT;

    /* Set clk to <= 5MHz */
    {
				// Source by PCLK
        uint32_t clk = 64000000 / 2;
        clk /= 5000000;
        sc_reader_config_clock_div((uint16_t)clk);
    }
    
    /* Clear Pending Interrupt */
    SC_CTRL->INTS_STATE = SC_INT_ALL;
    NVIC_EnableIRQ(ISO7816_IRQn);
}

/* FD configuration */
static void sc_reader_set_FD(uint8_t fd)
{
    int f = (fd >> 4) & 0x0F;
    int d = fd & 0x0F;
    int fi = (atr_decoder_get_FD() >> 4) & 0x0F;	
    uint32_t w;
    
	if (F_Table[f] == 0 || D_Table[d] == 0)//F:372/D:1
        SC_CTRL->BAUD_CLK_DIV = 0x173;
    else
        SC_CTRL->BAUD_CLK_DIV = (F_Table[f] / D_Table[d]) - 1;
	
    // 16 etu for RX FIFO timeout
    SC_CTRL->RX_TIMEOUT_DELAY = 16 * (SC_CTRL->BAUD_CLK_DIV + 1);
    
    SC_CTRL->GUARD_TIME = (D_Table[d] == 64) ? 16 : 12;
    
    w = 960 * atr_decoder_get_waiting_integer() * F_Table[fi];

	// 960 etu based
    w /= ((SC_CTRL->BAUD_CLK_DIV + 1) * 960);
    SC_CTRL->WAITING_TIME = w;
}

static bool sc_reader_pps_verification(SC_PPS *pps)
{
    int i;
    uint8_t mask = 0xFF, fd = DEFAULT_FD;
    bool pass = true;
    
    // get PPSS
    if (!get_byte(&pps->R_PPSS))
        goto PPS_VERIFY_FAIL;
    pass &= (pps->R_PPSS == 0xFF);
    
    // get PPS0
    if (!get_byte(&pps->R_PPS[0]))
        goto PPS_VERIFY_FAIL;
    
    // must be T=0
    pass &= (((pps->R_PPS[0] ^ pps->PPS[0]) & 0x0F) == 0);
    mask ^= pps->R_PPS[0];
    
    for (i = 1; i < 3; i++) {
        if (pps->R_PPS[0] & (0x08 << i)) {
            // Get data
            if (!get_byte(&pps->R_PPS[i]))
                goto PPS_VERIFY_FAIL;
            mask ^= pps->R_PPS[i];
            // Must be equal to request
            pass &= (pps->R_PPS[i] == pps->PPS[i]);
            
            if (i == 1) { /* PPS1 */
                fd = pps->R_PPS[i];
            }
        }   
    }
    
    // PCK
    if (!get_byte(&pps->R_PCK))
        goto PPS_VERIFY_FAIL;
    mask ^= pps->R_PCK;
    pass &= (mask == 0);


	dbg_printf("sc_reader_pps_verification\r\n");
	dbg_printf("Fd:%x\r\n",fd);
    sc_reader_set_FD(fd);
    
    return pass;
    PPS_VERIFY_FAIL:
    return false;
}

/* PPS helper function */
static void sc_reader_sendPPS(SC_PPS *pps)
{
    uint8_t tmp[6];
    int idx = 0, i;

    tmp[idx++] = pps->PPSS;
    tmp[idx++] = pps->PPS[0];
    pps->PCK = PPS.PPSS ^ PPS.PPS[0];
    for (i = 1; i < 3 ; i++) {
        if((pps->PPS[0] >> i) & 0x08) {
            tmp[idx++] = pps->PPS[i];
            pps->PCK ^= pps->PPS[i];
        }
    }
    tmp[idx++] = pps->PCK;
    
    sc_reader_send(tmp, idx);
}

/* Consume all data in queue */
__STATIC_INLINE bool get_byte(uint8_t *data)
{
    while(true) {
        if (!is_queue_empty(&sc_queue)) {
            SC_CS_IN();                 
            dequeue(&sc_queue, data);
            SC_CS_OUT();
            return true;
        }
        else if (SC_CTRL->INTS_STATE & WAITING_TIMEOUT) {
            dbg_printf("TIMEOUT\r\n");
            sc_reader_deactivate();
            break;
        }
        else if (!card_in) 
		{ // card being removed, no timeout will be detected
            break;
        }
    }
    return false;
}

__STATIC_INLINE void sc_reader_enable_retry(void)
{
    if (!enable_retry) return;
    SC_CTRL->TX_RETRY_ENABLE = true;
    SC_CTRL->RX_RETRY_ENABLE = true;
}
