#ifndef _SC_READER_H_
#define _SC_READER_H_

#include "armcm0.h"
#include "config.h"

typedef enum SC_CALL_BACK_TYPE {
    ACT_CB,
    DACT_CB,
    CALL_BACK_NUM,
}SC_CB;

typedef enum SC_PPS_CALL_BACK_TYPE {
    PPS_REQUEST_CB,
    PPS_FAIL_CB,
    PPS_CALL_BACK_NUM,
}SC_PPS_CB;

typedef struct {
    uint8_t PPSS;
    uint8_t PPS[4];
    uint8_t PCK;
    uint8_t R_PPSS;
    uint8_t R_PPS[4];
    uint8_t R_PCK;
} SC_PPS;

typedef void (*CALLBACK)(void);
typedef bool (*PPS_CALLBACK)(SC_PPS *pps);

void sc_reader_enable(void);
void sc_reader_disable(void);
void sc_reader_task(void);
void sc_reader_config_vcc_level(bool en);
void sc_reader_config_retry(bool en);
void sc_reader_add_callback(CALLBACK c, SC_CB type);
void sc_reader_add_PPS_callback(PPS_CALLBACK c, SC_PPS_CB type);

#endif //_SC_READER_H_
