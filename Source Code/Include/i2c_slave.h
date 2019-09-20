#ifndef _I2C_SLAVE_H_
#define _I2C_SLAVE_H_

#include "ARMCM0.h"
#include "stdbool.h"

#define    I2C_SLAVE_MAX_DMA_SIZE    32

typedef void (* I2C_SLAVE_IRQ_CALLBACK_TYPE)(int int_st);

typedef enum {
    I2C_SLAVE_0BYTE_ADDRESS,
    I2C_SLAVE_1BYTE_ADDRESS,
    I2C_SLAVE_2BYTE_ADDRESS,
} I2C_SLAVE_ADDRESS_MODE_TYPE;

typedef enum {
    I2C_SLAVE_TX,
    I2C_SLAVE_RX,
} I2C_SLAVE_DMA_TYPE;

typedef enum {
    CMD_DONE = 1,
    READ_CMD = 2,
    WRITE_CMD = 4,
} I2C_SLAVE_INT_ST_TYPE;

void i2c_slave_set_irq_callback(I2C_SLAVE_IRQ_CALLBACK_TYPE cb);
void i2c_slave_enable_int(int en);
void i2c_slave_enable(int en);
void i2c_slave_set_slave_id(int id1, int id2);
void i2c_slave_set_address_mode(I2C_SLAVE_ADDRESS_MODE_TYPE mode);
bool i2c_slave_set_dma(I2C_SLAVE_DMA_TYPE type, void * ptr, uint32_t length);
#endif //_I2C_SLAVE_H_
