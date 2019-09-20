#ifndef _I2C_MASTER_H_
#define _I2C_MASTER_H_

#include "ARMCM0.h"
#include "stdbool.h"

#define    I2C_MASTER_ID_NUM           2
#define    I2C_MASTER_MAX_DMA_SIZE     256

typedef void (* I2C_MASTER_IRQ_CALLBACK_TYPE)(void);

typedef enum {
    I2C_MASTER_ADDRESS_READ,
    I2C_MASTER_CURRENT_ADDRESS_READ,
} I2C_MASTER_READ_MODE_TYPE;

typedef enum {
    I2C_MASTER_2BYTE_ADDRESS,
    I2C_MASTER_1BYTE_ADDRESS,
} I2C_MASTER_ADDRESS_MODE_TYPE;

I2C_MASTER_CTRL_TYPE * i2c_master_get_ctrl(int id);
void i2c_master_set_irq_callback(int id, I2C_MASTER_IRQ_CALLBACK_TYPE cb);
void i2c_master_enable(int id, int en);
void i2c_master_enable_int(int id, int en);
void i2c_master_set_read_mode(int id, I2C_MASTER_READ_MODE_TYPE mode);
void i2c_master_set_speed(int id, int speed);
void i2c_master_set_address_mode(int id, I2C_MASTER_ADDRESS_MODE_TYPE mode);
bool i2c_master_trigger_read(int module_id, int slave_id, int slave_address,
    int length, uint8_t *ptr);
bool i2c_master_trigger_write(int module_id, int slave_id, int slave_address,
    int length, uint8_t *ptr);
uint8_t i2c_write(int module_id, uint8_t slave_id, uint16_t addr, uint8_t * buf, uint16_t sz);
uint8_t i2c_read(int module_id, uint8_t slave_id, uint16_t addr, uint8_t * buf, uint16_t sz);
#endif //_I2C_MASTER_H_
