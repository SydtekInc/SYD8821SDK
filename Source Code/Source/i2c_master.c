#include "i2c_master.h"
#include "debug.h"
#include <string.h>

static I2C_MASTER_CTRL_TYPE * I2C_CTRL[I2C_MASTER_ID_NUM] = 
{
    (I2C_MASTER_CTRL_TYPE *) I2C_MASTER0_CTRL_BASE,
    (I2C_MASTER_CTRL_TYPE *) I2C_MASTER1_CTRL_BASE,
};
static I2C_MASTER_IRQ_CALLBACK_TYPE irq_cb[I2C_MASTER_ID_NUM];

void I2CM0_IRQHandler(void)
{
    if (irq_cb[0])
        irq_cb[0]();
    I2C_CTRL[0]->I2CM_INT_CLEAR = 1;
}

void I2CM1_IRQHandler(void)
{
    if (irq_cb[1])
        irq_cb[1]();
    I2C_CTRL[1]->I2CM_INT_CLEAR = 1;
}

I2C_MASTER_CTRL_TYPE * i2c_master_get_ctrl(int id)
{
    return I2C_CTRL[id];
}

void i2c_master_set_irq_callback(int id, I2C_MASTER_IRQ_CALLBACK_TYPE cb)
{
    irq_cb[id] = cb;
}

void i2c_master_enable(int id, int en)
{
    I2C_CTRL[id]->I2CM_ENABLE = (en != 0);
}

void i2c_master_enable_int(int id, int en)
{
    I2C_CTRL[id]->I2CM_INT_MASK = (en == 0);
}

void i2c_master_set_read_mode(int id, I2C_MASTER_READ_MODE_TYPE mode)
{
    I2C_CTRL[id]->B_I2CM_CURR_ADDR = (mode != I2C_MASTER_ADDRESS_READ);
}

void i2c_master_set_speed(int id, int speed)
{
    I2C_CTRL[id]->B_I2CM_SPEED = speed;
}

void i2c_master_set_address_mode(int id, I2C_MASTER_ADDRESS_MODE_TYPE mode)
{
    I2C_CTRL[id]->B_I2CM_ADDR1B_MODE = (mode != I2C_MASTER_2BYTE_ADDRESS);
}

bool i2c_master_trigger_read(int module_id, int slave_id, int slave_address,
    int length, uint8_t *ptr)
{
    uint32_t dma_ptr = (uint32_t) ptr;
    
    if ((dma_ptr & 0x3) || length <= 0 || length > I2C_MASTER_MAX_DMA_SIZE) {
        DBG("DMA parameter incorrect\r\n");
        return false;
    }
    
    if (!I2C_CTRL[module_id]->I2CM_ENABLE) {
        DBG("I2C Master %d is disabled\r\n", module_id);
        return false;
    }
    
    I2C_CTRL[module_id]->I2CM_RX_MAXCNT = length;
    I2C_CTRL[module_id]->RX_PTR = dma_ptr;
    I2C_CTRL[module_id]->B_I2CM_DEV_INST = slave_id;
    I2C_CTRL[module_id]->B_I2CM_DEV_WRADDR = slave_address;
    I2C_CTRL[module_id]->I2CM_MCU_TRIG_RD = 1;
    
    return true;
}

bool i2c_master_trigger_write(int module_id, int slave_id, int slave_address,
    int length, uint8_t *ptr)
{
    uint32_t dma_ptr = (uint32_t) ptr;
    
    if ((dma_ptr & 0x3) || length <= 0 || length > I2C_MASTER_MAX_DMA_SIZE) {
        DBG("DMA parameter incorrect\r\n");
        return false;
    }
    
    if (!I2C_CTRL[module_id]->I2CM_ENABLE) {
        DBG("I2C Master %d is disabled\r\n", module_id);
        return false;
    }
    
    I2C_CTRL[module_id]->I2CM_TX_MAXCNT = length;
    I2C_CTRL[module_id]->TX_PTR = dma_ptr;
    I2C_CTRL[module_id]->B_I2CM_DEV_INST = slave_id;
    I2C_CTRL[module_id]->B_I2CM_DEV_WRADDR = slave_address;
    I2C_CTRL[module_id]->I2CM_MCU_TRIG_WR = 1;
    
    return true;
}

uint8_t i2c_write(int module_id, uint8_t slave_id, uint16_t addr, uint8_t * buf, uint16_t sz)
{
    uint8_t int_mask = I2C_CTRL[module_id]->I2CM_INT_MASK;
    uint8_t ret = 0;
    
    // Disable interrupt for polling mode
    I2C_CTRL[module_id]->I2CM_INT_MASK = 1;
    I2C_CTRL[module_id]->I2CM_INT_CLEAR = 1;
    
    if (i2c_master_trigger_write(module_id, slave_id, addr, sz, buf)) {
        while(!I2C_CTRL[module_id]->INT_I2CM_STOPPED) ;
        I2C_CTRL[module_id]->I2CM_INT_CLEAR = 1;
    }
    else {
        ret = 1;
    }
    
    // Restore interrupt configuration
    I2C_CTRL[module_id]->I2CM_INT_MASK = int_mask;
    return ret;
}

uint8_t i2c_read(int module_id, uint8_t slave_id, uint16_t addr, uint8_t * buf, uint16_t sz)
{
    uint8_t int_mask = I2C_CTRL[module_id]->I2CM_INT_MASK;
    uint8_t ret = 0;
    
    // Disable interrupt for polling mode
    I2C_CTRL[module_id]->I2CM_INT_MASK = 1;
    I2C_CTRL[module_id]->I2CM_INT_CLEAR = 1;
    
    if (i2c_master_trigger_read(module_id, slave_id, addr, sz, buf)) {
        while(!I2C_CTRL[module_id]->INT_I2CM_STOPPED) ;
        I2C_CTRL[module_id]->I2CM_INT_CLEAR = 1;
    }
    else {
        ret = 1;
    }
    
    // Restore interrupt configuration
    I2C_CTRL[module_id]->I2CM_INT_MASK = int_mask;
    return ret;
}
