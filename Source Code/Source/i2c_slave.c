#include "i2c_slave.h"
#include "debug.h"
#include <string.h>

#define    I2C_CTRL    ((I2C_SLAVE_CTRL_TYPE *) I2C_SLAVE_CTRL_BASE)

static I2C_SLAVE_IRQ_CALLBACK_TYPE irq_cb;

void I2CS_IRQHandler(void)
{
    if (irq_cb) {
        irq_cb(I2C_CTRL->INT_STATUS);
    }
    I2C_CTRL->I2CS_INT_CLEAR = 1;
}

void i2c_slave_set_irq_callback(I2C_SLAVE_IRQ_CALLBACK_TYPE cb)
{
    irq_cb = cb;
}

void i2c_slave_enable_int(int en)
{
    I2C_CTRL->I2CS_INT_MASK = (en == 0);
}

void i2c_slave_enable(int en)
{
    I2C_CTRL->I2CS_ENABLE = (en != 0);
}

void i2c_slave_set_address_mode(I2C_SLAVE_ADDRESS_MODE_TYPE mode)
{
    I2C_CTRL->B_I2CS_ADDR_MODE = mode;
}

void i2c_slave_set_slave_id(int id1, int id2)
{
    I2C_CTRL->I2CS_SLAVE_ID1 = id1;
    I2C_CTRL->I2CS_SLAVE_ID2 = id2;
}

/* I2C SLAVE DMA support 1 byte alignment access */
bool i2c_slave_set_dma(I2C_SLAVE_DMA_TYPE type, void * ptr, uint32_t length)
{
    uint32_t dma_ptr = (uint32_t) ptr;
    
    if (!I2C_CTRL->I2CS_ENABLE) {
        DBG("I2C Slave is disabled\r\n");
        return false;
    }
    
    if (type == I2C_SLAVE_TX) {
        I2C_CTRL->TX_PTR = dma_ptr;
        I2C_CTRL->I2CS_TX_MAXCNT = length;
        return true;
    }
    
    I2C_CTRL->RX_PTR = dma_ptr;
    I2C_CTRL->I2CS_RX_MAXCNT = length;
    return true;
}
