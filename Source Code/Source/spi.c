#include "spi.h"
#include "debug.h"
#include <string.h>

#define    SERIAL_IF_CTRL     ((SERIAL_IF_CTRL_TYPE  *) SERIAL_IF_CTRL_BASE)
#define    SPI_MASTER_CTRL    ((SPI_MASTER_CTRL_TYPE *) SPI_MASTER_CTRL_BASE)
#define    SPI_SLAVE_CTRL     ((SPI_SLAVE_CTRL_TYPE  *) SPI_SLAVE_CTRL_BASE)

static SPI_IRQ_CALLBACK_TYPE spi_irq_cb[SPI_TYPE_NUM];

/* Share feature between master & slave */
void spi_set_speed(SPI_SPEED_TYPE speed)
{
    if (speed >= SPIM_SPEED_MAX)
        speed = SPIM_SPEED_4M;
    
    SERIAL_IF_CTRL->B_SPIM_SPEED = speed;
}

void spi_set_mode(SPI_MODE mode)
{
    if (mode > SPI_MODE_3_DOUT)
        mode = SPI_MODE_3_DOUT;
    
    SERIAL_IF_CTRL->B_CPHA = mode;
    SERIAL_IF_CTRL->B_CPOL = (mode >> 1);
}

void spi_set_irq_callback(SPI_TYPE type, SPI_IRQ_CALLBACK_TYPE cb)
{
    if (type >= SPI_TYPE_NUM)
        return;
    spi_irq_cb[type] = cb;
}

/* Master */
void SPIM_IRQHandler(void)
{
    if (spi_irq_cb[SPI_MASTER])
        spi_irq_cb[SPI_MASTER]();
    
    // clear interrupt
    SPI_MASTER_CTRL->SPIM_INT_CLEAR = 1;
}

void spim_enable(bool en)
{
    SPI_MASTER_CTRL->SPIM_ENABLE = (en != 0);
}

void spim_enable_int(bool int_en)
{
    SPI_MASTER_CTRL->SPIM_INT_MASK = (int_en == 0);
	
    if (int_en)
        NVIC_EnableIRQ(SPIM_IRQn);
    else
        NVIC_DisableIRQ(SPIM_IRQn);
}

void spim_resync(void)
{
    SERIAL_IF_CTRL->B_SPIM_RESYNC = 1;
}

void spim_exchange(uint8_t size, uint8_t * rx, uint8_t * tx)
{
    if (((int)rx & 3) || ((int)tx & 3)) {
        DBG("SPI DMA must be 4-byte alignment\r\n");
        return;
    }
    
    if (size > 32) size = 32;
    SPI_MASTER_CTRL->SPIM_RX_MAXCNT = size;
    SPI_MASTER_CTRL->SPIM_TX_MAXCNT = size;
    SPI_MASTER_CTRL->SPIM_RX_PTR = (uint32_t)rx;
    SPI_MASTER_CTRL->SPIM_TX_PTR = (uint32_t)tx;

    // Trigger SPI exchange
    SPI_MASTER_CTRL->MCU_TRIG_TRX = 1;
	while (!SPI_MASTER_CTRL->INT_SPIM_ENDTR);
	SPI_MASTER_CTRL->SPIM_INT_CLEAR=1;  //这里要清除中断 但是对于中断模式再另行控制
}

uint8_t SPI_Read_Write_A_Byte(uint8_t data)
{
	__align(4) uint8_t rx[4], tx[4];
	uint8_t size = 1;
	tx[0] = data;
	spim_exchange(size, rx, tx);
	
	return rx[0];
}

/* Slave */
void SPIS_IRQHandler(void)
{
    if (spi_irq_cb[SPI_SLAVE])
        spi_irq_cb[SPI_SLAVE]();
    
    // clear interrupt
    SPI_SLAVE_CTRL->SPIS_INT_CLEAR = 1;
}

void spis_enable(bool en)
{
    SPI_SLAVE_CTRL->SPIS_ENABLE = (en != 0);
}

void spis_enable_int(bool int_en)
{
    SPI_SLAVE_CTRL->SPIS_INT_MASK = (int_en == 0);
    
    if (int_en)
        NVIC_EnableIRQ(SPIS_IRQn);
    else
        NVIC_DisableIRQ(SPIS_IRQn);
}

void spis_prepare_exchange(uint8_t size, uint8_t * rx, uint8_t * tx)
{
    if (((int)rx & 3) || ((int)tx & 3)) {
        DBG("SPI DMA must be 4-byte alignment\r\n");
        return;
    }
    if (size > 32) size = 32;
    SPI_SLAVE_CTRL->SPIS_RX_MAXCNT = size;
    SPI_SLAVE_CTRL->SPIS_TX_MAXCNT = size;
    SPI_SLAVE_CTRL->SPIS_RX_PTR = (uint32_t)rx;
    SPI_SLAVE_CTRL->SPIS_TX_PTR = (uint32_t)tx;
    
    // Trigger SPI exchange
    SPI_SLAVE_CTRL->MCU_TRIG_TXDMA = 1;
}
