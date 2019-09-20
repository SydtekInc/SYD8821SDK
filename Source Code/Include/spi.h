#ifndef _SPI_H_
#define _SPI_H_

#include <stdbool.h>
#include "ARMCM0.h"

typedef enum {
    /* PCLK 8MHz */
    SPIM_SPEED_125K,
		SPIM_SPEED_250K,
		SPIM_SPEED_500K,
    SPIM_SPEED_1M,
    SPIM_SPEED_2M,
    SPIM_SPEED_4M,
    SPIM_SPEED_MAX,
} SPI_SPEED_TYPE;

typedef enum {
    SPI_MASTER,
    SPI_SLAVE,
    SPI_TYPE_NUM,
} SPI_TYPE;


typedef enum {
    SPI_MODE_0_QIO = 0,
    SPI_MODE_1_QOUT,
    SPI_MODE_2_DIO,
		SPI_MODE_3_DOUT,
} SPI_MODE;


typedef void (* SPI_IRQ_CALLBACK_TYPE)(void);


//SPI1
#define SPI_SCLK_1 	GPIO_22
#define SPI_MISO_1 	GPIO_23
#define SPI_CSN_1 	GPIO_24
#define SPI_MOSI_1 	GPIO_25

//SPI0
#define SPI_SCLK_0 	GPIO_26
#define SPI_SDI_0 	GPIO_27
#define SPI_CSN_0 	GPIO_28
#define SPI_SDO_0 	GPIO_29



/* Share feature between master & slave */
void spi_set_speed(SPI_SPEED_TYPE speed);
void spi_set_mode(SPI_MODE mode);
void spi_set_irq_callback(SPI_TYPE type, SPI_IRQ_CALLBACK_TYPE cb);

/* Master */
void spim_enable(bool en);
void spim_enable_int(bool int_en);
void spim_resync(void);
void spim_exchange(uint8_t size, uint8_t * rx, uint8_t * tx);
uint8_t SPI_Read_Write_A_Byte(uint8_t data);
/* Slave */
void spis_enable(bool en);
void spis_enable_int(bool int_en);
void spis_prepare_exchange(uint8_t size, uint8_t * rx, uint8_t * tx);

#endif //_SPI_H_
