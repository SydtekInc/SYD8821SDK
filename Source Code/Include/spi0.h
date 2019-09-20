#ifndef _SPI0_H_
#define _SPI0_H_


#include "config.h"
#include "stdbool.h"

typedef enum {
    /* PCLK 8MHz */
    HID_SPEED_125K,
    HID_SPEED_250K,
    HID_SPEED_500K,
    HID_SPEED_1M,
    HID_SPEED_2M,
    HID_SPEED_4M,
    HID_SPEED_MAX,
} HID_SPI_SPEED_TYPE;

typedef enum {
    HID_4WIRE,
    HID_3WIRE,
    HID_2WIRE,
} HID_SPI_INTERFACE_TYPE;

typedef enum {
    HID_CS2C,
    HID_A2D,
    HID_C2CS,
} HID_SPI_DELAY_TYPE;

void hid_set_speed(HID_SPI_SPEED_TYPE speed);
void hid_switch_type(HID_SPI_INTERFACE_TYPE type);
void hid_write(uint8_t  size, uint8_t * addr, uint8_t * data);
void hid_read_normal_quick_burst(uint8_t  size, uint8_t * data);
void hid_read_normal_burst(uint8_t  size, uint8_t * addr, uint8_t * data);
void hid_read_avago_burst(uint8_t size, uint8_t addr, uint8_t *data);
void hid_resync(void);
void hid_set_mode(uint32_t mode);
HID_SPI_INTERFACE_TYPE hid_get_type(void);
void hid_set_spi_delay(HID_SPI_DELAY_TYPE type, uint32_t delay);

#endif //_HID_H_
