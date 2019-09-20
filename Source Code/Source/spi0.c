#include "spi0.h"
#include "debug.h"
#include "pad_mux_ctrl.h"
#include <string.h>

#define    HID_CTRL        ((HID_CTRL_TYPE *)   HID_CTRL_BASE)

#define    normal_BURST     0
#define    AVAGO_BURST      1

#define    REQ_QB_MODE       0x10
#define    REQ_WRITE         0x08
#define    REQ_START         0x04

static HID_SPI_INTERFACE_TYPE interface;

void hid_set_speed(HID_SPI_SPEED_TYPE speed)
{
    if (speed >= HID_SPEED_MAX)
        speed = HID_SPEED_125K;
    
    HID_CTRL->R_SPI_SPEED = speed;
}

HID_SPI_INTERFACE_TYPE hid_get_type(void)
{
    return interface;
}

void hid_switch_type(HID_SPI_INTERFACE_TYPE type)
{
    int i;
    switch (type) {
        case HID_4WIRE:
            for (i = 26; i < 30; i++)
                pad_mux_write(i, 3);
            break;
        case HID_3WIRE:
        case HID_2WIRE:
            pad_mux_write(29, 0);
            pad_mux_write(28, 4);
            pad_mux_write(27, 4);
            pad_mux_write(26, 4);
            break;
        default:
            return;
    }
    interface = type;
}

void hid_write(uint8_t  size, uint8_t * addr, uint8_t * data)
{
    int i;
    if (size > 7) size = 7;
    HID_CTRL->R_SPI_BURST_CNT = size;
    // single writes
    HID_CTRL->R_SPI_BURST_MODE = normal_BURST;
    
    for (i = 0; i < size; i++) {
        HID_CTRL->R_MOUSE_ADDR[i] = addr[i];
        HID_CTRL->R_MOUSE_WDATA[i] = data[i];
    }
    HID_CTRL->REQ = (REQ_START | REQ_WRITE);
    while (!HID_CTRL->HID_EOT);
}

/* 3 bytes data only */
void hid_read_normal_quick_burst(uint8_t  size, uint8_t * data)
{
    #define    QB_REQUEST    0x1
    int i;
    if (size > 3) size = 3;
    
    HID_CTRL->REQ = (REQ_START | REQ_QB_MODE);
    while (!HID_CTRL->HID_EOT);
    for (i = 0; i < size; i++)
        data[i] = HID_CTRL->HID_RDATA[i];
}

void hid_read_normal_burst(uint8_t  size, uint8_t * addr, uint8_t * data)
{
    int i;
    if (size > 7) size = 7;
    HID_CTRL->R_SPI_BURST_CNT = size;
    HID_CTRL->R_SPI_BURST_MODE = normal_BURST;
    for (i = 0; i < size; i++)
        HID_CTRL->R_MOUSE_ADDR[i] = addr[i];
    HID_CTRL->REQ = REQ_START;
    while (!HID_CTRL->HID_EOT);
    for (i = 0; i < size; i++)
        data[i] = HID_CTRL->HID_RDATA[i];
}

void hid_read_avago_burst(uint8_t  size, uint8_t addr, uint8_t *data)
{
    int i;
    if (size > 7) size = 7;
    HID_CTRL->R_SPI_BURST_CNT = size;
    HID_CTRL->R_SPI_BURST_MODE = AVAGO_BURST;
    HID_CTRL->R_MOUSE_ADDR[0] = addr;
    HID_CTRL->REQ = REQ_START;
    while (!HID_CTRL->HID_EOT);
    for (i = 0; i < size; i++)
        data[i] = HID_CTRL->HID_RDATA[i];
}

void hid_resync(void)
{
    HID_CTRL->SPI_RESYNC = 1;
}

void hid_set_mode(uint32_t mode)
{
    if (mode > 3)
        mode = 3;
    
    HID_CTRL->R_CPHA = mode;
    HID_CTRL->R_CPOL = (mode >> 1);
}

void hid_set_pol(uint32_t mode)
{
    if (mode > 3)
        mode = 3;
    
    HID_CTRL->R_CPHA = mode;
    HID_CTRL->R_CPOL = (mode >> 1);
}

void hid_set_spi_delay(HID_SPI_DELAY_TYPE type, uint32_t delay)
{
    if (type == HID_CS2C) {
        HID_CTRL->R_SPI_CS2SCK_TH = delay;
    }
    else if (type == HID_C2CS) {
        HID_CTRL->R_SPI_SCK2CS_TH = delay;
    }
    else {
        HID_CTRL->R_SPI_A2D_TH = delay;
    }
}
