#ifndef _BLE_PROFILE_H_
#define _BLE_PROFILE_H_

#include "ARMCM0.h"

/* Handle Index */
#define BLE_UART_NOTIFY_IDX 0
#define BLE_Battry_Level_IDX 1
#define BLE_WECHAT_Indication_IDX 2
#define BLE_Pedometer_Measurement_IDX 3
#define BLE_Sport_Target_IDX 4
#define BLE_BOOT_KEY_REPORT_IDX 5
#define BLE_BOOT_MOUSE_REPORT_IDX 6
#define BLE_Consumer_REPORT_IDX 7
#define BLE_BOOT_KEYBOARD_INPUT_REPORT_IDX 8
#define BLE_BOOT_MOUSE_INPUT_REPORT_IDX 9

/* GENERIC_ACCESS*/
#define BLE_GENERIC_ACCESS 0x1800
#define BLE_DEVICE_NAME_UUID 0x2A00
#define BLE_DEVICE_NAME_VALUE_HANDLE 0x0003
#define BLE_APPEARANCE_UUID 0x2A01
#define BLE_APPEARANCE_VALUE_HANDLE 0x0005
#define BLE_Peripheral_Preferred_Connection_Parameters_UUID 0x2A04
#define BLE_Peripheral_Preferred_Connection_Parameters_VALUE_HANDLE 0x0007

/* Generic_Attribute*/
#define BLE_Generic_Attribute 0x1801
#define BLE_Service_Changed_UUID 0x2A05
#define BLE_Service_Changed_VALUE_HANDLE 0x000A

/* Device_Information*/
#define BLE_Device_Information 0x180A
#define BLE_MANUFACTURER_NAME_STRING_UUID 0x2A29
#define BLE_MANUFACTURER_NAME_STRING_VALUE_HANDLE 0x000E
#define BLE_MODEL_NUMBER_STRING_UUID 0x2A24
#define BLE_MODEL_NUMBER_STRING_VALUE_HANDLE 0x0010
#define BLE_SERIAL_NUMBER_STRING_UUID 0x2A25
#define BLE_SERIAL_NUMBER_STRING_VALUE_HANDLE 0x0012
#define BLE_HARDWARE_REVISION_STRING_UUID 0x2A27
#define BLE_HARDWARE_REVISION_STRING_VALUE_HANDLE 0x0014
#define BLE_FIRMWARE_REVISION_STRING_UUID 0x2A26
#define BLE_FIRMWARE_REVISION_STRING_VALUE_HANDLE 0x0016
#define BLE_SOFTWARE_REVISION_STRING_UUID 0x2A28
#define BLE_SOFTWARE_REVISION_STRING_VALUE_HANDLE 0x0018
#define BLE_PNP_ID_UUID 0x2A50
#define BLE_PNP_ID_VALUE_HANDLE 0x001A

/* UART*/
#define BLE_UART 0x0001
#define BLE_UART_Write_UUID 0x0002
#define BLE_UART_Write_VALUE_HANDLE 0x001D
#define BLE_UART_NOTIFY_UUID 0x0003
#define BLE_UART_NOTIFY_VALUE_HANDLE 0x001F

/* OTA*/
#define BLE_OTA 0xFF00
#define BLE_OTA_Read_Write_UUID 0xFF01
#define BLE_OTA_Read_Write_VALUE_HANDLE 0x0023

/* BATTERY_SERVICE*/
#define BLE_BATTERY_SERVICE 0x180F
#define BLE_Battry_Level_UUID 0x2A19
#define BLE_Battry_Level_VALUE_HANDLE 0x0026

/* WECHAT*/
#define BLE_WECHAT 0xfee7
#define BLE_WECHAT_WRITE_UUID 0xFEC7
#define BLE_WECHAT_WRITE_VALUE_HANDLE 0x002A
#define BLE_WECHAT_Indication_UUID 0xFEC8
#define BLE_WECHAT_Indication_VALUE_HANDLE 0x002C
#define BLE_WECHAT_Read_UUID 0xFEC9
#define BLE_WECHAT_Read_VALUE_HANDLE 0x002F
#define BLE_Pedometer_Measurement_UUID 0xFEA1
#define BLE_Pedometer_Measurement_VALUE_HANDLE 0x0031
#define BLE_Sport_Target_UUID 0xFEA2
#define BLE_Sport_Target_VALUE_HANDLE 0x0034

/* HIDO_SERVICE*/
#define BLE_HIDO_SERVICE 0x1812
#define BLE_PROTOCOL_MODE_UUID 0x2A4E
#define BLE_PROTOCOL_MODE_VALUE_HANDLE 0x0038
#define BLE_BOOT_KEY_REPORT_UUID 0x2A4D
#define BLE_BOOT_KEY_REPORT_VALUE_HANDLE 0x003A
#define BLE_BOOT_MOUSE_REPORT_UUID 0x2A4D
#define BLE_BOOT_MOUSE_REPORT_VALUE_HANDLE 0x003E
#define BLE_Consumer_REPORT_UUID 0x2A4D
#define BLE_Consumer_REPORT_VALUE_HANDLE 0x0042
#define BLE_REPORT_MAP_UUID 0x2A4B
#define BLE_REPORT_MAP_VALUE_HANDLE 0x0046
#define BLE_BOOT_KEYBOARD_INPUT_REPORT_UUID 0x2A22
#define BLE_BOOT_KEYBOARD_INPUT_REPORT_VALUE_HANDLE 0x0049
#define BLE_BOOT_KEYBOARD_OUTPUT_REPORT_UUID 0x2A32
#define BLE_BOOT_KEYBOARD_OUTPUT_REPORT_VALUE_HANDLE 0x004C
#define BLE_BOOT_MOUSE_INPUT_REPORT_UUID 0x2A33
#define BLE_BOOT_MOUSE_INPUT_REPORT_VALUE_HANDLE 0x004E
#define BLE_HID_INFORMATION_UUID 0x2A4A
#define BLE_HID_INFORMATION_VALUE_HANDLE 0x0051
#define BLE_HID_CONTROL_POINT_UUID 0x2A4C
#define BLE_HID_CONTROL_POINT_VALUE_HANDLE 0x0053


static const uint8_t  _gatt_database_report_handle[] =
{
   0x0A,
   0x01, 0x00, 0x03, 0x00, 0x1F, 0x00, 0x20, 0x00, 0x00, 0x00, 
   0x0F, 0x18, 0x19, 0x2A, 0x26, 0x00, 0x27, 0x00, 0x00, 0x00, 
   0xE7, 0xFE, 0xC8, 0xFE, 0x2C, 0x00, 0x2D, 0x00, 0x00, 0x00, 
   0xE7, 0xFE, 0xA1, 0xFE, 0x31, 0x00, 0x32, 0x00, 0x00, 0x00, 
   0xE7, 0xFE, 0xA2, 0xFE, 0x34, 0x00, 0x35, 0x00, 0x00, 0x00, 
   0x12, 0x18, 0x4D, 0x2A, 0x3A, 0x00, 0x3B, 0x00, 0x00, 0x00, 
   0x12, 0x18, 0x4D, 0x2A, 0x3E, 0x00, 0x3F, 0x00, 0x00, 0x00, 
   0x12, 0x18, 0x4D, 0x2A, 0x42, 0x00, 0x43, 0x00, 0x00, 0x00, 
   0x12, 0x18, 0x22, 0x2A, 0x49, 0x00, 0x4A, 0x00, 
   0x12, 0x18, 0x33, 0x2A, 0x4E, 0x00, 0x4F, 0x00, 
};

static const uint8_t _gatt_database_include[] =
{
   0x0A, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00,
};

static const uint8_t _gatt_database_primary[] =
{
   0x09, 0x01, 0x00, 0x00, 0x28, 0x07, 0x00, 0x00, 0x18, //     0x0001  <Primary Service(0x2800)>  <GENERIC ACCESS(0x1800)>
   0x09, 0x08, 0x00, 0x00, 0x28, 0x0B, 0x00, 0x01, 0x18, //     0x0008  <Primary Service(0x2800)>  <Generic Attribute(0x1801)>
   0x09, 0x0C, 0x00, 0x00, 0x28, 0x1A, 0x00, 0x0A, 0x18, //     0x000C  <Primary Service(0x2800)>  <Device Information(0x180A)>
   0x09, 0x1B, 0x00, 0x00, 0x28, 0x20, 0x00, 0x01, 0x00, //     0x001B  <Primary Service(0x2800)>  <UART(0x0001)>
   0x09, 0x21, 0x00, 0x00, 0x28, 0x23, 0x00, 0x00, 0xFF, //     0x0021  <Primary Service(0x2800)>  <OTA(0xFF00)>
   0x09, 0x24, 0x00, 0x00, 0x28, 0x27, 0x00, 0x0F, 0x18, //     0x0024  <Primary Service(0x2800)>  <BATTERY_SERVICE(0x180F)>
   0x09, 0x28, 0x00, 0x00, 0x28, 0x35, 0x00, 0xE7, 0xFE, //     0x0028  <Primary Service(0x2800)>  <WECHAT(0xfee7)>
   0x09, 0x36, 0x00, 0x00, 0x28, 0xFF, 0xFF, 0x12, 0x18, //     0x0036  <Primary Service(0x2800)>  <HIDO_SERVICE(0x1812)>
};

static const uint8_t _gatt_database_characteristic[] =
{
   0x0A, 0x02, 0x00, 0x03, 0x28, 0x0A, 0x03, 0x00, 0x00, 0x2A, //     0x0002  <Characteristic(0x2803)>  <Properties:0x0A>  <Value Handle:0x0003>  <UUID:DEVICE NAME(0x2A00)>
   0x0A, 0x04, 0x00, 0x03, 0x28, 0x02, 0x05, 0x00, 0x01, 0x2A, //     0x0004  <Characteristic(0x2803)>  <Properties:0x02>  <Value Handle:0x0005>  <UUID:APPEARANCE(0x2A01)>
   0x0A, 0x06, 0x00, 0x03, 0x28, 0x02, 0x07, 0x00, 0x04, 0x2A, //     0x0006  <Characteristic(0x2803)>  <Properties:0x02>  <Value Handle:0x0007>  <UUID:Peripheral Preferred Connection Parameters(0x2A04)>
   0x0A, 0x09, 0x00, 0x03, 0x28, 0x02, 0x0A, 0x00, 0x05, 0x2A, //     0x0009  <Characteristic(0x2803)>  <Properties:0x02>  <Value Handle:0x000A>  <UUID:Service Changed(0x2A05)>
   0x0A, 0x0D, 0x00, 0x03, 0x28, 0x02, 0x0E, 0x00, 0x29, 0x2A, //     0x000D  <Characteristic(0x2803)>  <Properties:0x02>  <Value Handle:0x000E>  <UUID:MANUFACTURER_NAME_STRING(0x2A29)>
   0x0A, 0x0F, 0x00, 0x03, 0x28, 0x02, 0x10, 0x00, 0x24, 0x2A, //     0x000F  <Characteristic(0x2803)>  <Properties:0x02>  <Value Handle:0x0010>  <UUID:MODEL_NUMBER_STRING(0x2A24)>
   0x0A, 0x11, 0x00, 0x03, 0x28, 0x02, 0x12, 0x00, 0x25, 0x2A, //     0x0011  <Characteristic(0x2803)>  <Properties:0x02>  <Value Handle:0x0012>  <UUID:SERIAL_NUMBER_STRING(0x2A25)>
   0x0A, 0x13, 0x00, 0x03, 0x28, 0x02, 0x14, 0x00, 0x27, 0x2A, //     0x0013  <Characteristic(0x2803)>  <Properties:0x02>  <Value Handle:0x0014>  <UUID:HARDWARE_REVISION_STRING(0x2A27)>
   0x0A, 0x15, 0x00, 0x03, 0x28, 0x02, 0x16, 0x00, 0x26, 0x2A, //     0x0015  <Characteristic(0x2803)>  <Properties:0x02>  <Value Handle:0x0016>  <UUID:FIRMWARE_REVISION_STRING(0x2A26)>
   0x0A, 0x17, 0x00, 0x03, 0x28, 0x02, 0x18, 0x00, 0x28, 0x2A, //     0x0017  <Characteristic(0x2803)>  <Properties:0x02>  <Value Handle:0x0018>  <UUID:SOFTWARE_REVISION_STRING(0x2A28)>
   0x0A, 0x19, 0x00, 0x03, 0x28, 0x02, 0x1A, 0x00, 0x50, 0x2A, //     0x0019  <Characteristic(0x2803)>  <Properties:0x02>  <Value Handle:0x001A>  <UUID:PNP_ID(0x2A50)>
   0x0A, 0x1C, 0x00, 0x03, 0x28, 0x08, 0x1D, 0x00, 0x02, 0x00, //     0x001C  <Characteristic(0x2803)>  <Properties:0x08>  <Value Handle:0x001D>  <UUID:UART Write(0x0002)>
   0x0A, 0x1E, 0x00, 0x03, 0x28, 0x10, 0x1F, 0x00, 0x03, 0x00, //     0x001E  <Characteristic(0x2803)>  <Properties:0x10>  <Value Handle:0x001F>  <UUID:UART NOTIFY(0x0003)>
   0x0A, 0x22, 0x00, 0x03, 0x28, 0x0A, 0x23, 0x00, 0x01, 0xFF, //     0x0022  <Characteristic(0x2803)>  <Properties:0x0A>  <Value Handle:0x0023>  <UUID:OTA Read Write(0xFF01)>
   0x0A, 0x25, 0x00, 0x03, 0x28, 0x12, 0x26, 0x00, 0x19, 0x2A, //     0x0025  <Characteristic(0x2803)>  <Properties:0x12>  <Value Handle:0x0026>  <UUID:Battry Level(0x2A19)>
   0x0A, 0x29, 0x00, 0x03, 0x28, 0x08, 0x2A, 0x00, 0xC7, 0xFE, //     0x0029  <Characteristic(0x2803)>  <Properties:0x08>  <Value Handle:0x002A>  <UUID:WECHAT WRITE(0xFEC7)>
   0x0A, 0x2B, 0x00, 0x03, 0x28, 0x22, 0x2C, 0x00, 0xC8, 0xFE, //     0x002B  <Characteristic(0x2803)>  <Properties:0x22>  <Value Handle:0x002C>  <UUID:WECHAT Indication(0xFEC8)>
   0x0A, 0x2E, 0x00, 0x03, 0x28, 0x02, 0x2F, 0x00, 0xC9, 0xFE, //     0x002E  <Characteristic(0x2803)>  <Properties:0x02>  <Value Handle:0x002F>  <UUID:WECHAT Read(0xFEC9)>
   0x0A, 0x30, 0x00, 0x03, 0x28, 0x12, 0x31, 0x00, 0xA1, 0xFE, //     0x0030  <Characteristic(0x2803)>  <Properties:0x12>  <Value Handle:0x0031>  <UUID:Pedometer Measurement(0xFEA1)>
   0x0A, 0x33, 0x00, 0x03, 0x28, 0x2A, 0x34, 0x00, 0xA2, 0xFE, //     0x0033  <Characteristic(0x2803)>  <Properties:0x2A>  <Value Handle:0x0034>  <UUID:Sport Target(0xFEA2)>
   0x0A, 0x37, 0x00, 0x03, 0x28, 0x06, 0x38, 0x00, 0x4E, 0x2A, //     0x0037  <Characteristic(0x2803)>  <Properties:0x06>  <Value Handle:0x0038>  <UUID:PROTOCOL_MODE(0x2A4E)>
   0x0A, 0x39, 0x00, 0x03, 0x28, 0x12, 0x3A, 0x00, 0x4D, 0x2A, //     0x0039  <Characteristic(0x2803)>  <Properties:0x12>  <Value Handle:0x003A>  <UUID:BOOT_KEY_REPORT(0x2A4D)>
   0x0A, 0x3D, 0x00, 0x03, 0x28, 0x12, 0x3E, 0x00, 0x4D, 0x2A, //     0x003D  <Characteristic(0x2803)>  <Properties:0x12>  <Value Handle:0x003E>  <UUID:BOOT_MOUSE_REPORT(0x2A4D)>
   0x0A, 0x41, 0x00, 0x03, 0x28, 0x12, 0x42, 0x00, 0x4D, 0x2A, //     0x0041  <Characteristic(0x2803)>  <Properties:0x12>  <Value Handle:0x0042>  <UUID:Consumer_REPORT(0x2A4D)>
   0x0A, 0x45, 0x00, 0x03, 0x28, 0x02, 0x46, 0x00, 0x4B, 0x2A, //     0x0045  <Characteristic(0x2803)>  <Properties:0x02>  <Value Handle:0x0046>  <UUID:REPORT_MAP(0x2A4B)>
   0x0A, 0x48, 0x00, 0x03, 0x28, 0x12, 0x49, 0x00, 0x22, 0x2A, //     0x0048  <Characteristic(0x2803)>  <Properties:0x12>  <Value Handle:0x0049>  <UUID:BOOT_KEYBOARD_INPUT_REPORT(0x2A22)>
   0x0A, 0x4B, 0x00, 0x03, 0x28, 0x0E, 0x4C, 0x00, 0x32, 0x2A, //     0x004B  <Characteristic(0x2803)>  <Properties:0x0E>  <Value Handle:0x004C>  <UUID:BOOT_KEYBOARD_OUTPUT_REPORT(0x2A32)>
   0x0A, 0x4D, 0x00, 0x03, 0x28, 0x12, 0x4E, 0x00, 0x33, 0x2A, //     0x004D  <Characteristic(0x2803)>  <Properties:0x12>  <Value Handle:0x004E>  <UUID:BOOT_MOUSE_INPUT_REPORT(0x2A33)>
   0x0A, 0x50, 0x00, 0x03, 0x28, 0x02, 0x51, 0x00, 0x4A, 0x2A, //     0x0050  <Characteristic(0x2803)>  <Properties:0x02>  <Value Handle:0x0051>  <UUID:HID_INFORMATION(0x2A4A)>
   0x0A, 0x52, 0x00, 0x03, 0x28, 0x04, 0x53, 0x00, 0x4C, 0x2A, //     0x0052  <Characteristic(0x2803)>  <Properties:0x04>  <Value Handle:0x0053>  <UUID:HID_CONTROL_POINT(0x2A4C)>
   0x0A, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, //     End
};

static const uint8_t _gatt_database_value[] =
{
   0x14, 0x00, 0x03, 0x00, 0x10, 0x00, 0x2A, 0x53, 0x59, 0x44, 0x20, 0x42, 0x4C, 0x45, 0x20, 0x4D, 0x6F, 0x75, 0x73, 0x65, //     0x0003    <UUID:DEVICE NAME(0x2A00)>  <Value:Name(SYD BLE Mouse)>
   0x09, 0x00, 0x05, 0x00, 0x10, 0x01, 0x2A, 0xC2, 0x03, //     0x0005    <UUID:APPEARANCE(0x2A01)>  <Value:Category(0x03 0xC2)>
   0x0F, 0x00, 0x07, 0x00, 0x10, 0x04, 0x2A, 0x06, 0x00, 0x06, 0x00, 0x30, 0x00, 0x64, 0x00, //     0x0007    <UUID:Peripheral Preferred Connection Parameters(0x2A04)>  <Value:Minimum Connection Interval(0x0006), Maximum Connection Interval(0x0006), Slave Latency(0x0030), Connection Supervision Timeout Multiplier(0x0064)>
   0x0B, 0x00, 0x0A, 0x00, 0x10, 0x05, 0x2A, 0x00, 0x00, 0x00, 0x00, //     0x000A    <UUID:Service Changed(0x2A05)>  <Value:Start of Affected Attribute Handle Range(0x0000), End of Affected Attribute Handle Range(0x0000)>
   0x09, 0x00, 0x0B, 0x00, 0x10, 0x02, 0x29, 0x00, 0x00, //     0x000B    <UUID:Client Characteristic Configuration(0x2902)>  <Value:(Properties(0x00 0x00))>
   0x0A, 0x00, 0x0E, 0x00, 0x10, 0x29, 0x2A, 0x53, 0x59, 0x44, //     0x000E    <UUID:MANUFACTURER_NAME_STRING(0x2A29)>  <Value:Manufacturer Name(SYD)>
   0x14, 0x00, 0x10, 0x00, 0x10, 0x24, 0x2A, 0x42, 0x4C, 0x45, 0x2D, 0x4D, 0x4F, 0x44, 0x45, 0x4C, 0x2D, 0x30, 0x30, 0x31, //     0x0010    <UUID:MODEL_NUMBER_STRING(0x2A24)>  <Value:Model Number(BLE-MODEL-001)>
   0x0E, 0x00, 0x12, 0x00, 0x10, 0x25, 0x2A, 0x42, 0x4C, 0x45, 0x2D, 0x30, 0x30, 0x31, //     0x0012    <UUID:SERIAL_NUMBER_STRING(0x2A25)>  <Value:Serial Number(BLE-001)>
   0x0A, 0x00, 0x14, 0x00, 0x10, 0x27, 0x2A, 0x31, 0x2E, 0x30, //     0x0014    <UUID:HARDWARE_REVISION_STRING(0x2A27)>  <Value:Hardware Revision(1.0)>
   0x0A, 0x00, 0x16, 0x00, 0x10, 0x26, 0x2A, 0x31, 0x2E, 0x30, //     0x0016    <UUID:FIRMWARE_REVISION_STRING(0x2A26)>  <Value:Firmware Revision(1.0)>
   0x0A, 0x00, 0x18, 0x00, 0x10, 0x28, 0x2A, 0x31, 0x2E, 0x30, //     0x0018    <UUID:SOFTWARE_REVISION_STRING(0x2A28)>  <Value:Software Revision(1.0)>
   0x0E, 0x00, 0x1A, 0x00, 0x10, 0x50, 0x2A, 0x02, 0x3A, 0x09, 0x54, 0x42, 0x01, 0x00, //     0x001A    <UUID:PNP_ID(0x2A50)>  <Value:Vendor ID Source(0x02), Vendor ID(0x093A), Product ID(0x4254), Product Version(0x0001)>
   0x1C, 0x00, 0x1D, 0x00, 0x10, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //     0x001D    <UUID:UART Write(0x0002)>  <Value:Vender Define Value(0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00)>
   0x1C, 0x00, 0x1F, 0x00, 0x10, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //     0x001F    <UUID:UART NOTIFY(0x0003)>  <Value:Vender Define Value(0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00)>
   0x09, 0x00, 0x20, 0x00, 0x10, 0x02, 0x29, 0x00, 0x00, //     0x0020    <UUID:CLIENT_CHARACTERISTIC_CONFIGURATION(0x2902)>  <Value:(Properties(0x00 0x00))>
   0x1C, 0x00, 0x23, 0x00, 0x10, 0x01, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //     0x0023    <UUID:OTA Read Write(0xFF01)>  <Value:Vender Define Value(0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00)>
   0x08, 0x00, 0x26, 0x00, 0x10, 0x19, 0x2A, 0x64, //     0x0026    <UUID:Battry Level(0x2A19)>  <Value:Level(0x64)>
   0x09, 0x00, 0x27, 0x00, 0x10, 0x02, 0x29, 0x00, 0x00, //     0x0027    <UUID:CLIENT_CHARACTERISTIC_CONFIGURATION(0x2902)>  <Value:(Properties(0x00 0x00))>
   0x1C, 0x00, 0x2A, 0x00, 0x10, 0xC7, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //     0x002A    <UUID:WECHAT WRITE(0xFEC7)>  <Value:Vender Define Value(0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00)>
   0x1C, 0x00, 0x2C, 0x00, 0x10, 0xC8, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //     0x002C    <UUID:WECHAT Indication(0xFEC8)>  <Value:Vender Define Value(0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00)>
   0x09, 0x00, 0x2D, 0x00, 0x10, 0x02, 0x29, 0x00, 0x00, //     0x002D    <UUID:CLIENT_CHARACTERISTIC_CONFIGURATION(0x2902)>  <Value:(Properties(0x00 0x00))>
   0x1C, 0x00, 0x2F, 0x00, 0x10, 0xC9, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //     0x002F    <UUID:WECHAT Read(0xFEC9)>  <Value:Vender Define Value(0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00)>
   0x1C, 0x00, 0x31, 0x00, 0x10, 0xA1, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //     0x0031    <UUID:Pedometer Measurement(0xFEA1)>  <Value:Vender Define Value(0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00)>
   0x09, 0x00, 0x32, 0x00, 0x10, 0x02, 0x29, 0x00, 0x00, //     0x0032    <UUID:CLIENT_CHARACTERISTIC_CONFIGURATION(0x2902)>  <Value:(Properties(0x00 0x00))>
   0x1C, 0x00, 0x34, 0x00, 0x10, 0xA2, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //     0x0034    <UUID:Sport Target(0xFEA2)>  <Value:Vender Define Value(0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00)>
   0x09, 0x00, 0x35, 0x00, 0x10, 0x02, 0x29, 0x00, 0x00, //     0x0035    <UUID:CLIENT_CHARACTERISTIC_CONFIGURATION(0x2902)>  <Value:(Properties(0x00 0x00))>
   0x08, 0x00, 0x38, 0x00, 0x10, 0x4E, 0x2A, 0x01, //     0x0038    <UUID:PROTOCOL_MODE(0x2A4E)>  <Value:Protocol Mode Value(0x01)>
   0x0F, 0x00, 0x3A, 0x00, 0x70, 0x4D, 0x2A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //     0x003A    <UUID:BOOT_KEY_REPORT(0x2A4D)>  <Value:Report Value(0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00)>
   0x09, 0x00, 0x3B, 0x00, 0x10, 0x02, 0x29, 0x00, 0x00, //     0x003B    <UUID:CLIENT_CHARACTERISTIC_CONFIGURATION(0x2902)>  <Value:(Properties(0x00 0x00))>
   0x09, 0x00, 0x3C, 0x00, 0x10, 0x08, 0x29, 0x01, 0x01, //     0x003C    <UUID:REPORT_REFERENCE(0x2908)>  <Value:(Report ID(0x01), Report Type(0x01))>
   0x0F, 0x00, 0x3E, 0x00, 0x70, 0x4D, 0x2A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //     0x003E    <UUID:BOOT_MOUSE_REPORT(0x2A4D)>  <Value:Report Value(0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00)>
   0x09, 0x00, 0x3F, 0x00, 0x10, 0x02, 0x29, 0x00, 0x00, //     0x003F    <UUID:CLIENT_CHARACTERISTIC_CONFIGURATION(0x2902)>  <Value:(Properties(0x00 0x00))>
   0x09, 0x00, 0x40, 0x00, 0x10, 0x08, 0x29, 0x02, 0x01, //     0x0040    <UUID:REPORT_REFERENCE(0x2908)>  <Value:(Report ID(0x02), Report Type(0x01))>
   0x0F, 0x00, 0x42, 0x00, 0x70, 0x4D, 0x2A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //     0x0042    <UUID:Consumer_REPORT(0x2A4D)>  <Value:Report Value(0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00)>
   0x09, 0x00, 0x43, 0x00, 0x10, 0x02, 0x29, 0x00, 0x00, //     0x0043    <UUID:CLIENT_CHARACTERISTIC_CONFIGURATION(0x2902)>  <Value:(Properties(0x00 0x00))>
   0x09, 0x00, 0x44, 0x00, 0x10, 0x08, 0x29, 0x03, 0x01, //     0x0044    <UUID:REPORT_REFERENCE(0x2908)>  <Value:(Report ID(0x03), Report Type(0x01))>
   0xA6, 0x00, 0x46, 0x00, 0x70, 0x4B, 0x2A, 0x05, 0x01, 0x09, 0x06, 0xa1, 0x01, 0x85, 0x01, 0x05, 0x07, 0x19, 0xE0, 0x29, 0xE7, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x08, 0x81, 0x02, 0x95, 0x01, 0x75, 0x08, 0x81, 0x01, 0x95, 0x05, 0x75, 0x01, 0x05, 0x08, 0x19, 0x01, 0x29, 0x05, 0x91, 0x02, 0x95, 0x01, 0x75, 0x03, 0x91, 0x01, 0x95, 0x06, 0x75, 0x08, 0x15, 0x00, 0x25, 0x65, 0x05, 0x07, 0x19, 0x00, 0x29, 0x65, 0x81, 0x00, 0xc0, 0x05, 0x01, 0x09, 0x02, 0xa1, 0x01, 0x85, 0x02, 0x09, 0x01, 0xa1, 0x00, 0x05, 0x09, 0x19, 0x01, 0x29, 0x08, 0x15, 0x00, 0x25, 0x01, 0x95, 0x08, 0x75, 0x01, 0x81, 0x02, 0x05, 0x01, 0x16, 0x01, 0xf8, 0x26, 0xff, 0x07, 0x75, 0x0c, 0x95, 0x02, 0x09, 0x30, 0x09, 0x31, 0x81, 0x06, 0x15, 0x81, 0x25, 0x7F, 0x75, 0x08, 0x95, 0x01, 0x09, 0x38, 0x81, 0x06, 0x05, 0x0c, 0x0A, 0x38, 0x02, 0x95, 0x01, 0x81, 0x06, 0xc0, 0xc0, 0x05, 0x0C, 0x09, 0x01, 0xA1, 0x01, 0x85, 0x03, 0x19, 0x00, 0x2A, 0x9c, 0x02, 0x15, 0x00, 0x26, 0x9c, 0x02, 0x75, 0x10, 0x95, 0x01, 0x81, 0x00, 0xC0, //     0x0046    <UUID:REPORT_MAP(0x2A4B)>  <Value:Report Map Value(0x05,0x01,0x09,0x06,0xa1,0x01,0x85,0x01,0x05,0x07,0x19,0xE0,0x29,0xE7,0x15,0x00,0x25,0x01,0x75,0x01,0x95,0x08,0x81,0x02,0x95,0x01,0x75,0x08,0x81,0x01,0x95,0x05,0x75,0x01,0x05,0x08,0x19,0x01,0x29,0x05,0x91,0x02,0x95,0x01,0x75,0x03,0x91,0x01,0x95,0x06,0x75,0x08,0x15,0x00,0x25,0x65,0x05,0x07,0x19,0x00,0x29,0x65,0x81,0x00,0xc0,0x05,0x01,0x09,0x02,0xa1,0x01,0x85,0x02,0x09,0x01,0xa1,0x00,0x05,0x09,0x19,0x01,0x29,0x08,0x15,0x00,0x25,0x01,0x95,0x08,0x75,0x01,0x81,0x02,0x05,0x01,0x16,0x01,0xf8,0x26,0xff,0x07,0x75,0x0c,0x95,0x02,0x09,0x30,0x09,0x31,0x81,0x06,0x15,0x81,0x25,0x7F,0x75,0x08,0x95,0x01,0x09,0x38,0x81,0x06,0x05,0x0c,0x0A,0x38,0x02,0x95,0x01,0x81,0x06,0xc0,0xc0,0x05,0x0C,0x09,0x01,0xA1,0x01,0x85,0x03,0x19,0x00,0x2A,0x9c,0x02,0x15,0x00,0x26,0x9c,0x02,0x75,0x10,0x95,0x01,0x81,0x00,0xC0)>
   0x07, 0x00, 0x47, 0x00, 0x10, 0x07, 0x29, //     0x0047    <UUID:EXTERNAL_REPORT_REFERENCE(0x2907)>  <Value:(External Report Reference())>
   0x08, 0x00, 0x49, 0x00, 0x10, 0x22, 0x2A, 0x00, //     0x0049    <UUID:BOOT_KEYBOARD_INPUT_REPORT(0x2A22)>  <Value:Boot Keyboard Input Report Value(0x00)>
   0x07, 0x00, 0x4A, 0x00, 0x10, 0x02, 0x29, //     0x004A    <UUID:GATT.CLIENT_CHARACTERISTIC_CONFIGURATION(0x2902)>  <Value:(Properties())>
   0x08, 0x00, 0x4C, 0x00, 0x10, 0x32, 0x2A, 0x00, //     0x004C    <UUID:BOOT_KEYBOARD_OUTPUT_REPORT(0x2A32)>  <Value:Boot Keyboard Output Report Value(0x00)>
   0x0B, 0x00, 0x4E, 0x00, 0x10, 0x33, 0x2A, 0x00, 0x00, 0x00, 0x00, //     0x004E    <UUID:BOOT_MOUSE_INPUT_REPORT(0x2A33)>  <Value:Boot Mouse Input Report Value(0x00, 0x00, 0x00, 0x00)>
   0x07, 0x00, 0x4F, 0x00, 0x10, 0x02, 0x29, //     0x004F    <UUID:GATT.CLIENT_CHARACTERISTIC_CONFIGURATION(0x2902)>  <Value:(Properties())>
   0x0B, 0x00, 0x51, 0x00, 0x10, 0x4A, 0x2A, 0x11, 0x01, 0x00, 0x03, //     0x0051    <UUID:HID_INFORMATION(0x2A4A)>  <Value:bcdHID(0x0111), bCountryCode(0x00), Flags(0x03)>
   0x08, 0x00, 0x53, 0x00, 0x10, 0x4C, 0x2A, 0x00, //     0x0053    <UUID:HID_CONTROL_POINT(0x2A4C)>  <Value:HID Control Point Command(0x00)>
   0xFF, 0xFF, 0xFF, 0xFF, //     End
};

#endif
