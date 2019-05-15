#ifndef __MODBUS_H
#define __MODBUS_H

#ifdef __cplusplus
extern "C" {
#endif
	
#include "main.h"
#define MODBUS_READ_HOLDING_REG			0x03
#define MODBUS_READ_INPUT_REG			0x04
#define MODBUS_WRITE_SINGLE_REG			0x06
#define MODBUS_WRITE_MULTIPLY_REG		0x10
#define MODBUS_WRITE_EEPROM				0x43
	
#define BEGIN_MODBUS_REG				0x0000
#define END_MODBUS_REG					0x0026
	
#define reg_UUID						0x0001
#define reg_UDID						0x0004
#define reg_SETTINGS					0x000C
#define reg_DATE						0x0012
#define reg_WEEK_PTS					0x0016
#define reg_CUSTOM_DAY_PTS				0x001A
	
#define count_UUID						3
#define count_UDID						8
#define count_SETTINGS					6
#define count_DATE						4
#define count_WEEK_PTS					4
#define count_CUSTOM_DAY_PTS			12
	
	
extern uint8_t read_UUID[4];
extern uint8_t read_UDID[4];
extern uint8_t read_SETTINGS[4];
extern uint8_t read_DATE[4];	
extern uint8_t read_WEEK_PTS[4];
extern uint8_t read_CUSTOM_DAY_PTS[4];
	
extern uint8_t modbus_slave_address;
extern uint8_t modbus_rx_complete;
extern uint8_t modbus_tx_complete;
extern uint8_t modbus_raw_byte_count;
extern uint16_t mb_CRC;
//------------------------------------------------------------------ for modbus rx 
	
typedef struct 
{
	uint8_t		address;
	uint8_t		function;
	uint16_t	start_reg_address;
	uint16_t	reg_count;
	uint8_t		data_buff[BUFFER_SIZE];
} modbus_rx_frame_t;	
	

typedef union 
{
	modbus_rx_frame_t		modbus_frame;
	uint8_t					modbus_master_frame[BUFFER_SIZE];
} mb_rx_union_u; 

mb_rx_union_u modbus_raw;

//------------------------------------------------------------------ for modbus tx 03
typedef struct 
{
	uint8_t		address;
	uint8_t		function;
	uint8_t		byte_count;   // __builtin_bswap16(16bit);
	uint8_t		data_buff[BUFFER_SIZE]; 
} modbus_03_tx_frame_t; 

typedef union 
{
	modbus_03_tx_frame_t    modbus_frame;
	uint8_t					modbus_slave_frame[BUFFER_SIZE];
} mb_03_tx_union_u; 

mb_03_tx_union_u modbus_03_raw;
	
//------------------------------------------------------------------ for modbus tx 10
	
typedef struct 
{
	uint8_t		address;
	uint8_t		function;
	uint16_t	start_reg_address;
	uint16_t	reg_count;
	uint16_t	mb_CRC;
} modbus_10_tx_frame_t; 

typedef union 
{
	modbus_10_tx_frame_t    modbus_frame;
	uint8_t					modbus_slave_frame[8];
} mb_10_tx_union_u; 

mb_10_tx_union_u modbus_10_raw;
//------------------------------------------------------------------


uint8_t checksum8(uint8_t * buff, uint8_t size);
uint16_t modbus_rtu_calc_crc(const void* data, size_t size);


	
#ifdef __cplusplus
}
#endif
	
	
#endif 