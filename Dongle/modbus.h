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
	
#define BEGIN_MODBUS_REG				0x0100
#define END_MODBUS_REG					0x0126
	
uint8_t modbus_slave_address;
uint8_t modbus_rx_complete;
uint8_t modbus_tx_complete;
uint8_t modbus_raw_byte_count;
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

void modbus_handler();
	
#ifdef __cplusplus
}
#endif
	
	
#endif 