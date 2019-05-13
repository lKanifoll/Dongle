#ifndef __RING_BUFFER_H
#define __RING_BUFFER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
	
	typedef struct
	{
		uint8_t *ring_buff;
		uint16_t start_ptr;
		uint16_t end_ptr;
		uint16_t buff_size;
	} ring_buff_t;	
	
	void ring_buff_init(ring_buff_t *buff, uint16_t size);
	void ring_buff_clear(ring_buff_t *buff);
	void ring_buff_push_byte(ring_buff_t *buff, uint8_t byte);
	void ring_buff_push_data(ring_buff_t *buff, uint8_t* data, uint16_t size);
	void ring_buff_pull_data(ring_buff_t *buff, uint8_t* data, uint16_t start_ptr, uint16_t size);
	uint8_t ring_buff_pull_byte(ring_buff_t *buff);


#ifdef __cplusplus
}
#endif

#endif 