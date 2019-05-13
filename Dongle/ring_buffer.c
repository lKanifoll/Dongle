#include "ring_buffer.h"

void ring_buff_push_byte(ring_buff_t *buff, uint8_t byte)
{
	buff->ring_buff[buff->end_ptr] = byte;
	buff->end_ptr++;
	if (buff->end_ptr >= buff->buff_size)
	{
		buff->end_ptr = 0;
	}
}


uint8_t ring_buff_pull_byte(ring_buff_t *buff)
{
	uint8_t byte;
	byte = buff->ring_buff[buff->start_ptr];
	buff->start_ptr++;
	if (buff->start_ptr >= buff->buff_size)
	{
		buff->start_ptr = 0;
	}
	return byte;
}


void ring_buff_init(ring_buff_t *buff, uint16_t size)
{
	buff->buff_size = size;
	buff->ring_buff = (uint8_t*)malloc(size);
	buff->start_ptr = 0;
	buff->end_ptr   = 0;
}


void ring_buff_push_data(ring_buff_t *buff, uint8_t* data, uint16_t size)
{
	while (size--)
	{
		ring_buff_push_byte(buff, *(data++));
	}
}


void ring_buff_pull_data(ring_buff_t *buff, uint8_t* data, uint16_t start_ptr, uint16_t size)
{
	data += start_ptr;
	while (size--)
	{
		*(data++) = ring_buff_pull_byte(buff);
	}
}
