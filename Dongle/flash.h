#ifndef __FLASH_H
#define __FLASH_H

#include "main.h"

uint32_t				ModBus_addr;
uint32_t				tmp;
uint32_t				FLASH_ADDRESS;   // 16 page, last
uint32_t				PageError;
HAL_StatusTypeDef		flash_err;
FLASH_EraseInitTypeDef	EraseInitStruct;


HAL_StatusTypeDef		FLASH_Init();
uint32_t				FLASH_Read(uint32_t address);
HAL_StatusTypeDef		FLASH_Write(uint32_t address, uint32_t* data);

#endif 