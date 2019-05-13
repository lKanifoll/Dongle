#include "flash.h"

HAL_StatusTypeDef FLASH_Init()
{
	HAL_StatusTypeDef flash_err;
	EraseInitStruct.TypeErase	= FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress = FLASH_ADDRESS;
	EraseInitStruct.NbPages		= 1;
	
	HAL_FLASH_Unlock();
	flash_err = HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
	HAL_FLASH_Lock();
	
	return flash_err;
}

uint32_t FLASH_Read(uint32_t address)
{
	return (*(__IO uint32_t*)address);
}

HAL_StatusTypeDef FLASH_Write(uint32_t address, uint32_t* data)
{
	HAL_StatusTypeDef flash_err;
	return flash_err;
}

