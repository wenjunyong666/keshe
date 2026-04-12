#ifndef __FLASH_H
#define __FLASH_H

#include "stm32f4xx_hal.h"

void FLASH_WriteByte(uint32_t addr, uint8_t data);
uint8_t FLASH_ReadByte(uint32_t addr);
void FLASH_WriteInt16Array(uint32_t addr, int16_t *data, uint8_t len);
void FLASH_ReadInt16Array(uint32_t addr, int16_t *data, uint8_t len);
void FLASH_EraseSectorByAddr(uint32_t addr);

#endif