#include "flash.h"

uint8_t FLASH_ReadByte(uint32_t addr)
{
    return *(__IO uint8_t*)addr;
}

void FLASH_WriteByte(uint32_t addr, uint8_t data)
{
    HAL_FLASH_Unlock();
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, addr, data);
    HAL_FLASH_Lock();
}

void FLASH_WriteInt16Array(uint32_t addr, int16_t *data, uint8_t len)
{
    HAL_FLASH_Unlock();
    for(uint8_t i=0; i<len; i++)
    {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr+i*2, data[i]);
    }
    HAL_FLASH_Lock();
}

void FLASH_ReadInt16Array(uint32_t addr, int16_t *data, uint8_t len)
{
    uint8_t i;
    for(i=0; i<len; i++)
    {
        data[i] = *(__IO int16_t*)(addr+i*2);
    }
}
static uint32_t FLASH_GetSector(uint32_t addr)
{
    if(addr < 0x08004000U) return FLASH_SECTOR_0;
    if(addr < 0x08008000U) return FLASH_SECTOR_1;
    if(addr < 0x0800C000U) return FLASH_SECTOR_2;
    if(addr < 0x08010000U) return FLASH_SECTOR_3;
    if(addr < 0x08020000U) return FLASH_SECTOR_4;
    return FLASH_SECTOR_5;
}

void FLASH_EraseSectorByAddr(uint32_t addr)
{
    FLASH_EraseInitTypeDef erase;
    uint32_t sector_error = 0;

    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Sector = FLASH_GetSector(addr);
    erase.NbSectors = 1;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    HAL_FLASH_Unlock();
    HAL_FLASHEx_Erase(&erase, &sector_error);
    HAL_FLASH_Lock();
}

void FLASH_WriteFloat(uint32_t addr, float value)
{
    uint32_t data = *((uint32_t*)&value);
    HAL_FLASH_Unlock();
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, data);
    HAL_FLASH_Lock();
}

float FLASH_ReadFloat(uint32_t addr)
{
    uint32_t data = *(__IO uint32_t*)addr;
    return *((float*)&data);
}