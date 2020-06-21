#include "CRC32.h"

uint32_t HWCRC32Calc(uint8_t *buffer, size_t count, uint32_t init)
{
	uint32_t crc;
	uint32_t *tmp = (uint32_t*)buffer;
	
	RCC->AHBENR |= RCC_AHBENR_CRCEN;

	size_t count32 = count >> 2;
	if(0xFFFFFFFF == init)
	{
		CRC->CR |= CRC_CR_RESET;
	}
	while(count32--)
	{
		CRC->DR = __rbit(*tmp++);
	}
	
	crc = __rbit((uint32_t)CRC->DR);
	count = count % 4;
	
	if(count)
	{
		CRC->DR = __rbit(crc);
		switch(count)
		{
			case 1:
				CRC->DR = __rbit((*tmp & 0x000000FF) ^ crc) >> 24;
			  crc = (crc >> 8) ^ __rbit(CRC->DR);
			  break;
			case 2:
				CRC->DR = __rbit((*tmp & 0x0000FFFF) ^ crc) >> 16;
			  crc = (crc >> 16) ^ __rbit(CRC->DR);
			  break;
			case 3:
				CRC->DR = __rbit((*tmp & 0x00FFFFFF) ^ crc) >> 8;
			  crc = (crc >> 24) ^ __rbit(CRC->DR);
			  break;
		}
	}
	return ~crc;
}