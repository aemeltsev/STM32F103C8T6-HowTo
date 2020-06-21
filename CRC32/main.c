#include <stdio.h>
#include <stdint.h>
#include "stm32f10x.h"
#include "CRC32.h"

int main(void){
	uint32_t tmp = '0x0a';
	printf("%14d/n", tmp);
	printf("%14d/n", revbit(tmp));
	while(1){
		
	}
}