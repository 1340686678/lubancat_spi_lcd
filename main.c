#include "./drv/drv_init.h"
#include "./drv/drv_lcd/drv_lcd.h"
#include <stdio.h>

void main(void)
{
	printf("SPI_LCD dev demo\r\n");
	drv_init();
}