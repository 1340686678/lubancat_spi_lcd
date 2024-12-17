#include <gpiod.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

#include "drv/drv_gpio/drv_gpio.h"

M_GPIO_INFO_T info;
int init_gpio(void)
{
	M_GPIO_INIT_PARAM_T param = {
		.mode = GPIO_OUTPUT,
		.name = "GPIO1",
		.pin = 3,
		.port = 4,
		.status = GPIO_ON
	};

	int ret = drv_gpio_get(param, &info);
	if(ret < 0)
	{
		printf("gpio init fail\r\n");
	}
	else
	{
		printf("gpio init true\r\n");
	}
}

void main(void)
{
	printf("SPI_LCD dev demo\r\n");
	init_gpio();
	uint8_t i = 20;
	while (i--)
	{
		drv_gpio_set_status(&info, 1);
		usleep(2000000);	//延时2s
		drv_gpio_set_status(&info, 0);
		usleep(2000000);	//延时2s
	}

	drv_gpio_release(&info);
}