#include "drv_init.h"
#include "drv_lcd/drv_lcd.h"
#include "drv_lcd/drv_lcd_touch.h"

#include "common/log.h"

#include <stdio.h>

/*-1初始化失败 0初始化成功*/
typedef int (*init_func)(void);

/***************************************************************
 * Name:	 drv_init()
 * Input : void
 * Output: void
 * Return: -1:失败 0:成功
 * Author: hwl
 * Revise: V1.0
 * Description: 驱动初始化
 ***************************************************************/
int drv_init(void)
{
	static struct init_drv_list
	{
		char name[50];
		init_func func;
	} init_list_t[] = {
		{
			.name = "SPI_LCD",
			.func = drv_lcd_init
		},
		{
			.name = "I2C_LCD_TOUCH",
			.func = drv_lcd_touch_init
		},
	};

	for(int i = 0; i < sizeof(init_list_t) / sizeof(struct init_drv_list); i++)
	{
		if(init_list_t[i].func() == -1)
		{
			LOG_WARNING("DRV %s init fail\r\n", init_list_t[i].name);
			return -1;
		}
		else
		{
			LOG_DEBUG("DRV %s init true\r\n", init_list_t[i].name);
		}
	}

	return 0;
	
}