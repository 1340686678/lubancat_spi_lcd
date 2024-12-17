#include "drv_gpio.h"
#include <string.h>
#include <stdio.h>

/***************************************************************
 * Name:	 drv_gpio_get()
 * Input : param:初始状态
 * Output: info:IO信息
 * Return: -1 失败 0正常
 * Author: hwl
 * Revise: V1.0
 * Description: 获取一个io
 ***************************************************************/
int drv_gpio_get(const M_GPIO_INIT_PARAM_T param, M_GPIO_INFO_T* info)
{
	/*保存名字*/
	memcpy(info->name, param.name, sizeof(param.name));

	/*获取GPIO控制器*/
	printf(" gpiod_chip_open \r\n");
	char path[20];
	sprintf(path, "/dev/gpiochip%d", param.port);
	info->chip = gpiod_chip_open(path);
	if(info->chip == NULL)
	{
			printf("gpio%d gpiod_chip_open error\n", param.port);
			return -1;
	}

	int ret = 0;

	/*获取GPIO引脚*/
	printf(" gpiod_chip_get_line \r\n");
	info->line = gpiod_chip_get_line(info->chip, param.pin);
	if(info->line == NULL)
	{
		ret = -1;
		printf("gpiod_chip_get_line error\n");
		goto release_chip;
	}

	/*配置GPIO工作模式*/
	printf(" gpiod_line_request_output \r\n");
	if(param.mode == GPIO_OUTPUT)
	{
		/*设置GPIO为输出模式*/
		int ret = gpiod_line_request_output(info->line, info->name, param.status);
		if(ret < 0)
		{
			printf("gpiod_line_request_output error\n");
			goto release_line;
		}
	}
	else
	{

	}

	return 0;

release_line:
	/*释放GPIO引脚*/
	gpiod_line_release(info->line);
	info->line = NULL;
release_chip:
	/*释放GPIO控制器*/
	gpiod_chip_close(info->chip);
	info->chip = NULL;

	return ret;
}

/***************************************************************
 * Name:	 drv_gpio_release()
 * Input : info:IO信息
 * Output: info 初始化后得到的数据
 * Return: void
 * Author: hwl
 * Revise: V1.0
 * Description: 释放一个IO
 ***************************************************************/
void drv_gpio_release(M_GPIO_INFO_T* info)
{
	/*释放GPIO引脚*/
	if(info->line != NULL)
	{
		gpiod_line_release(info->line);
		info->line = NULL;
	}

	/*释放GPIO控制器*/
	if(info->chip != NULL)
	{
		gpiod_chip_close(info->chip);
		info->chip = NULL;
	}
}

/***************************************************************
 * Name:	 drv_gpio_set_status()
 * Input : info:IO信息 status:IO状态
 * Output: void
 * Return: void
 * Author: hwl
 * Revise: V1.0
 * Description: 设置IO输出
 ***************************************************************/
int drv_gpio_set_status(const M_GPIO_INFO_T* info, const M_GPIO_STATUS_T status)
{
	return gpiod_line_set_value(info->line, status == GPIO_ON ? 1 : 0);
}