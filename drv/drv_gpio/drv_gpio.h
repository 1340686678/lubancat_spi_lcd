#ifndef __DRV_GPIO_H
#define __DRV_GPIO_H

#include <gpiod.h>
#include <stdint.h>

typedef struct
{
	char name[10];
	struct gpiod_chip* chip;	//GPIO控制器句柄
	struct gpiod_line* line;	//GPIO引脚句柄
}M_GPIO_INFO_T;


typedef enum
{
	GPIO_OFF = GPIOD_LINE_ACTIVE_STATE_LOW,
	GPIO_ON = GPIOD_LINE_ACTIVE_STATE_HIGH
}M_GPIO_STATUS_T;

typedef enum
{
	GPIO_INPUT = GPIOD_LINE_DIRECTION_INPUT,
	GPIO_OUTPUT = GPIOD_LINE_DIRECTION_OUTPUT
}M_GPIO_WORK_MODE_T;

typedef struct
{
	char name[10];
	uint8_t port;	//GPIO1_B3 port=1
	uint8_t pin;	//GPIO1_B3 pin=1*8+3
	M_GPIO_WORK_MODE_T mode;
	M_GPIO_STATUS_T status;
}M_GPIO_INIT_PARAM_T;

int drv_gpio_get(const M_GPIO_INIT_PARAM_T param, M_GPIO_INFO_T* info);
void drv_gpio_release(M_GPIO_INFO_T* info);

int drv_gpio_set_status(const M_GPIO_INFO_T* info, const M_GPIO_STATUS_T status);

#endif