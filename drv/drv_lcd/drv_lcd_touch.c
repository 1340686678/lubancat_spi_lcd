#include "drv_lcd_touch.h"

#include "../../common/log.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

static int g_touch_fd;

#define TOUCH_FILE_PATH "/dev/i2c-5"
#define TOUCH_ADDR 0x38

#define TOUCH_POS_COUNT			0x02

#define TOUCH_POS_1					0x03
#define TOUCH_POS_2					0x09
#define TOUCH_POS_DATA_LEN	4

static int i2c_write(uint8_t reg,uint8_t val)
{
	if(g_touch_fd == 0)
	{
		LOG_WARNING("touch fd == NULL\r\n");
		return -1;
	}

	uint8_t data[2] = {reg, val};

	if (write(g_touch_fd, data, 2) == 2)
	{
			return 0;
	}
	else{
			return -1;
	}
}

/***************************************************************
 * Name:	 touch_read()
 * Input : reg:寄存器地址 len:读取长度
 * Output: val:读取到的值
 * Return: -1:失败 0:成功
 * Author: hwl
 * Revise: V1.0
 * Description: 触控SOC的寄存器读取
 ***************************************************************/
static int touch_read(const uint8_t reg, uint8_t * val, const size_t len)
{
	if(g_touch_fd == 0)
	{
		LOG_WARNING("touch fd == NULL\r\n");
		return -1;
	}

	if (write(g_touch_fd, &reg, 1) == 1)
	{
		if (read(g_touch_fd, val, len) == len)
		{
			return 0;
		}
	}
	else
	{
		return -1;
	}
}

typedef struct 
{
	uint8_t vendor_id;	//0xA8
	uint8_t soc_id[3];	//0x9F 0xA0 0xA3
}touch_soc_id;

/***************************************************************
 * Name:	 touch_get_id()
 * Input : void
 * Output: id:读取到的ID
 * Return: void
 * Author: hwl
 * Revise: V1.0
 * Description: 获取触控SOC的ID
 ***************************************************************/
static void touch_get_id(touch_soc_id* id)
{
	touch_read(0xA8, &(id->vendor_id), 1);

	touch_read(0xA0, &(id->soc_id[0]), 1);
	touch_read(0x9F, &(id->soc_id[1]), 1);
	touch_read(0xA3, &(id->soc_id[2]), 1);
}

/***************************************************************
 * Name:	 touch_id_checK()
 * Input : id:读取到的ID
 * Output: void
 * Return: 0:校验通过 -1:未通过
 * Author: hwl
 * Revise: V1.0
 * Description: 校验触控SOC的ID
 ***************************************************************/
static int touch_id_checK(touch_soc_id id)
{
	if(	id.vendor_id != 0x11 ||
			id.soc_id[0] > 0x03 ||
			id.soc_id[1] != 0x26 ||
			id.soc_id[2] != 0x64)
	{
		return -1;
	}
	else
	{
		return 0;
	}
}

/***************************************************************
 * Name:	 drv_lcd_touch_init()
 * Input : void
 * Output: void
 * Return: 0:初始成功 -1:初始失败
 * Author: hwl
 * Revise: V1.0
 * Description: 触控SOC初始化
 ***************************************************************/
int drv_lcd_touch_init(void)
{
	g_touch_fd = open(TOUCH_FILE_PATH, O_RDWR);
	if (g_touch_fd < 0)
	{
		LOG_ERROR("LCD TOUCH open fail TOUCH_FILE_PATH:%s\r\n", TOUCH_FILE_PATH);
		return -1;
	}

	//设置地址长度：0为7位地址
	ioctl(g_touch_fd, I2C_TENBIT, 0);

	//设置从机地址
	if(ioctl(g_touch_fd, I2C_SLAVE, TOUCH_ADDR) < 0)
	{
		LOG_ERROR("fail to set i2c device slave address!\r\n");
		close(g_touch_fd);
		return -1;
	}

	//设置收不到ACK时的重试次数
	ioctl(g_touch_fd,I2C_RETRIES,5);

	//校验触控SOC的ID
	touch_soc_id id = {
		.vendor_id = 0,
		.soc_id = ""
	};
	touch_get_id(&id);
	if(touch_id_checK(id) != 0)
	{
		LOG_WARNING("TOUCH SOC ID FAIL\r\n");
		return -1;
	}

	return 0;
}

/***************************************************************
 * Name:	 drv_lcd_touch_pos_count()
 * Input : void
 * Output: void
 * Return: 获取到的触控点个数
 * Author: hwl
 * Revise: V1.0
 * Description: 获取触控点个数
 ***************************************************************/
int drv_lcd_touch_pos_count(void)
{
	uint8_t count = 0;

	if(touch_read(TOUCH_POS_COUNT, &count, 1) != 0)	//获取异常
	{
		//清空获取到的值
		count = 0;
	}

	count = count > 2 ? count = 0 : count;

	return count;
}

/***************************************************************
 * Name:	 drv_lcd_touch_pos_count()
 * Input : void
 * Output: data_list:存储触控点的队列 必须长度为 TOUCH_POS_COUNT
 * Return: 获取到的触控点个数
 * Author: hwl
 * Revise: V1.0
 * Description: 获取触控点坐标
 ***************************************************************/
int drv_lcd_touch_pos_data(touch_pos_t* data_list)
{
	/*0=false 1=true*/
	static uint8_t pos_is_down[TOUCH_POS_COUNT] = {0x00};
	const uint8_t data_reg[TOUCH_POS_COUNT] = {TOUCH_POS_1, TOUCH_POS_2};

	uint8_t get_count = drv_lcd_touch_pos_count();

	for(uint8_t i = 0; i < get_count; i++)
	{
		uint8_t r_buf[TOUCH_POS_DATA_LEN];
		if(touch_read(data_reg[i], r_buf, TOUCH_POS_DATA_LEN) == 0)
		{
			data_list[i].x = ((uint16_t)(r_buf[0] & 0X0F) << 8) + r_buf[1];
			data_list[i].y = ((uint16_t)(r_buf[2] & 0X0F) << 8) + r_buf[3];
		}
	}

	return get_count;
}