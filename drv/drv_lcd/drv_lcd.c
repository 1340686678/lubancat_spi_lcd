#include "drv_lcd.h"
#include "drv_lcd_def.h"

#include "../drv_gpio/drv_gpio.h"

#include "../../common/log.h"

#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <linux/spi/spidev.h>

static M_GPIO_INFO_T lcd_rst_info = {
	.chip = NULL,
	.line = NULL,
	.name = "",
};

static M_GPIO_INFO_T lcd_rs_info = {
	.chip = NULL,
	.line = NULL,
	.name = "",
};

/***************************************************************
 * Name:	 drv_lcd_gpio_init()
 * Input : void
 * Output: void
 * Return: -1:失败 0:成功
 * Author: hwl
 * Revise: V1.0
 * Description: lcd的IO初始化
 ***************************************************************/
static int drv_lcd_gpio_init(void)
{
	M_GPIO_INIT_PARAM_T rst_param = {
		.mode = GPIO_OUTPUT,
		.name = LCD_LCD_RST_NAME,
		.pin = LCD_LCD_RST_PIN,
		.port = LCD_LCD_RST_PORT,
		.status = GPIO_OFF,
	};
	if(drv_gpio_get(rst_param, &lcd_rst_info) < 0)
	{
		goto lcd_rst_get_fail;
	}

	M_GPIO_INIT_PARAM_T rs_param = {
		.mode = GPIO_OUTPUT,
		.name = LCD_LCD_RS_NAME,
		.pin = LCD_LCD_RS_PIN,
		.port = LCD_LCD_RS_PORT,
		.status = GPIO_ON,
	};
	if(drv_gpio_get(rs_param, &lcd_rs_info) < 0)
	{
		goto lcd_rs_get_fail;
	}
	return 0;

lcd_rs_get_fail:
	drv_gpio_release(&lcd_rst_info);
lcd_rst_get_fail:
	return -1;
}

#define SPI_DEV_PATH "/dev/spidev0.0"

int fd;
/***************************************************************
 * Name:	 spi_init()
 * Input : void
 * Output: void
 * Return: void
 * Author: hwl
 * Revise: V1.0
 * Description: lcd的spi初始化
 ***************************************************************/
static void spi_init(void)
{
	int ret = 0;
	//打开 SPI 设备
	fd = open(SPI_DEV_PATH, O_RDWR);
	if(fd < 0)
	{
		LOG_ERROR("LCD_SPI can't open PATH:%s\n",SPI_DEV_PATH);
	}

	//设置 SPI 工作模式
	unsigned mode = SPI_MODE_0;
	ret = ioctl(fd, SPI_IOC_WR_MODE32, &mode);
	if (ret == -1)
	{
		LOG_ERROR("LCD_SPI can't set spi mode\n");
	}

	//设置一个字节的位数
	uint8_t bits = 8;
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
	{
		LOG_ERROR("can't set bits per word\n");
	}

	//设置 SPI 最高工作频率
	uint32_t speed = 10000000;
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
	{
		LOG_ERROR("can't set max speed hz\n");
	}
}

/***************************************************************
 * Name:	 spi_transfer()
 * Input : fd:SPI设备描述符 tx:发送数据 rx:接收数据 len:发送数据长度(单位，字节)
 * Output: void
 * Return: void
 * Author: hwl
 * Revise: V1.0
 * Description: spi参数数据
 * 如果 只读不发送 tx设为0x00
 ***************************************************************/
static void spi_transfer(int fd, uint8_t const *tx, uint8_t const *rx, size_t len)
{
	int ret;

	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = len,
		.delay_usecs = 0,
		.speed_hz = 10000000,
		.bits_per_word = 8,
		.tx_nbits = 1,
		.rx_nbits = 1
	};

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	
	if (ret < 1)
	{
		LOG_WARNING("can't send spi message\n");
	}
}

/***************************************************************
 * Name:	 spi_w_data()
 * Input : data:写的数据
 * Output: void
 * Return: void
 * Author: hwl
 * Revise: V1.0
 * Description: spi发送
 ***************************************************************/
static void spi_w_data(uint8_t data)
{
	uint8_t tx_buffer = data;
	uint8_t rx_buffer = 0;

	spi_transfer(fd, &tx_buffer, &rx_buffer, 1);
}

/***************************************************************
 * Name:	 spi_r_data()
 * Input : void
 * Output: void
 * Return: 接收到的数据
 * Author: hwl
 * Revise: V1.0
 * Description: spi接收
 ***************************************************************/
static uint8_t spi_r_data(void)
{
	uint8_t tx_buffer = 0;
	uint8_t rx_buffer = 0;

	spi_transfer(fd, &tx_buffer, &rx_buffer, 1);

	return rx_buffer;
} 

/* SET 命令 CLR 数据 */
#define LCD_RS_SET drv_gpio_set_status(&lcd_rs_info, GPIO_ON)
#define LCD_RS_CLR drv_gpio_set_status(&lcd_rs_info, GPIO_OFF)

/* SET 运行 CLR 复位 */
#define LCD_RST_SET drv_gpio_set_status(&lcd_rst_info, GPIO_ON)
#define LCD_RST_CLR drv_gpio_set_status(&lcd_rst_info, GPIO_OFF)

#define delay_ms(x) usleep(x*1000)

//LCD重要参数集
typedef struct  
{
	uint16_t width;		//LCD 宽度
	uint16_t height;	//LCD 高度
	uint8_t dir;			//横屏还是竖屏控制：0，竖屏；1，横屏。
	uint16_t wramcmd;	//开始写gram指令
	uint16_t rramcmd;	//开始读gram指令
	uint16_t setxcmd;	//设置x坐标指令
	uint16_t setycmd;	//设置y坐标指令
}_lcd_dev;

static _lcd_dev g_lcd_param;

uint16_t POINT_COLOR = 0x0000,BACK_COLOR = 0xFFFF;  

/***************************************************************
 * Name:	 lcd_w_cmd()
 * Input : cmd:写的指令
 * Output: void
 * Return: void
 * Author: hwl
 * Revise: V1.0
 * Description: 向lcd写指令
 ***************************************************************/
static void lcd_w_cmd(uint8_t cmd)
{
	LCD_RS_CLR;
	spi_w_data(cmd);
}

/***************************************************************
 * Name:	 lcd_w_data()
 * Input : data:写的数据
 * Output: void
 * Return: void
 * Author: hwl
 * Revise: V1.0
 * Description: 向lcd写数据
 ***************************************************************/
static void lcd_w_data(uint8_t data)
{
	LCD_RS_SET;
	spi_w_data(data);
}

/***************************************************************
 * Name:	 lcd_r_data()
 * Input : void
 * Output: void
 * Return: 接收到的数据
 * Author: hwl
 * Revise: V1.0
 * Description: 向lcd读数据
 ***************************************************************/
static uint8_t lcd_r_data(void)
{
	uint8_t data;
	LCD_RS_SET;
	data = spi_r_data();
	return data;
}

/***************************************************************
 * Name:	 lcd_w_reg()
 * Input : reg:寄存器 data:写入的数据
 * Output: void
 * Return: void
 * Author: hwl
 * Revise: V1.0
 * Description: 写lcd寄存器数据
 ***************************************************************/
void lcd_w_reg(uint8_t reg, uint16_t data)
{	
	lcd_w_cmd(reg);
	lcd_w_data(data);
}

/***************************************************************
 * Name:	 lcd_r_reg()
 * Input : reg:寄存器
 * Output: void
 * Return: 寄存器数据
 * Author: hwl
 * Revise: V1.0
 * Description: 读lcd寄存器数据
 ***************************************************************/
uint8_t lcd_r_reg(uint8_t reg)
{
	lcd_w_cmd(reg);
  return lcd_r_data();
}

/***************************************************************
 * Name:	 lcd_w_data_16bit()
 * Input : Data:需要写入的数据
 * Output: void
 * Return: void
 * Author: hwl
 * Revise: V1.0
 * Description: 写入16bit数据
 ***************************************************************/
void lcd_w_data_16bit(uint16_t Data)
{	
	LCD_RS_SET;
	spi_w_data(Data>>8);
	spi_w_data(Data);
}

/***************************************************************
 * Name:	 lcd_set_windows()
 * Input : xStar:窗口开始X点 yStar:窗口开始Y点 xEnd:窗口结束Y点 yEnd:窗口结束Y点 
 * Output: void
 * Return: void
 * Author: hwl
 * Revise: V1.0
 * Description: 设置操作窗口
 ***************************************************************/
void lcd_set_windows(uint16_t x_star, uint16_t y_star,uint16_t x_end,uint16_t y_end)
{	
	lcd_w_cmd(g_lcd_param.setxcmd);
	lcd_w_data(x_star>>8);
	lcd_w_data(0x00FF&x_star);
	lcd_w_data(x_end>>8);
	lcd_w_data(0x00FF&x_end);

	lcd_w_cmd(g_lcd_param.setycmd);
	lcd_w_data(y_star>>8);
	lcd_w_data(0x00FF&y_star);
	lcd_w_data(y_end>>8);
	lcd_w_data(0x00FF&y_end);

	lcd_w_cmd(g_lcd_param.wramcmd);	//开始写入GRAM
}

/***************************************************************
 * Name:	 lcd_set_cursor()
 * Input : 光标坐标
 * Output: void
 * Return: void
 * Author: hwl
 * Revise: V1.0
 * Description: 设置光标
 ***************************************************************/
void lcd_set_cursor(uint16_t Xpos, uint16_t Ypos)
{
	lcd_set_windows(Xpos,Ypos,Xpos,Ypos);
} 

/***************************************************************
 * Name:	 lcd_draw_point()
 * Input : 绘制点的位置
 * Output: void
 * Return: void
 * Author: hwl
 * Revise: V1.0
 * Description: 绘制一个点
 ***************************************************************/
void lcd_draw_point(uint16_t x,uint16_t y)
{
	lcd_set_cursor(x,y);	//设置光标位置 
	lcd_w_data_16bit(POINT_COLOR);
}

/***************************************************************
 * Name:	 lcd_fill()
 * Input : Color:填充的颜色
 * Output: void
 * Return: void
 * Author: hwl
 * Revise: V1.0
 * Description: 填充屏幕
 ***************************************************************/
void lcd_fill(uint16_t Color)
{
  unsigned int i,m;  
	lcd_set_windows(0,0,g_lcd_param.width-1,g_lcd_param.height-1);   
	LCD_RS_SET;
	for(i=0;i<g_lcd_param.height;i++)
	{
    for(m=0;m<g_lcd_param.width;m++)
    {	
			spi_w_data(Color>>8);
			spi_w_data(Color);
		}
	}
} 

/***************************************************************
 * Name:	 lcd_reset()
 * Input : void
 * Output: void
 * Return: void
 * Author: hwl
 * Revise: V1.0
 * Description: 复位LCD
 ***************************************************************/
static void lcd_reset(void)
{
	LCD_RST_SET;
	usleep(50000);
	LCD_RST_CLR;
	usleep(1000000);
	LCD_RST_SET;
	usleep(50000);
}

/***************************************************************
 * Name:	 lcd_direction()
 * Input : direction:屏幕旋转方向
 * Output: void
 * Return: void
 * Author: hwl
 * Revise: V1.0
 * Description: 屏幕屏幕方向
 ***************************************************************/
void lcd_direction(uint8_t direction)
{ 
	g_lcd_param.setxcmd=0x2A;
	g_lcd_param.setycmd=0x2B;
	g_lcd_param.wramcmd=0x2C;
	g_lcd_param.rramcmd=0x2E;
	g_lcd_param.dir = direction%4;

	switch(g_lcd_param.dir)
	{
		case 0:	//0度
		{
			g_lcd_param.width=LCD_W;
			g_lcd_param.height=LCD_H;
			lcd_w_reg(0x36,(1<<3)|(0<<6)|(0<<7));					//BGR==1,MY==0,MX==0,MV==0
			break;
		}
		case 1:	//1-90度
		{
			g_lcd_param.width=LCD_H;
			g_lcd_param.height=LCD_W;
			lcd_w_reg(0x36,(1<<3)|(0<<7)|(1<<6)|(1<<5));	//BGR==1,MY==1,MX==0,MV==1
			break;
		}
		case 2:	//2-180度
		{
			g_lcd_param.width=LCD_W;
			g_lcd_param.height=LCD_H;
			lcd_w_reg(0x36,(1<<3)|(1<<6)|(1<<7));					//BGR==1,MY==0,MX==0,MV==0
			break;
		}
		case 3:	//3-270度
		{
			g_lcd_param.width=LCD_H;
			g_lcd_param.height=LCD_W;
			lcd_w_reg(0x36,(1<<3)|(1<<7)|(1<<5));					//BGR==1,MY==1,MX==0,MV==1
			break;
		}
		default:
		{
			break;
		}
	}
}

/***************************************************************
 * Name:	 lcd_read_id()
 * Input : void
 * Output: void
 * Return: ID
 * Author: hwl
 * Revise: V1.0
 * Description: 获取屏幕ID
 ***************************************************************/
static uint16_t lcd_read_id(void)
{
	uint8_t i,val[3] = {0};
	uint16_t id;
	
	unsigned char tx_buffer = 0;
	unsigned char rx_buffer = 0;

	for(i=1;i<4;i++)
	{
		lcd_w_cmd(0xD9);

		lcd_w_data(0x10+i);

		lcd_w_cmd(0xD3);

		val[i-1] = lcd_r_data();
	}

	id=val[1];
	id<<=8;
	id|=val[2];
	return id;
}

/***************************************************************
 * Name:	 drv_lcd_init()
 * Input : void
 * Output: void
 * Return: ID
 * Author: hwl
 * Revise: V1.0
 * Description: lcd初始化
 ***************************************************************/
int drv_lcd_init(void)
{
	if(drv_lcd_gpio_init() < 0)
	{
		LOG_ERROR("LCD GPIO INIT FAIL");
	}
	spi_init();
 	lcd_reset();
	printf("READ ID %x\r\n", lcd_read_id());

	lcd_w_cmd(0xCF);
	lcd_w_data(0x00);
	lcd_w_data(0xC1);
	lcd_w_data(0x30);
 
	lcd_w_cmd(0xED);
	lcd_w_data(0x64);
	lcd_w_data(0x03);
	lcd_w_data(0X12);
	lcd_w_data(0X81);
 
	lcd_w_cmd(0xE8);
	lcd_w_data(0x85);
	lcd_w_data(0x00);
	lcd_w_data(0x78);

	lcd_w_cmd(0xCB);
	lcd_w_data(0x39);
	lcd_w_data(0x2C);
	lcd_w_data(0x00);
	lcd_w_data(0x34);
	lcd_w_data(0x02);
	
	lcd_w_cmd(0xF7);
	lcd_w_data(0x20);
 
	lcd_w_cmd(0xEA);
	lcd_w_data(0x00);
	lcd_w_data(0x00);

	lcd_w_cmd(0xC0);
	lcd_w_data(0x13);
 
	lcd_w_cmd(0xC1);
	lcd_w_data(0x13);
 
	lcd_w_cmd(0xC5);
	lcd_w_data(0x22);
	lcd_w_data(0x35);
 
	lcd_w_cmd(0xC7);
	lcd_w_data(0xBD);

	lcd_w_cmd(0x21);

	lcd_w_cmd(0x36);
	lcd_w_data(0x08);

	lcd_w_cmd(0xB6);
	lcd_w_data(0x0A);
	lcd_w_data(0xA2);

	lcd_w_cmd(0x3A);
	lcd_w_data(0x55); 

	lcd_w_cmd(0xF6);
	lcd_w_data(0x01);
	lcd_w_data(0x30);

	lcd_w_cmd(0xB1);
	lcd_w_data(0x00);
	lcd_w_data(0x1B);
 
	lcd_w_cmd(0xF2);
	lcd_w_data(0x00);
 
	lcd_w_cmd(0x26);
	lcd_w_data(0x01);
 
	lcd_w_cmd(0xE0);
	lcd_w_data(0x0F);
	lcd_w_data(0x35);
	lcd_w_data(0x31);
	lcd_w_data(0x0B);
	lcd_w_data(0x0E);
	lcd_w_data(0x06);
	lcd_w_data(0x49);
	lcd_w_data(0xA7);
	lcd_w_data(0x33);
	lcd_w_data(0x07);
	lcd_w_data(0x0F);
	lcd_w_data(0x03);
	lcd_w_data(0x0C);
	lcd_w_data(0x0A);
	lcd_w_data(0x00);
 
	lcd_w_cmd(0XE1);
	lcd_w_data(0x00);
	lcd_w_data(0x0A);
	lcd_w_data(0x0F);
	lcd_w_data(0x04);
	lcd_w_data(0x11);
	lcd_w_data(0x08);
	lcd_w_data(0x36);
	lcd_w_data(0x58);
	lcd_w_data(0x4D);
	lcd_w_data(0x07);
	lcd_w_data(0x10);
	lcd_w_data(0x0C);
	lcd_w_data(0x32);
	lcd_w_data(0x34);
	lcd_w_data(0x0F);

	lcd_w_cmd(0x11);
	delay_ms(120);
	lcd_w_cmd(0x29);

	lcd_direction(0);	//设置LCD显示方向 
	
	lcd_fill(DARKBLUE);
}