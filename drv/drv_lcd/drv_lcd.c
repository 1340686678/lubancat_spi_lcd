#include "drv_lcd.h"
#include "drv_lcd_def.h"

#include "../drv_gpio/drv_gpio.h"

#include "../../common/log.h"

#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <linux/spi/spidev.h>


static M_GPIO_INFO_T lcd_cs_info = {
	.chip = NULL,
	.line = NULL,
	.name = "",
};

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

static M_GPIO_INFO_T lcd_mosi_info = {
	.chip = NULL,
	.line = NULL,
	.name = "",
};

static M_GPIO_INFO_T lcd_sck_info = {
	.chip = NULL,
	.line = NULL,
	.name = "",
};

static M_GPIO_INFO_T lcd_miso_info = {
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
	M_GPIO_INIT_PARAM_T cs_param = {
		.mode = GPIO_OUTPUT,
		.name = LCD_LCD_CS_NAME,
		.pin = LCD_LCD_CS_PIN,
		.port = LCD_LCD_CS_PORT,
		.status = GPIO_OFF,
	};
	if(drv_gpio_get(cs_param, &lcd_cs_info) < 0)
	{
		goto lcd_cs_get_fail;
	}

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
	
	M_GPIO_INIT_PARAM_T mosi_param = {
		.mode = GPIO_OUTPUT,
		.name = LCD_LCD_MOSI_NAME,
		.pin = LCD_LCD_MOSI_PIN,
		.port = LCD_LCD_MOSI_PORT,
		.status = GPIO_ON,
	};
	if(drv_gpio_get(mosi_param, &lcd_mosi_info) < 0)
	{
		goto lcd_mosi_get_fail;
	}

	M_GPIO_INIT_PARAM_T sck_param = {
		.mode = GPIO_OUTPUT,
		.name = LCD_LCD_SCK_NAME,
		.pin = LCD_LCD_SCK_PIN,
		.port = LCD_LCD_SCK_PORT,
		.status = GPIO_OFF,
	};
	if(drv_gpio_get(sck_param, &lcd_sck_info) < 0)
	{
		goto lcd_sck_get_fail;
	}

	M_GPIO_INIT_PARAM_T miso_param = {
		.mode = GPIO_INPUT,
		.name = LCD_LCD_MISO_NAME,
		.pin = LCD_LCD_MISO_PIN,
		.port = LCD_LCD_MISO_PORT,
		.status = GPIO_OFF,
	};
	if(drv_gpio_get(miso_param, &lcd_miso_info) < 0)
	{
		goto lcd_miso_get_fail;
	}

	return 0;

lcd_miso_get_fail:
	drv_gpio_release(&lcd_sck_info);
lcd_sck_get_fail:
	drv_gpio_release(&lcd_mosi_info);
lcd_mosi_get_fail:
	drv_gpio_release(&lcd_rs_info);
lcd_rs_get_fail:
	drv_gpio_release(&lcd_rst_info);
lcd_rst_get_fail:
	drv_gpio_release(&lcd_cs_info);
lcd_cs_get_fail:
	return -1;
}

#define SPI_MOSI_SET drv_gpio_set_status(&lcd_mosi_info, GPIO_ON)
#define SPI_MOSI_CLR drv_gpio_set_status(&lcd_mosi_info, GPIO_OFF)

#define SPI_SCLK_SET drv_gpio_set_status(&lcd_sck_info, GPIO_ON)
#define SPI_SCLK_CLR drv_gpio_set_status(&lcd_sck_info, GPIO_OFF)

#define SPI_MISO_READ (drv_gpio_get_status(&lcd_miso_info) == GPIO_ON)

#define LCD_CS_SET drv_gpio_set_status(&lcd_cs_info, GPIO_ON)
#define LCD_CS_CLR drv_gpio_set_status(&lcd_cs_info, GPIO_OFF)

#define LCD_RS_SET drv_gpio_set_status(&lcd_rs_info, GPIO_ON)
#define LCD_RS_CLR drv_gpio_set_status(&lcd_rs_info, GPIO_OFF)

#define LCD_RST_SET drv_gpio_set_status(&lcd_rst_info, GPIO_ON)
#define LCD_RST_CLR drv_gpio_set_status(&lcd_rst_info, GPIO_OFF)

#define delay_ms(x) usleep(x*1000)

//LCD重要参数集
typedef struct  
{										    
	uint16_t width;			//LCD 宽度
	uint16_t height;			//LCD 高度
	uint16_t id;				  //LCD ID
	uint8_t		dir;			  //横屏还是竖屏控制：0，竖屏；1，横屏。	
	uint16_t	 wramcmd;		//开始写gram指令
	uint16_t  rramcmd;   //开始读gram指令
	uint16_t  setxcmd;		//设置x坐标指令
	uint16_t  setycmd;		//设置y坐标指令	 
}_lcd_dev;

static _lcd_dev lcddev;

uint16_t POINT_COLOR = 0x0000,BACK_COLOR = 0xFFFF;  

static void SPI_WriteByte(uint8_t Byte)
{
	uint8_t i=0;
	for(i=0;i<8;i++)
	{
		if(Byte&0x80)
		{
			SPI_MOSI_SET;
		}
		else
		{
			SPI_MOSI_CLR;
		}
		SPI_SCLK_CLR;
		SPI_SCLK_SET;
		Byte<<=1;
	}
}

static uint8_t SPI_ReadByte(void)
{
	uint8_t value=0,i=0,byte=0xFF;
	for(i=0;i<8;i++)
	{
		value<<=1;
		if(byte&0x80)
		{
			SPI_MOSI_SET;
		}
		else
		{
			SPI_MOSI_CLR;
		}
		byte<<=1;
		SPI_SCLK_CLR;
		if(SPI_MISO_READ)
		{
			value += 1;
		}
		SPI_SCLK_SET;	
	}
	return value;
} 

static void LCD_WR_REG(uint8_t data)
{ 
   LCD_CS_CLR;     
	 LCD_RS_CLR;	  
   SPI_WriteByte(data);
   LCD_CS_SET;	
}

static void LCD_WR_DATA(uint8_t data)
{
   LCD_CS_CLR;
	 LCD_RS_SET;
   SPI_WriteByte(data);
   LCD_CS_SET;
}

static uint8_t LCD_RD_DATA(void)
{
	 uint8_t data;
	 LCD_CS_CLR;
	 LCD_RS_SET;
	 data = SPI_ReadByte();
	 LCD_CS_SET;
	 return data;
}

void LCD_WriteReg(uint8_t LCD_Reg, uint16_t LCD_RegValue)
{	
	LCD_WR_REG(LCD_Reg);
	LCD_WR_DATA(LCD_RegValue);
}

uint8_t LCD_ReadReg(uint8_t LCD_Reg)
{
	LCD_WR_REG(LCD_Reg);
  return LCD_RD_DATA();
}

/***************************************************************
 * Name:	 Lcd_WriteData_16Bit()
 * Input : Data:需要写入的数据
 * Output: void
 * Return: void
 * Author: hwl
 * Revise: V1.0
 * Description: 写入16bit数据
 ***************************************************************/
void Lcd_WriteData_16Bit(uint16_t Data)
{	
	LCD_CS_CLR;
	LCD_RS_SET;  
	SPI_WriteByte(Data>>8);
	SPI_WriteByte(Data);

	LCD_CS_SET;
}

/***************************************************************
 * Name:	 LCD_SetWindows()
 * Input : Data:需要写入的数据
 * Output: void
 * Return: void
 * Author: hwl
 * Revise: V1.0
 * Description: 设置操作窗口
 ***************************************************************/
void LCD_SetWindows(uint16_t xStar, uint16_t yStar,uint16_t xEnd,uint16_t yEnd)
{	
	LCD_WR_REG(lcddev.setxcmd);
	LCD_WR_DATA(xStar>>8);
	LCD_WR_DATA(0x00FF&xStar);
	LCD_WR_DATA(xEnd>>8);
	LCD_WR_DATA(0x00FF&xEnd);

	LCD_WR_REG(lcddev.setycmd);
	LCD_WR_DATA(yStar>>8);
	LCD_WR_DATA(0x00FF&yStar);
	LCD_WR_DATA(yEnd>>8);
	LCD_WR_DATA(0x00FF&yEnd);

	LCD_WR_REG(lcddev.wramcmd);	//开始写入GRAM
}

/***************************************************************
 * Name:	 LCD_SetCursor()
 * Input : 光标坐标
 * Output: void
 * Return: void
 * Author: hwl
 * Revise: V1.0
 * Description: 设置光标
 ***************************************************************/
void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos)
{	  	    			
	LCD_SetWindows(Xpos,Ypos,Xpos,Ypos);	
} 

/***************************************************************
 * Name:	 LCD_DrawPoint()
 * Input : 绘制点的位置
 * Output: void
 * Return: void
 * Author: hwl
 * Revise: V1.0
 * Description: 绘制一个点
 ***************************************************************/
void LCD_DrawPoint(uint16_t x,uint16_t y)
{
	LCD_SetCursor(x,y);//设置光标位置 
	Lcd_WriteData_16Bit(POINT_COLOR); 
}

/***************************************************************
 * Name:	 LCD_Fill()
 * Input : Color:填充的颜色
 * Output: void
 * Return: void
 * Author: hwl
 * Revise: V1.0
 * Description: 填充屏幕
 ***************************************************************/
void LCD_Fill(uint16_t Color)
{
  unsigned int i,m;  
	LCD_SetWindows(0,0,lcddev.width-1,lcddev.height-1);   
	LCD_CS_CLR;
	LCD_RS_SET;
	for(i=0;i<lcddev.height;i++)
	{
    for(m=0;m<lcddev.width;m++)
    {	
			SPI_WriteByte(Color>>8);
			SPI_WriteByte(Color);
		}
	}
	 LCD_CS_SET;
} 

/***************************************************************
 * Name:	 LCD_RESET()
 * Input : void
 * Output: void
 * Return: void
 * Author: hwl
 * Revise: V1.0
 * Description: 复位LCD
 ***************************************************************/
static void LCD_RESET(void)
{
	LCD_RST_SET;
	usleep(50000);
	LCD_RST_CLR;
	usleep(1000000);
	LCD_RST_SET;
	usleep(50000);
}

/***************************************************************
 * Name:	 LCD_direction()
 * Input : direction:屏幕旋转方向
 * Output: void
 * Return: void
 * Author: hwl
 * Revise: V1.0
 * Description: 复位LCD
 ***************************************************************/
void LCD_direction(uint8_t direction)
{ 
	lcddev.setxcmd=0x2A;
	lcddev.setycmd=0x2B;
	lcddev.wramcmd=0x2C;
	lcddev.rramcmd=0x2E;
	lcddev.dir = direction%4;
	switch(lcddev.dir)
	{
		case 0:	//0度
		{
			lcddev.width=LCD_W;
			lcddev.height=LCD_H;		
			LCD_WriteReg(0x36,(1<<3)|(0<<6)|(0<<7));//BGR==1,MY==0,MX==0,MV==0
			break;
		}
		case 1:	//1-90度
		{
			lcddev.width=LCD_H;
			lcddev.height=LCD_W;
			LCD_WriteReg(0x36,(1<<3)|(0<<7)|(1<<6)|(1<<5));//BGR==1,MY==1,MX==0,MV==1
			break;
		}
		case 2:	//2-180度
		{
			lcddev.width=LCD_W;
			lcddev.height=LCD_H;
			LCD_WriteReg(0x36,(1<<3)|(1<<6)|(1<<7));//BGR==1,MY==0,MX==0,MV==0
			break;
		}

		case 3:	//3-270度
		{
			lcddev.width=LCD_H;
			lcddev.height=LCD_W;
			LCD_WriteReg(0x36,(1<<3)|(1<<7)|(1<<5));//BGR==1,MY==1,MX==0,MV==1
			break;
		}
		default:
		{
			break;
		}
	}	
}	 

/***************************************************************
 * Name:	 LCD_Read_ID()
 * Input : void
 * Output: void
 * Return: ID
 * Author: hwl
 * Revise: V1.0
 * Description: 获取屏幕ID
 ***************************************************************/
static uint16_t LCD_Read_ID(void)
{
	uint8_t i,val[3] = {0};
	uint16_t id;
	
	for(i=1;i<4;i++)
	{
		LCD_WR_REG(0xD9);
		LCD_WR_DATA(0x10+i);
		LCD_WR_REG(0xD3);
		val[i-1] = LCD_RD_DATA();
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

 	LCD_RESET();
	printf("READ ID %x\r\n", LCD_Read_ID());

	LCD_WR_REG(0xCF);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0xC1);
	LCD_WR_DATA(0x30);
 
	LCD_WR_REG(0xED);  
	LCD_WR_DATA(0x64); 
	LCD_WR_DATA(0x03); 
	LCD_WR_DATA(0X12); 
	LCD_WR_DATA(0X81); 
 
	LCD_WR_REG(0xE8);  
	LCD_WR_DATA(0x85); 
	LCD_WR_DATA(0x00); 
	LCD_WR_DATA(0x78); 

	LCD_WR_REG(0xCB);  
	LCD_WR_DATA(0x39); 
	LCD_WR_DATA(0x2C); 
	LCD_WR_DATA(0x00); 
	LCD_WR_DATA(0x34); 
	LCD_WR_DATA(0x02); 
	
	LCD_WR_REG(0xF7);  
	LCD_WR_DATA(0x20); 
 
	LCD_WR_REG(0xEA);  
	LCD_WR_DATA(0x00); 
	LCD_WR_DATA(0x00); 

	LCD_WR_REG(0xC0);       //Power control 
	LCD_WR_DATA(0x13);     //VRH[5:0] 
 
	LCD_WR_REG(0xC1);       //Power control 
	LCD_WR_DATA(0x13);     //SAP[2:0];BT[3:0] 
 
	LCD_WR_REG(0xC5);       //VCM control 
	LCD_WR_DATA(0x22);   //22
	LCD_WR_DATA(0x35);   //35
 
	LCD_WR_REG(0xC7);       //VCM control2 
	LCD_WR_DATA(0xBD);  //AF

	LCD_WR_REG(0x21);

	LCD_WR_REG(0x36);       // Memory Access Control 
	LCD_WR_DATA(0x08); 

	LCD_WR_REG(0xB6);  
	LCD_WR_DATA(0x0A); 
	LCD_WR_DATA(0xA2); 

	LCD_WR_REG(0x3A);       
	LCD_WR_DATA(0x55); 

	LCD_WR_REG(0xF6);  //Interface Control
	LCD_WR_DATA(0x01); 
	LCD_WR_DATA(0x30);  //MCU

	LCD_WR_REG(0xB1);       //VCM control 
	LCD_WR_DATA(0x00); 
	LCD_WR_DATA(0x1B); 
 
	LCD_WR_REG(0xF2);       // 3Gamma Function Disable 
	LCD_WR_DATA(0x00); 
 
	LCD_WR_REG(0x26);       //Gamma curve selected 
	LCD_WR_DATA(0x01); 
 
	LCD_WR_REG(0xE0);       //Set Gamma 
	LCD_WR_DATA(0x0F); 
	LCD_WR_DATA(0x35); 
	LCD_WR_DATA(0x31); 
	LCD_WR_DATA(0x0B); 
	LCD_WR_DATA(0x0E); 
	LCD_WR_DATA(0x06); 
	LCD_WR_DATA(0x49); 
	LCD_WR_DATA(0xA7); 
	LCD_WR_DATA(0x33); 
	LCD_WR_DATA(0x07); 
	LCD_WR_DATA(0x0F); 
	LCD_WR_DATA(0x03); 
	LCD_WR_DATA(0x0C); 
	LCD_WR_DATA(0x0A); 
	LCD_WR_DATA(0x00); 
 
	LCD_WR_REG(0XE1);       //Set Gamma 
	LCD_WR_DATA(0x00); 
	LCD_WR_DATA(0x0A); 
	LCD_WR_DATA(0x0F); 
	LCD_WR_DATA(0x04); 
	LCD_WR_DATA(0x11); 
	LCD_WR_DATA(0x08); 
	LCD_WR_DATA(0x36); 
	LCD_WR_DATA(0x58); 
	LCD_WR_DATA(0x4D); 
	LCD_WR_DATA(0x07); 
	LCD_WR_DATA(0x10); 
	LCD_WR_DATA(0x0C); 
	LCD_WR_DATA(0x32); 
	LCD_WR_DATA(0x34); 
	LCD_WR_DATA(0x0F); 

	LCD_WR_REG(0x11);       //Exit Sleep 
	delay_ms(120); 
	LCD_WR_REG(0x29);       //Display on 

  LCD_direction(0);//设置LCD显示方向 
	
	LCD_Fill(BLUE);//清全屏白色

}