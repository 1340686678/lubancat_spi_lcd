#ifndef _DRV_LCD_DEF
#define _DRV_LCD_DEF

/*LCD_LCD_CS GPIO1_B4*/
#define LCD_LCD_CS_NAME		"LCD_LCD_CS"
#define LCD_LCD_CS_PORT		3
#define LCD_LCD_CS_PIN		13

/*LCD_LCD_RST GPIO3_D4*/
#define LCD_LCD_RST_NAME	"LCD_LCD_RST"
#define LCD_LCD_RST_PORT	3
#define LCD_LCD_RST_PIN		28

/*LCD_LCD_RS GPIO3_D2*/
#define LCD_LCD_RS_NAME		"LCD_LCD_RS"
#define LCD_LCD_RS_PORT		3
#define LCD_LCD_RS_PIN		26

/*LCD_LCD_MOSI GPIO1_B2*/
#define LCD_LCD_MOSI_NAME	"LCD_LCD_MOSI"
#define LCD_LCD_MOSI_PORT	1
#define LCD_LCD_MOSI_PIN	10

/*LCD_LCD_SCK GPIO1_B3*/
#define LCD_LCD_SCK_NAME	"LCD_LCD_SCK"
#define LCD_LCD_SCK_PORT	1
#define LCD_LCD_SCK_PIN		11

/*LCD_LCD_MISO GPIO1_B1 READ*/
#define LCD_LCD_MISO_NAME	"LCD_LCD_MISO"
#define LCD_LCD_MISO_PORT	1
#define LCD_LCD_MISO_PIN	9

//画笔颜色
#define WHITE       0xFFFF
#define BLACK      	0x0000	  
#define BLUE       	0x001F  
#define BRED        0XF81F
#define GRED 			 	0XFFE0
#define GBLUE			 	0X07FF
#define RED         0xF800
#define MAGENTA     0xF81F
#define GREEN       0x07E0
#define CYAN        0x7FFF
#define YELLOW      0xFFE0
#define BROWN 			0XBC40 //棕色
#define BRRED 			0XFC07 //棕红色
#define GRAY  			0X8430 //灰色
//GUI颜色

#define DARKBLUE      	 0X01CF	//深蓝色
#define LIGHTBLUE      	 0X7D7C	//浅蓝色  
#define GRAYBLUE       	 0X5458 //灰蓝色
//以上三色为PANEL的颜色

#define LIGHTGREEN     	0X841F //浅绿色
#define LIGHTGRAY     0XEF5B //浅灰色(PANNEL)
#define LGRAY 			 		0XC618 //浅灰色(PANNEL),窗体背景色

#define LGRAYBLUE      	0XA651 //浅灰蓝色(中间层颜色)
#define LBBLUE          0X2B12 //浅棕蓝色(选择条目的反色)

#endif