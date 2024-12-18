#ifndef _DRV_LCD_H
#define _DRV_LCD_H

#include <stdint.h>

//定义LCD的尺寸
#define LCD_W 240
#define LCD_H 320

int drv_lcd_init(void);
void lcd_draw_point(uint16_t x,uint16_t y, uint16_t color);

#endif