#ifndef _DRV_LCD_TOUCCH_H
#define _DRV_LCD_TOUCH_H

#include <stdint.h>
#include <stdbool.h>

int drv_lcd_touch_init(void);

int drv_lcd_touch_pos_count(void);
typedef struct
{
	uint16_t x;
	uint16_t y;
	bool is_down;
}touch_pos_t;
int drv_lcd_touch_pos_data(touch_pos_t* data_list);


#endif