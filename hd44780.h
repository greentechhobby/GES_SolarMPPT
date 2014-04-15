#ifndef __HD44780_H_INCLUDED
#define __HD44780_H_INCLUDED

#include "main.h"

#define LCD_PINS_BIT_ACCESS

#ifndef BITBAND_PERI
#define BITBAND_PERI(a,b) ((PERIPH_BB_BASE + (a-PERIPH_BASE)*32 + (b*4)))
#endif
#ifndef BITBAND_SRAM
#define BITBAND_SRAM(a,b) ((PERIPH_BB_BASE + (a-PERIPH_BASE)*32 + (b*4)))
#endif

//DB4 - PD6
//...
//DB7 - PD0
//RS - PE3
//RW - PE1
//EN - PB9

#ifdef LCD_PINS_BIT_ACCESS

#define LCD_RS_PORT		GPIOE_BASE
#define LCD_RS_PIN		3
#define LCD_RW_PORT		GPIOE_BASE
#define LCD_RW_PIN		1
#define LCD_EN_PORT		GPIOB_BASE
#define LCD_EN_PIN		9

#define LCD_D4_PORT		GPIOD_BASE
#define LCD_D4_PIN		6
#define LCD_D5_PORT		GPIOD_BASE
#define LCD_D5_PIN		4
#define LCD_D6_PORT		GPIOD_BASE
#define LCD_D6_PIN		2
#define LCD_D7_PORT		GPIOD_BASE
#define LCD_D7_PIN		0

#define BSRRL_REG_OFFSET		0x18
#define BSRRH_REG_OFFSET		0x1A

#define LCD_HIGH(lcd_signal)	\
		*((volatile unsigned char *) \
			(BITBAND_PERI(lcd_signal##_PORT+BSRRL_REG_OFFSET,lcd_signal##_PIN))) = 1

#define LCD_LOW(lcd_signal)	\
		*((volatile unsigned char *) \
			(BITBAND_PERI(lcd_signal##_PORT+BSRRH_REG_OFFSET,lcd_signal##_PIN))) = 1
#else

#define LCD_RS_PORT		GPIOE
#define LCD_RS_PIN		GPIO_Pin_3
#define LCD_RW_PORT		GPIOE
#define LCD_RW_PIN		GPIO_Pin_1
#define LCD_EN_PORT		GPIOB
#define LCD_EN_PIN		GPIO_Pin_9

#define LCD_D4_PORT		GPIOD
#define LCD_D4_PIN		GPIO_Pin_6
#define LCD_D5_PORT		GPIOD
#define LCD_D5_PIN		GPIO_Pin_4
#define LCD_D6_PORT		GPIOD
#define LCD_D6_PIN		GPIO_Pin_2
#define LCD_D7_PORT		GPIOD
#define LCD_D7_PIN		GPIO_Pin_0

#define LCD_HIGH(lcd_signal) \
		lcd_signal##_PORT->BSRRL |= lcd_signal##_PIN

#define LCD_LOW(lcd_signal)	\
		lcd_signal##_PORT->BSRRH |= lcd_signal##_PIN

#endif




#define LCD_Data(byte)		LCD_SendByte(byte, 1)
#define LCD_Command(_byte)	LCD_SendByte(_byte, 0)

void LCD_port_init(void);
void LCD_Init(void);
void LCD_GotoXY         ( uint8_t x, uint8_t y ); //the first row/collumn is 0
void LCD_printf         ( const uint8_t* format, ... );
void _delay_ms ( int ms );
void _delay_us ( int us );


#endif
