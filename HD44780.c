#include "main.h"
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>


#define HD44780_CLEAR					   0x01
#define HD44780_HOME					      0x02
// flags for display entry mode
#define HD44780_ENTRY_MODE				   0x04
	#define HD44780_EM_SHIFT_CURSOR		0
	#define HD44780_EM_SHIFT_DISPLAY	   1
	#define HD44780_EM_DECREMENT		   0
	#define HD44780_EM_INCREMENT		   2
// flags for display on/off control
#define HD44780_DISPLAY_ONOFF			   0x08
	#define HD44780_DISPLAY_OFF		   0
	#define HD44780_DISPLAY_ON			   4
	#define HD44780_CURSOR_OFF			   0
	#define HD44780_CURSOR_ON			   2
	#define HD44780_CURSOR_NOBLINK	   0
	#define HD44780_CURSOR_BLINK		   1
// flags for display/cursor shift
#define HD44780_DISPLAY_CURSOR_SHIFT	0x10
	#define HD44780_SHIFT_CURSOR		   0
	#define HD44780_SHIFT_DISPLAY		   8
	#define HD44780_SHIFT_LEFT			   0
	#define HD44780_SHIFT_RIGHT		   4
// flags for function set
#define HD44780_FUNCTION_SET			   0x20
	#define HD44780_FONT5x7				   0
	#define HD44780_FONT5x10			   4
	#define HD44780_ONE_LINE			   0
	#define HD44780_TWO_LINE			   8
	#define HD44780_4_BIT				   0
	#define HD44780_8_BIT				   16
#define HD44780_CGRAM_SET				   0x40
#define HD44780_DDRAM_SET				   0x80
// custom characters ram addresses ( to display them use ASCII 0..7 )
#define GOTO_CGRAM_0            0x40
#define GOTO_CGRAM_1            0x40 + 1 * 0x08
#define GOTO_CGRAM_2            0x40 + 2 * 0x08
#define GOTO_CGRAM_3            0x40 + 3 * 0x08
#define GOTO_CGRAM_4            0x40 + 4 * 0x08
#define GOTO_CGRAM_5            0x40 + 5 * 0x08
#define GOTO_CGRAM_6            0x40 + 6 * 0x08
#define GOTO_CGRAM_7            0x40 + 7 * 0x08


void LCD_SetData        ( uint8_t data );
void LCD_Print          ( uint8_t *text );
void LCD_SendByte	(uint8_t byte, int type);




void LCD_port_init(void){
	GPIO_InitTypeDef  GPIO_InitStructure;

	//Enable the GPIO_LED Clock GPIOD
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOB , ENABLE);
	/* Configure the GPIO_LED pins*/
	//DB4 - PD6
	//...
	//DB7 - PD0
	//RW - PC9
	//EN - PC7
	//RS - PA9
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_2| GPIO_Pin_4| GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);


	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_3;
	GPIO_Init(GPIOE, &GPIO_InitStructure);


	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}



// initialisation of the LCD
void LCD_Init(void) {

    _delay_ms(30);


    LCD_HIGH(LCD_EN);
    LCD_SetData(3);
    LCD_LOW(LCD_EN);
    _delay_ms(10);


    LCD_HIGH(LCD_EN);
    LCD_SetData(3);
    LCD_LOW(LCD_EN);
    _delay_ms(1);


    LCD_HIGH(LCD_EN);
    LCD_SetData(3);
    LCD_LOW(LCD_EN);
    _delay_ms(1);

    LCD_HIGH(LCD_EN);
    LCD_SetData(2);     // 4 bit mode
    LCD_LOW(LCD_EN);
    _delay_ms(1);

    // init
    LCD_Command(HD44780_FUNCTION_SET |
					HD44780_FONT5x7 |
					HD44780_TWO_LINE |
					HD44780_4_BIT);

	LCD_Command(HD44780_DISPLAY_ONOFF |
				HD44780_DISPLAY_OFF);

	LCD_Command(HD44780_CLEAR);

	LCD_Command(HD44780_ENTRY_MODE |
				HD44780_EM_SHIFT_CURSOR |
				HD44780_EM_INCREMENT);

	LCD_Command(HD44780_DISPLAY_ONOFF |
				HD44780_DISPLAY_ON |
				HD44780_CURSOR_OFF |
				HD44780_CURSOR_NOBLINK);
	LCD_Command(HD44780_CLEAR);
	LCD_Command(HD44780_HOME);


}


// send 8bit to LCD
// @param int type,     0 - LCD command
//                                      1 - LCD data
void LCD_SendByte( uint8_t byte, int RS_mode ) {

    // set RS
    if( RS_mode == 1 )  LCD_HIGH( LCD_RS );
    else LCD_LOW( LCD_RS );

    // first the high nibble
    LCD_HIGH( LCD_EN );
    LCD_SetData( ( byte >> 4 ) & 0x0F );
    LCD_LOW( LCD_EN );
    _delay_ms(1);

    // then send the low nibble
    LCD_HIGH( LCD_EN );
    LCD_SetData( byte & 0x0F);
    LCD_LOW( LCD_EN );
    _delay_ms(1);


    if( RS_mode == 1 )
        _delay_us( 200 );
    else
        _delay_ms( 5 );
}


// set 4 bit data to lcd port
void LCD_SetData( uint8_t data) {
    if(data & (1<<0))
        LCD_HIGH(LCD_D4);
    else
        LCD_LOW(LCD_D4);

    if(data & (1<<1))
        LCD_HIGH(LCD_D5);
    else
        LCD_LOW(LCD_D5);

    if(data & (1<<2))
        LCD_HIGH(LCD_D6);
    else
        LCD_LOW(LCD_D6);

    if(data & (1<<3))
        LCD_HIGH(LCD_D7);
    else
        LCD_LOW(LCD_D7);
}


uint8_t _numlines = 4; //number of lines on LCD
void LCD_GotoXY(uint8_t col, uint8_t row){
	int row_offsets[4] = { 0x00, 0x40, 0x14, 0x54 };
	if ( row >= _numlines )
		row = _numlines-1;    // we count rows starting w/0
	LCD_Command(HD44780_DDRAM_SET | (col + row_offsets[row]));
}


// write a string on the display
void LCD_Print( uint8_t *text ) {
    int i = 0;
    int len = strlen( text );

    for( i = 0; i < len; i++ )
        LCD_Data( text[i] );
}


// printf like function for the LCD screen
void LCD_printf( const uint8_t* format, ... ) {
    va_list args;
    char        string[88];

    va_start( args, format );
    vsprintf( string, format, args );
    va_end( args );
    LCD_Print( string );
}

void _delay_ms ( int ms ) {
    int i = 4000 * ms;
    while( i-- > 0 ) {
        asm( "nop" );
    }
}
//80 cycles     168MHz
void _delay_us ( int us ) {
    int i = 4 * us;
    while( i-- > 0 ) {
        asm( "nop" );
    }
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
void LCD_DefineCharacter(char index, char * pattern)
{
    char i ;
    LCD_Command(HD44780_CGRAM_SET + (8 * index));
    for(i = 0; i < 8; i++)
        LCD_Data(*(pattern + i));
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------

void LCD_Clear(void)
{
    LCD_Command(HD44780_CLEAR);
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
void LCD_ShiftLeft(void)
{
    LCD_Command(HD44780_DISPLAY_CURSOR_SHIFT | HD44780_SHIFT_LEFT | HD44780_SHIFT_DISPLAY);
}
//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
void LCD_ShiftRight(void)
{
    LCD_Command(HD44780_DISPLAY_CURSOR_SHIFT | HD44780_SHIFT_RIGHT | HD44780_SHIFT_DISPLAY);
}



