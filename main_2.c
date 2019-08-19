/*
 * usart_rfit.c
 *
 * Created: 11/15/2018 3:30:56 PM
 * Author : Tommy
 */ 

 #define F_CPU 8000000UL
#include <avr/io.h>
#include "usart_ATmega1284.h"
#include "nokia5110.h"
#include <util/delay.h>

int main(void)
{
	DDRA = 0xFF; PORTA = 0x00;
    initUSART(0);
	nokia_lcd_init();
	nokia_lcd_clear();
	char temp = '0';
	//char card[12] = {'5','5','0','0','8','3','B','6','A','6','C','6'};
	//char input[12] = {'0','0','0','0','0','0','0','0','0','0','0','0'};
	//unsigned char valid = 0;
    while (1) 
    {
		if (USART_HasReceived(0))
		{
			temp = USART_Receive(0);
		}
		nokia_lcd_set_cursor(0,10);
		if (temp == 0x01)
		{
			nokia_lcd_clear();
			nokia_lcd_write_string("Connected!", 1);
			
			nokia_lcd_render();
			PORTA = 0x01;
		}
		else if (temp == 0x02)
		{
			nokia_lcd_clear();
			nokia_lcd_write_string("Invalid.", 1);
			
			nokia_lcd_render();
			PORTA = 0x00;
		}
		else
		{
			nokia_lcd_clear();
			nokia_lcd_write_string("Disconnected.", 1);
			nokia_lcd_render();
			PORTA = 0x00;
		}
		//nokia_lcd_render();
		_delay_ms(100);
    }
}

