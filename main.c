/*
 * tft lcd test.c
 *
 * Created: 11/27/2018 2:46:47 PM
 * Author : Tommy
 */ 

#define F_CPU 8000000UL
#include <avr/io.h>
#include "scheduler.h"
#include "usart_ATmega1284.h"
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h> // string manipulation routines
#include <avr/sleep.h> // used for sleep functions
#include <stdlib.h>#include <avr/pgmspace.h> // put character data into progmemchar temp = '0';
char card[12] = {'5','5','0','0','8','3','B','6','A','6','C','6'};
char input[12] = {'0','0','0','0','0','0','0','0','0','0','0','0'};
unsigned char valid = 0;
#define ClearBit(x,y) x &= ~_BV(y) // equivalent to cbi(x,y)
#define SetBit(x,y) x |= _BV(y) // equivalent to sbi(x,y)

#define SWRESET 0x01 // software reset
#define SLPOUT 0x11 // sleep out
#define DISPOFF 0x28 // display off
#define DISPON 0x29 // display on
#define CASET 0x2A // column address set
#define RASET 0x2B // row address set
#define RAMWR 0x2C // RAM write
#define MADCTL 0x36 // axis control
#define COLMOD 0x3A // color mode
// 1.8" TFT display constants
#define XSIZE 128
#define YSIZE 160
#define XMAX XSIZE-1
#define YMAX YSIZE-1
// Color constants
#define BLACK 0x0000
#define BLUE 0x001F
#define RED 0xF800
#define GREEN 0x0400
#define LIME 0x07E0
#define CYAN 0x07FF
#define MAGENTA 0xF81F
#define YELLOW 0xFFE0
#define WHITE 0xFFFF

typedef uint8_t byte; // I just like byte & sbyte better
typedef int8_t sbyte;const byte FONT_CHARS[96][5] PROGMEM =
{
	{ 0x00, 0x00, 0x00, 0x00, 0x00 }, // (space)
	{ 0x00, 0x00, 0x5F, 0x00, 0x00 }, // !
	{ 0x00, 0x07, 0x00, 0x07, 0x00 }, // "
	{ 0x14, 0x7F, 0x14, 0x7F, 0x14 }, // #
	{ 0x24, 0x2A, 0x7F, 0x2A, 0x12 }, // $
	{ 0x23, 0x13, 0x08, 0x64, 0x62 }, // %
	{ 0x36, 0x49, 0x55, 0x22, 0x50 }, // &
	{ 0x00, 0x05, 0x03, 0x00, 0x00 }, // '
	{ 0x00, 0x1C, 0x22, 0x41, 0x00 }, // (
	{ 0x00, 0x41, 0x22, 0x1C, 0x00 }, // )
	{ 0x08, 0x2A, 0x1C, 0x2A, 0x08 }, // *
	{ 0x08, 0x08, 0x3E, 0x08, 0x08 }, // +
	{ 0x00, 0x50, 0x30, 0x00, 0x00 }, // ,
	{ 0x08, 0x08, 0x08, 0x08, 0x08 }, // -
	{ 0x00, 0x60, 0x60, 0x00, 0x00 }, // .
	{ 0x20, 0x10, 0x08, 0x04, 0x02 }, // /
	{ 0x3E, 0x51, 0x49, 0x45, 0x3E }, // 0
	{ 0x00, 0x42, 0x7F, 0x40, 0x00 }, // 1
	{ 0x42, 0x61, 0x51, 0x49, 0x46 }, // 2
	{ 0x21, 0x41, 0x45, 0x4B, 0x31 }, // 3
	{ 0x18, 0x14, 0x12, 0x7F, 0x10 }, // 4
	{ 0x27, 0x45, 0x45, 0x45, 0x39 }, // 5
	{ 0x3C, 0x4A, 0x49, 0x49, 0x30 }, // 6
	{ 0x01, 0x71, 0x09, 0x05, 0x03 }, // 7
	{ 0x36, 0x49, 0x49, 0x49, 0x36 }, // 8
	{ 0x06, 0x49, 0x49, 0x29, 0x1E }, // 9
	{ 0x00, 0x36, 0x36, 0x00, 0x00 }, // :
	{ 0x00, 0x56, 0x36, 0x00, 0x00 }, // ;
	{ 0x00, 0x08, 0x14, 0x22, 0x41 }, // <
	{ 0x14, 0x14, 0x14, 0x14, 0x14 }, // =
	{ 0x41, 0x22, 0x14, 0x08, 0x00 }, // >
	{ 0x02, 0x01, 0x51, 0x09, 0x06 }, // ?
	{ 0x32, 0x49, 0x79, 0x41, 0x3E }, // @
	{ 0x7E, 0x11, 0x11, 0x11, 0x7E }, // A
	{ 0x7F, 0x49, 0x49, 0x49, 0x36 }, // B
	{ 0x3E, 0x41, 0x41, 0x41, 0x22 }, // C
	{ 0x7F, 0x41, 0x41, 0x22, 0x1C }, // D
	{ 0x7F, 0x49, 0x49, 0x49, 0x41 }, // E
	{ 0x7F, 0x09, 0x09, 0x01, 0x01 }, // F
	{ 0x3E, 0x41, 0x41, 0x51, 0x32 }, // G
	{ 0x7F, 0x08, 0x08, 0x08, 0x7F }, // H
	{ 0x00, 0x41, 0x7F, 0x41, 0x00 }, // I
	{ 0x20, 0x40, 0x41, 0x3F, 0x01 }, // J
	{ 0x7F, 0x08, 0x14, 0x22, 0x41 }, // K
	{ 0x7F, 0x40, 0x40, 0x40, 0x40 }, // L
	{ 0x7F, 0x02, 0x04, 0x02, 0x7F }, // M
	{ 0x7F, 0x04, 0x08, 0x10, 0x7F }, // N
	{ 0x3E, 0x41, 0x41, 0x41, 0x3E }, // O
	{ 0x7F, 0x09, 0x09, 0x09, 0x06 }, // P
	{ 0x3E, 0x41, 0x51, 0x21, 0x5E }, // Q
	{ 0x7F, 0x09, 0x19, 0x29, 0x46 }, // R
	{ 0x46, 0x49, 0x49, 0x49, 0x31 }, // S
	{ 0x01, 0x01, 0x7F, 0x01, 0x01 }, // T
	{ 0x3F, 0x40, 0x40, 0x40, 0x3F }, // U
	{ 0x1F, 0x20, 0x40, 0x20, 0x1F }, // V
	{ 0x7F, 0x20, 0x18, 0x20, 0x7F }, // W
	{ 0x63, 0x14, 0x08, 0x14, 0x63 }, // X
	{ 0x03, 0x04, 0x78, 0x04, 0x03 }, // Y
	{ 0x61, 0x51, 0x49, 0x45, 0x43 }, // Z
	{ 0x00, 0x00, 0x7F, 0x41, 0x41 }, // [
	{ 0x02, 0x04, 0x08, 0x10, 0x20 }, // "\"
	{ 0x41, 0x41, 0x7F, 0x00, 0x00 }, // ]
	{ 0x04, 0x02, 0x01, 0x02, 0x04 }, // ^
	{ 0x40, 0x40, 0x40, 0x40, 0x40 }, // _
	{ 0x00, 0x01, 0x02, 0x04, 0x00 }, // `
	{ 0x20, 0x54, 0x54, 0x54, 0x78 }, // a
	{ 0x7F, 0x48, 0x44, 0x44, 0x38 }, // b
	{ 0x38, 0x44, 0x44, 0x44, 0x20 }, // c
	{ 0x38, 0x44, 0x44, 0x48, 0x7F }, // d
	{ 0x38, 0x54, 0x54, 0x54, 0x18 }, // e
	{ 0x08, 0x7E, 0x09, 0x01, 0x02 }, // f
	{ 0x08, 0x14, 0x54, 0x54, 0x3C }, // g
	{ 0x7F, 0x08, 0x04, 0x04, 0x78 }, // h
	{ 0x00, 0x44, 0x7D, 0x40, 0x00 }, // i
	{ 0x20, 0x40, 0x44, 0x3D, 0x00 }, // j
	{ 0x00, 0x7F, 0x10, 0x28, 0x44 }, // k
	{ 0x00, 0x41, 0x7F, 0x40, 0x00 }, // l
	{ 0x7C, 0x04, 0x18, 0x04, 0x78 }, // m
	{ 0x7C, 0x08, 0x04, 0x04, 0x78 }, // n
	{ 0x38, 0x44, 0x44, 0x44, 0x38 }, // o
	{ 0x7C, 0x14, 0x14, 0x14, 0x08 }, // p
	{ 0x08, 0x14, 0x14, 0x18, 0x7C }, // q
	{ 0x7C, 0x08, 0x04, 0x04, 0x08 }, // r
	{ 0x48, 0x54, 0x54, 0x54, 0x20 }, // s
	{ 0x04, 0x3F, 0x44, 0x40, 0x20 }, // t
	{ 0x3C, 0x40, 0x40, 0x20, 0x7C }, // u
	{ 0x1C, 0x20, 0x40, 0x20, 0x1C }, // v
	{ 0x3C, 0x40, 0x30, 0x40, 0x3C }, // w
	{ 0x44, 0x28, 0x10, 0x28, 0x44 }, // x
	{ 0x0C, 0x50, 0x50, 0x50, 0x3C }, // y
	{ 0x44, 0x64, 0x54, 0x4C, 0x44 }, // z
	{ 0x00, 0x08, 0x36, 0x41, 0x00 }, // {
	{ 0x00, 0x00, 0x7F, 0x00, 0x00 }, // |
	{ 0x00, 0x41, 0x36, 0x08, 0x00 }, // }
	{ 0x08, 0x08, 0x2A, 0x1C, 0x08 }, // ->
	{ 0x08, 0x1C, 0x2A, 0x08, 0x08 }, // <-
};
void SPI_MasterInit(void) 
{
	DDRB=0xBF;

	SPCR = (1<<SPE)|(1<<MSTR|(1<<SPR0));
	sei();
}

void SPI_MasterTransmit(unsigned char cData) 
{

	SPDR = cData;
	PORTB = (PORTB&0xEF);
	while(!(SPSR & (1<<SPIF))) {
		;
	}
	PORTB = (PORTB|0x10);

}

void WriteData (unsigned char b)
{
	SPI_MasterTransmit(b); // assumes DC (PB1) is high
}

void WriteCmd (unsigned char cmd)
{
	ClearBit(PORTC,0); // 0=command, 1=data
	SPI_MasterTransmit(cmd);
	SetBit(PORTC,0); // return DC high
}

void HardwareReset()
{
	ClearBit(PORTC,1); // pull PB0 (digital 8) briefly low
	_delay_ms(1); // 1 mS is enough
	SetBit(PORTC,1); // return PB0 high
	_delay_ms(200); // wait 200 mS for reset to finish
}

void InitDisplay()
{
	HardwareReset(); // initialize display controller
	WriteCmd(SLPOUT); // take display out of sleep mode
	_delay_ms(150); // wait 150mS for TFT driver circuits
	WriteCmd(COLMOD); // select color mode:
	WriteData(0x05); // mode 5 = 16bit pixels (RGB565)
	WriteCmd(DISPON); // turn display on!
}unsigned long intsqrt(unsigned long val)
// calculate integer value of square root
{
	unsigned long mulMask = 0x0008000;
	unsigned long retVal = 0;
	if (val > 0)
	{
		while (mulMask != 0)
		{
			retVal |= mulMask;
			if ((retVal*retVal) > val)
			retVal &= ~ mulMask;
			mulMask >>= 1;
		}
	}
	return retVal;
}
void WriteWord (int w)
{
	SPI_MasterTransmit(w >> 8); // write upper 8 bits
	SPI_MasterTransmit(w & 0xFF); // write lower 8 bits
}
void Write888 (long data, int count)
{
	byte red = data>>16; // red = upper 8 bits
	byte green = (data>>8) & 0xFF; // green = middle 8 bits
	byte blue = data & 0xFF; // blue = lower 8 bits
	for (;count>0;count--)
	{
		WriteData(red);
		WriteData(green);
		WriteData(blue);
	}
}
void Write565 (int data, unsigned int count)
// send 16-bit pixel data to the controller
// note: inlined spi xfer for optimization
{
	WriteCmd(RAMWR);
	for (;count>0;count--)
	{
		SPDR = (data >> 8); // write hi byte
		while (!(SPSR & 0x80)); // wait for transfer to complete
		SPDR = (data & 0xFF); // write lo byte
		while (!(SPSR & 0x80)); // wait for transfer to complete
	}
}

void SetAddrWindow(byte x0, byte y0, byte x1, byte y1)
{
	WriteCmd(CASET); // set column range (x0,x1)
	WriteWord(x0);
	WriteWord(x1);
	WriteCmd(RASET); // set row range (y0,y1)
	WriteWord(y0);
	WriteWord(y1);
}
void ClearScreen()
{
	SetAddrWindow(0,0,XMAX,YMAX); // set window to entire display
	WriteCmd(RAMWR);
	for (unsigned int i=40960;i>0;--i) // byte count = 128*160*2
	{
		SPDR = 0; // initiate transfer of 0x00
		while (!(SPSR & 0x80)); // wait for xfer to finish
	}
}
// ---------------------------------------------------------------------------
// SIMPLE GRAPHICS ROUTINES
//
// note: many routines have byte parameters, to save space,
// but these can easily be changed to int params for larger displays.
void DrawPixel (byte x, byte y, int color)
{
	SetAddrWindow(x,y,x,y);
	Write565(color,1);
}

// ---------------------------------------------------------------------------
// TEXT ROUTINES
//
// Each ASCII character is 5x7, with one pixel space between characters
// So, character width = 6 pixels & character height = 8 pixels.
//
// In portrait mode:
// Display width = 128 pixels, so there are 21 chars/row (21x6 = 126).
// Display height = 160 pixels, so there are 20 rows (20x8 = 160).
// Total number of characters in portait mode = 21 x 20 = 420.
//
// In landscape mode:
// Display width is 160, so there are 26 chars/row (26x6 = 156).
// Display height is 128, so there are 16 rows (16x8 = 128).
// Total number of characters in landscape mode = 26x16 = 416.
byte curX,curY; // current x & y cursor position
void GotoXY (byte x,byte y)
// position cursor on character x,y grid, where 0<x<20, 0<y<19.
{
	curX = x;
	curY = y;
}
void GotoLine(byte y)
// position character cursor to start of line y, where 0<y<19.
{
	curX = 0;
	curY = y;
}
void AdvanceCursor()
// moves character cursor to next position, assuming portrait orientation
{
	curX++; // advance x position
	if (curX>20) // beyond right margin?
	{
		curY++; // go to next line
		curX = 0; // at left margin
	}
	if (curY>19) // beyond bottom margin?
	curY = 0; // start at top again
}
void SetOrientation(int degrees)
// Set the display orientation to 0,90,180,or 270 degrees
{
	byte arg;
	switch (degrees)
	{
		case 90: arg = 0x60; break;
		case 180: arg = 0xC0; break;
		case 270: arg = 0xA0; break;
		default: arg = 0x00; break;
	}
	WriteCmd(MADCTL);
	WriteData(arg);
}
void PutCh (char ch, byte x, byte y, int color)
// write ch to display X,Y coordinates using ASCII 5x7 font
{
	int pixel;
	byte row, col, bit, data, mask = 0x01;
	SetAddrWindow(x,y,x+4,y+6);
	WriteCmd(RAMWR);
	for (row=0; row<7;row++)
	{
		for (col=0; col<5;col++)
		{
			data = pgm_read_byte(&(FONT_CHARS[ch-32][col]));
			bit = data & mask;
			if (bit==0) pixel=BLACK;
			else pixel=color;
			WriteWord(pixel);
		}
		mask <<= 1;
	}
}
void WriteChar(char ch, int color)
// writes character to display at current cursor position.
{
	PutCh(ch,curX*6, curY*8, color);
	AdvanceCursor();
}
void WriteString(char *text, int color)
// writes string to display at current cursor position.
{
	for (;*text;text++) // for all non-nul chars
	WriteChar(*text,color); // write the char
}

enum RFID_States { SM1_Validate, SM1_Wait, SM1_Verify, SM1_Wait2 };
int TickFct_RFID(int state) {
static unsigned char wait = 0;
static unsigned char wait2 = 0;

	switch(state) { // Transitions
		case -1: // Initial transition
		// Initialization behavior
		state = SM1_Validate;
		break;

		case SM1_Validate:
		state = SM1_Wait;
		break;

		case SM1_Wait:
		if (wait < 10)
		{
			state = SM1_Wait;
		}
		else
		{
			wait = 0;
			state = SM1_Verify;
		}
		break;

		case SM1_Verify:
		state = SM1_Wait2;
		break;

		case SM1_Wait2:
		if (wait2 < 10)
		{
			state = SM1_Wait2;
		}
		else
		{
			wait2 = 0;
			state = SM1_Validate;
		}
		break;

		default:
		state = -1;
	}

	switch(state) { // State actions
		case SM1_Validate:
		if (USART_HasReceived(0))
		{
			ClearScreen();
			for (unsigned char i = 0; i < 16; ++i)
			{
				temp = USART_Receive(0);
				if (i > 3)
				{
					input[i - 4] = temp;
				}
			}
			GotoXY(0,0);
			for (unsigned char i = 0; i < 12; ++i)
			{
				WriteChar(input[i], WHITE);
			}
		}
		break;

		case SM1_Wait:
		++wait;
		break;

		case SM1_Verify:
		for (unsigned char i = 0; i < 12; ++i)
		{
			if (card[i] != input[i])
			{
				valid = 0;
			}
			else
			{
				valid = 1;
			}
		}
		if (input[0] != '0')
		{
			if (valid)
			{
				GotoLine(5);
				WriteString("Access Granted", LIME);

				if (USART_IsSendReady(1))
				{
					USART_Send(0x01, 1);
				}
			}
			else
			{
				GotoLine(5);
				WriteString("Access Denied", RED);

				if (USART_IsSendReady(1))
				{
					USART_Send(0x02, 1);
				}
			}
		}
		break;

		case SM1_Wait2:
		++wait2;
		break;

		default:
		break;
	}
	return state;
}

int main(void)
{
	DDRB = 0xFF; PORTB = 0x00;
	DDRC = 0xFF; PORTC = 0x00;
	SPI_MasterInit();
    InitDisplay();
	SetOrientation(180);
	initUSART(0);
	initUSART(1);
	ClearScreen();

	tasksNum = 1; // declare number of tasks
	task tsks[1]; // initialize the task array
	tasks = tsks; // set the task array
	
	// define tasks
	unsigned char i=0; // task counter
	tasks[i].state = -1;
	tasks[i].period = 200;
	tasks[i].elapsedTime = tasks[i].period;
	tasks[i].TickFct = &TickFct_RFID;
	
	TimerSet(200);
	TimerOn();
    while (1) 
    {	
    }
}

