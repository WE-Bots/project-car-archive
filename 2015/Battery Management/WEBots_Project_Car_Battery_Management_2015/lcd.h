//LCD Functions Developed by electroSome
#include <xc.h>
#include <stdio.h> // allows access to sprintf

void LCD_Port(char a)
{
	if(a & 16)
		D4 = 1;
	else
		D4 = 0;

	if(a & 32)
		D5 = 1;
	else
		D5 = 0;

	if(a & 64)
		D6 = 1;
	else
		D6 = 0;

	if(a & 128)
		D7 = 1;
	else
		D7 = 0;
}

void pulse()
{
    EN = 1;
    __delay_ms(1);          // enable pule width >= 300ns
    EN = 0;                 // clock enable -> falling edge
}

// first 4 bit number to be sent is in the high order 4 bits and the
// second 4 bit number to be sent is in the low order 4 bits
void LCD_Cmd(char a)
{
	LCD_Port(a);        // put the data on the output port
        RS = 0;             // RS = 0, send instruction
        RW = 0;             // RW = 0, write
        pulse();            // send the lower 4 bits
        a = a<<4;           // shift to access the higher order bytes
	LCD_Port(a);        // put the data on the output port
        pulse();            // send the upper 4 bits
}

// 4 bit instruction to be sent is in the high order bit of a
void LCD_Cmd_4bit(char a)
{
	LCD_Port(a);        // put the data on the output port
        RS = 0;             // RS = 0, send instruction
        RW = 0;             // RW = 0, write
        pulse();            // send the lower 4 bits
}

void LCD_Init()
{
    LCD_Port(0);
    __delay_ms(100);         // wait for at least 15ms after power is applied
    LCD_Cmd_4bit(0x30);      // function set
    __delay_ms(100);
    LCD_Cmd(0x28);           // function set
    __delay_ms(100);
    LCD_Cmd(0x28);           // function set
    __delay_ms(100);
    LCD_Cmd(0x0F);           // Display on/off control
    __delay_ms(100);         // display on, cursor on, blink on
    LCD_Cmd(0x01);           // Display clear
    __delay_ms(100);
    LCD_Cmd(0x06);           // entry mode set
    __delay_ms(100);
}

void LCD_Write_Char(char a)
{
    LCD_Port(a);        // put the data on the output port
    RS = 1;             // RS = 0, send data
    RW = 0;             // RW = 0, write
    pulse();            // send the lower 4 bits
    a = a<<4;           // shift to access the higher order bytes
    LCD_Port(a);        // put the data on the output port
    pulse();            // send the upper 4 bits
}


// for first line specify high order bits of a to 0
// and for second line specify high order bits of a to 1
// the location along the line is specified by a [0 - F]
void LCD_Set_Cursor(char a )
{
    if (a & 16) // second line of LCD
    {
       LCD_Cmd_4bit(0xC0);
       a = a << 4;
       LCD_Cmd_4bit(a);
    }

    else        // first line of LCD
    {
       LCD_Cmd_4bit(0x80);
       a = a << 4;
       LCD_Cmd_4bit(a);
    }
}

void LCD_Write_String(char *a)
{
    for(int i=0;a[i]!='\0';i++)
    {
       LCD_Write_Char(a[i]);
    }
}

void LCD_Write_Int(int a)
{
    char s[5];

    sprintf(s, "%d", a);
    
    LCD_Write_String(&s);
}

void LCD_Write_Float(float a)
{
    char s[5];

    sprintf(s, "%f", a);

    LCD_Write_String(&s);
}

void LCD_Clear()
{
    LCD_Cmd(0x20);
    LCD_Set_Cursor(0x00);
}