/*
 * CFile1.c
 *
 * Created: 9/8/2022 9:55:00 AM
 *  Author: Israel Macias H. V1.0
 */ 

#include "TM1640.h"
#include <avr/io.h>
#define F_CPU 16000000UL
#include "util/delay.h"

#include "TM1640.h"

void TM1640::setData(unsigned char data)
{
    unsigned char temp;
    CLEAR(port, dataPin);
    CLEAR(port, clockPin);
    
      for (unsigned char i = 0; i < 8; i++)
       {
           temp = data&1;
           data = (data>>1);
        CLEAR(port, clockPin);
           if(temp) SET(port,dataPin);         
           else     CLEAR(port,dataPin);        
           _delay_us(1);           
        SET(port, clockPin);
          _delay_us(1);
       }
         CLEAR(port, clockPin);
         CLEAR(port, dataPin);
         _delay_us(1);
         SET(port, clockPin);
         SET(port, dataPin);
}
void TM1640::sendData(unsigned char addres, unsigned char data)  
{
     setupDisplay(1, 15);      //On display, and Set pulse width 14/16
     setData(0x44);            //Datta command(0100)<==Fixed address(0100)
     send(0xC0 | addres);      //Address Command (11)<==Display Address(0x0H to 0x0FH)
     send(data);               //Dots in addres
     SET(port, clockPin);
     SET(port, dataPin);
      
}
//Display control command setting
void TM1640::setupDisplay(unsigned char active, unsigned char intensity)
{
   unsigned char temp = 0x80;
   if(active) temp|= 0x08;
   temp|=intensity&0x0F;
   send(temp);
}
//Display Clear 
void TM1640::clearDisplay()
{
        SET(DDRC,clockPin);   //CLK of TM1640 as OutPut
        SET(DDRC,dataPin);   //DIN of TM1640 as Output
        SET(PORTC,clockPin);   //CLK of TM1640 as OutPut
        SET(PORTC,dataPin);   //DIN of TM1640 as Output
        SET(DDRD,PIND2);
        
         setupDisplay(1, 15);
         setData(0x40);        //Datta command(0100)<==Address auto + 1 (0000)
         send(0xC0);      //Address Command (11)<==Display Address(0x0H))
     for (int i = 0; i < 16; i++) 
     {
         send(0x00);
      //   SET(port, clockPin);
      //   SET(port, dataPin);
     }         
}
//Send one byte bit by bit starting whit LSB 
void TM1640::send(unsigned char data)
{
    unsigned char temp;
    CLEAR(port, dataPin);
    CLEAR(port, clockPin);
    
    for (unsigned char i = 0; i < 8; i++)
    {
        temp = data&1;
        data = (data>>1);
        CLEAR(port, clockPin);
        if(temp) SET(port,dataPin);
        else     CLEAR(port,dataPin);
        _delay_us(1);
        SET(port, clockPin);
        _delay_us(1);
    }
    CLEAR(port, clockPin);
    CLEAR(port, dataPin);
}
//Send 16 bytes starting in the address 00H (GRID1)
void TM1640::sendString(unsigned char *data)
{
         setupDisplay(1, 15);      //On display, and Set pulse width 14/16
         setData(0x40);            //Datta command(0100)<==Addres auto+1(0000)
         send(0xC0);               //Address Command (11)<==Display Address(0x0H to 0x0FH)
         for( char i=0;i<16;i++)
         send(*data++);            //Send 16 bytes 
         SET(port, clockPin);
         SET(port, dataPin);
}