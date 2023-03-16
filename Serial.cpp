/*
 * Serial.c
 *
 * Created: 10/9/2022 12:12:21 PM
 *  Author: Israel Macias H.
 */ 
#include <avr/io.h>
#include "Serial.h"


void HW_Serial::starUpSerial(void)
{
        UCSR0B |= (1 << RXEN0) | (1 << TXEN0);    // Turn on transmission and reception 
        UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);  // Use 8-bit character sizes
        UBRR0L = 208;		                      // Load lower 8-bits of the baud rate value
                                                  // Baud = 9600
        UCSR0A |= (1<<U2X0);                      // Double Speed Operation
        UCSR0B |= (1<<RXCIE0) | (1<<TXCIE0);      // Enable Rx & Tx Interrupt
        DDRD |= (1<<PIND1);                       // TXD as Output
       
}

unsigned char HW_Serial::UART_RxChar(void)
{
    while ((UCSR0A & (1 << RXC0)) == 0);          // Wait till data is received 
    return(UDR0);			                      // Return the byte
}

void HW_Serial::UART_TxChar(char ch)
{
   if(Tx_wr_index==0) UDR0 = ch;
   else
   {
       Tx_Buffer[Tx_wr_index]=ch;
       Tx_wr_index+=1;
       if(Tx_wr_index>TX_BUFFER_SIZE)
         Tx_wr_index=0;
   }
}

void HW_Serial::ISR_TX_vect(void)
{
    if(Tx_Index<=Tx_wr_index)
    UDR0 = Tx_Buffer[Tx_Index++];
    if(Tx_Index>TX_BUFFER_SIZE)
      Tx_Index=0;   
}    

void HW_Serial::ISR_RX_vect(void)
{              
        data=UDR0;
        Rx_Buffer[rx_wr_index++]=data;
        if (rx_wr_index == RX_BUFFER_SIZE) rx_wr_index=0;
        if (data == 0x0D)                             //Detect ENTER KEY
        {
            Rx_Flag=1;
            rx_wr_index = 0;
        } 
}

