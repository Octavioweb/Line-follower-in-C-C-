/*
 * Serial.c
 *
 * Created: 10/9/2022 12:12:21 PM
 *  Author: Israel Macias H.
 */ 
#include <avr/io.h>
#include <avr/interrupt.h>
#include "Serial.h"



void starUpSerial(void)
{
        UCSR0B |= (1 << RXEN0) | (1 << TXEN0);    // Turn on transmission and reception 
        UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);  // Use 8-bit character sizes
        UBRR0L = 17;		                      // Load lower 8-bits of the baud rate value
                                                  // Baud = 115200
        UCSR0A |= (1<<U2X0);                      // Double Speed Operation
    //  UCSR0B |= (1<<RXCIE0);                    // Enable Rx Interrupt          
}

unsigned char UART_RxChar(void)
{
    while ((UCSR0A & (1 << RXC0)) == 0);          // Wait till data is received 
    return(UDR0);			                      // Return the byte
}

void UART_TxChar(char ch)
{
    while (! (UCSR0A & (1<<UDRE0)));	          // Wait for empty transmit buffer*/
    UDR0 = ch ;
}

ISR(USART_RX_vect)
{
    unsigned char data;
    static unsigned char rx_wr_index=0;
    
    data=UDR0;
    Rx_Buffer[rx_wr_index++]=data;
    if (rx_wr_index == RX_BUFFER_SIZE) rx_wr_index=0;
    if (data == 0x0D)                             //Detect ENTER KEY
    {
        Rx_Flag=1;
        rx_wr_index = 0;
    }   
}

ISR(USART_TX_vect)
{

}