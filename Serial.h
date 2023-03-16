/*
 * Serial.h
 *
 * Created: 10/9/2022 12:11:47 PM
 *  Author: Israel Macias H.
 */ 


#ifndef SERIAL_H_
#define SERIAL_H_

#define RX_BUFFER_SIZE 31
#define TX_BUFFER_SIZE 31

class HW_Serial
{  
 public:
    volatile char Rx_Flag;
    char Rx_Buffer[RX_BUFFER_SIZE];
    void starUpSerial(void);
    unsigned char UART_RxChar(void);
    void UART_TxChar(char ch);
    void ISR_TX_vect(void);
    void ISR_RX_vect(void);
 private:
    char Tx_Index=0;
    unsigned char data;
    unsigned char rx_wr_index=0;
    char Tx_wr_index=0;
    char Tx_Buffer[TX_BUFFER_SIZE];   
};



#endif /* SERIAL_H_ */