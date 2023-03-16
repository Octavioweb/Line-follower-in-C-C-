/*
 * TM1640.h
 *
 * Created: 10/9/2022 11:31:43 AM
 *  Author: macias
 */ 

#ifndef TM1640_H_
#define TM1640_H_

#define CLEAR(PORT,BIT) PORT&=~(1<<BIT)     /* Clear bit */
#define SET(PORT,BIT)   PORT|=(1<<BIT)      /* Set bit y */
#define CHECKBIT(PORT,BIT) (PORT & (BIT))   /* Check bit BIT in byte PORT*/

#define clockPin PORTC0
#define dataPin  PORTC1
#define port     PORTC

class TM1640  
{
 private:
    void setupDisplay(unsigned char active, unsigned char intensity);
    void send(unsigned char data);
    void setData(unsigned char data);
 public:
    void clearDisplay(void);
    void sendData(unsigned char addres, unsigned char data);
    void sendString(unsigned char *data);
    
};
    






#endif /* TM1640_H_ */