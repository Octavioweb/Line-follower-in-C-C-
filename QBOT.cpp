/*Begining of Auto generated code by Atmel studio */
/*End of auto generated code by Atmel studio */


#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Ticker.h>
#include <avr/io.h>
#include <avr/interrupt.h>

asm("#include <WS2812.S>");


#define SW_Tx 10     //Arduino Digital signals (10)  PB1
#define SW_RX 9      //Arduino Digital signals  (9)  PB2
#define infrarojo_d  11
#define infrarojo_i  12

#define infrarrojo_frontal_izquierda 26
#define infrarrojo_frontal_derecha 25

// SE TIENE LOS PINES PC4, PC5. ESTOS SON 18 Y 19


#define motor_i  6
#define motor_d  5
#define dir_motor_i  8
#define dir_motor_d  7
#define velocidad 70


char n_intersection = 0;
bool Qbot_intersection = false;

char Qbot_ruta_prueba2[15] = {'a', 'H', 'H', 'H', 'a', 'a', 'a', 'H', 'a', 'a', 'H', 'H', 'a' };

char Qbot_ruta_hasta_atras [15] = { 'a', 'a', 'a', 'a', 'H', 'H', 'H', 'a', 'i', 'X'};


//QBOT3
char Qbot_ruta_qbot [40] = {'a', 'a', 'a', 'a', 'H', 'H', 'H', 'H', 'H', 'H', 'H', 'H', 'H', 'H', 'H', 'H','a', 'a', 'H', 'H', 'H', 'H', 'H', 'H', 'H', 'H', 'H', 'H', 'H', 'H', 'a', 'i', 'X'};

int start_once =0;

///////////////////////////////////
///////////////////////////////////

int Blink_R = 2;
extern "C" void RGB_BYTE(unsigned char);
//timer
double nOverflow = 0;
float Tdeseado = 0.01;
int counter_overflows = 0;
int timer_on = 0;
char var = 0x01;
int step = 1;
int increase = 5;
int changing = 1; // Give 100 overflows for color to change a little bit
int counter_rgb = 0;

int count = 0;
///////////
//////////

void SEGUIDOR(void);
void principal_rgb(void);
void principal_ultrasonico(void);
void BLUETOOTH(void);
void Left(void);

 
Ticker principal_rgb2 (principal_rgb, 20, 0, MILLIS);
Ticker principal_ultrasonico_2 (principal_ultrasonico, 10, 0, MILLIS);
Ticker BLUETOOTH2 (BLUETOOTH, 10, 0, MILLIS);
Ticker Left2 (Left, 60, 0, MILLIS);


/////////
/////////
///////

int rotaciones = 11;

//////////
//////////
//////////////
void Qbot_adelante_lento(void);
void Qbot_adelante(void);
void Qbot_derecha(void);
void Qbot_izquierda(void);
void Qbot_detenerse(void);
void Qbot_acciones(char c);
void Qbot_izquierda_lento(void);
////////////////////
///////////////////
/////////////////

//CODIGOS LUCES RGB
int initial = 0;
int light = 0x80;
char color = 2;
int downwards = 1;
extern "C" void RGB_BYTE(unsigned char);
short int iters = 0;
int counter = 0;

//////////
/////////////////
//////////////////////
/////CODIGO ULTRASÓNICO

#include <avr/io.h>
#define F_CPU 16000000
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/io.h>
#define F_CPU 16000000UL
#include "stdio.h"
#include "StringArray.h"
#include "util/delay.h"


#define CLEAR(PORT,BIT) PORT&=~(1<<BIT)     /* Clear bit */
#define SET(PORT,BIT)   PORT|=(1<<BIT)      /* Set bit y */
#define CHECKBIT(PORT,BIT) (PORT & (BIT))

short int intersection_count =0;
#define clockPin PORTC0
extern void RGB_BYTE(unsigned char);
float distancia;
float tiempo;
double dutycycle = 0;
int count2 = 0;
int temp;
unsigned char T;
int tam=0;
unsigned char buffer[16] ;

#define clockPin PORTC0
int counter_ultra = 0;
char obstaculo = 1;

/////////////////////
////////////////////
///////////////////
////BLUETOOTH
#include <avr/io.h>
#include <avr/interrupt.h>
char ruta_temp [15] = {'a', 'a', 'a','a', 'a', 'a','a', 'a', 'a','a'};

#include "TM1640.h"
#include "StringArray.h"
#include "Serial.h"
char temp2;
char p2=0;
char i2=0;
char j2=0;
bool once = false;
int bt = 0;
String pre_msg;


//char msg_azul[20] =
static char *Mensaje = "Listo";
char buffer2[15];
char Memoria[150]={0x0D};
char qbot_esperando = 0;
char ind_buffer;
int max_rotaciones = 50;

String mensaje_bt = "0010";

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////AQUÍ ESTOY

String mensaje;
SoftwareSerial mySerial(SW_RX, SW_Tx); // RX, TX
TM1640 TM1640A;
HW_Serial HW_SerialA;

//////////////////
///////////////////
ISR(USART_RX_vect)
{
	HW_SerialA.ISR_RX_vect();
}

ISR(USART_TX_vect)
{
	HW_SerialA.ISR_TX_vect();
}

ISR(TIMER2_OVF_vect){
	SEGUIDOR();
}

void setup() {
  // put your setup code here, to run once:
                        //Timer 2 as CTC
    TCCR2B = 0b00000110; //Timer 2 1024 prescaler
    TCNT2 = 0;	                     //Value for 10ms
    TIMSK2 =  (1<<TOIE2);                   //Timer 2 OCM A Interrupt Enable
    
	mySerial.begin(9600);
    mySerial.listen();
	////////////////////////
	pinMode(infrarojo_d, INPUT);
	pinMode(infrarojo_i, INPUT);
	pinMode(infrarrojo_frontal_derecha, INPUT);
	pinMode(infrarrojo_frontal_izquierda, INPUT);
	
	pinMode(motor_d, OUTPUT);
	pinMode(motor_i, OUTPUT);
	pinMode(dir_motor_d, OUTPUT);
	pinMode(dir_motor_i, OUTPUT);
	
	///////////////////
	/////////////////////
	///////ULTRASONICO/////
	TCCR1A = 0x00 ;
	TCCR1B |= (1<<CS11)|(1<<CS10) ;
	TIMSK1 = 0b00000001;
	
	
	SET(DDRC,clockPin);   //CLK of TM1640 as OutPut
	SET(DDRC,PORTC1);   //DINTM1640 as Output
	SET(PORTC,clockPin);   //CLK of TM1640 as OutPut
	SET(PORTC,PORTC1);   //DIN of TM1640 as Output
	SET(DDRD,PIND2);
	
	DDRD |= (1<<DDC3);
	DDRD &=~ (1<<DDC4);
	
	PORTD &=~ ((1<<PIND4)|(1<<PIND3));//DIR;
	PORTD |= (1 <<PIND3);
	//TCCR1A = 0x00 ;
	//TCCR1B |= (1<<CS11)|(1<<CS10) ;
	
//////////////////////////
//////////////////////
///////////BLUETOOTH
	DDRD |= (1<<PIND0); // PARA LUCES RGB
	pinMode(SW_RX, INPUT);
	pinMode(SW_Tx, OUTPUT);
	mySerial.begin(9600);
	mySerial.listen();
	HW_SerialA.starUpSerial();
	qbot_esperando = 0;

	//starUpSerial();
	sei();
	
	mensaje_bt = "0010";
	
	//SEGUIDOR2.start();
	principal_rgb2.start();
	principal_ultrasonico_2.start();
	BLUETOOTH2.start();
	Left2.start();	
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

void loop() {
	
	//SE LE A;ADE UN	/r/n	PARA SOLUCIONAR COMENTARIOS FINALES DE MENSAJE BLUETOOTH
	#define esperar_nueva_solicitud "1111"
	#define solicitud_amarillo "0001"
	#define solicitud_azul "0010"
	#define solicitud_verde "0011"
	
	#define solicitud_temp "0100"
	#define solicitud_temp_2 "0101"
	
	
	// MENSAJE PARA COMPARAR ANTERIOR MENSAJE A NUEVO
	// EN CASO  DE SER NUEVO, SE CORRE EL CODIGO PARA PROGRAMAR. 
	// EN LO CONTRARIO, NO SE PROGRAMA.
	principal_ultrasonico_2.update();
	Left2.update();
	principal_rgb2.update();
	////
	DDRD &= ~(1<PIND0);
	UCSR0B |= ((1<<RXEN0));


	
	
	if (bt++ == 1000){
		BLUETOOTH2.update();
		bt = 0;	
	}
	
	
	if (mensaje_bt != pre_msg){
	
		
		
		n_intersection = 0;
	
		if (mensaje_bt == "none"){
			qbot_esperando = 1;
			
		}
		else if (mensaje_bt == "suicidio\r\n"){
			cli();
		}
		else if (mensaje_bt == esperar_nueva_solicitud){ //FUNCIÓN PARA PARAR
			qbot_esperando = 1;
			mensaje_bt = "none";
		
		}
	
		else if (mensaje_bt == solicitud_azul){ //IZQUIERDA, CUBO AZUL
			//mySerial.write("OK_AZUL");
			qbot_esperando = 0;
			
				char buffer2[15] = "Cubo azul\r\n";
				//mySerial.print(buffer2);
				Matrix(Memoria,buffer2);	
			
		}
	
		else if (mensaje_bt == solicitud_amarillo){ //DERECHO, CUBO AMARILLO
			//mySerial.write("OK_AMARILLO");
			qbot_esperando = 0;
			
			char buffer2[18] = "Cubo amarillo\r\n";
			//mySerial.print(buffer2);
			Matrix(Memoria,buffer2);
		}
	
		else if (mensaje_bt == solicitud_verde){ //DERECHA, CUBO VERDE
			//mySerial.write("OK_VERDE");
			qbot_esperando = 0;
			
			char buffer2[15] = "Cubo verde\r\n";
			//mySerial.print(buffer2);
			Matrix(Memoria,buffer2);
		}
	
		
			}
	
	pre_msg = mensaje_bt;
	//mySerial.println(mensaje_bt);

		//SEGUIDOR2.update();
}


void BLUETOOTH (void){
	i2=0;
	ind_buffer =0;
	while(mySerial.available()>0)  //Send from SoftwareSerial (mySerial) to CoolTerm
	{	
		temp2 =  mySerial.read();
		mensaje += temp2;
		buffer2[ind_buffer++]=temp2;
		
		once = false;
		
		}
		
		if(mensaje.length() > 0 && once == false){
			//mySerial.print(buffer2);
			mensaje_bt = mensaje;
			mensaje = "";
			once = true;
		}
			
}

void SEGUIDOR(){
	
	if (qbot_esperando == 1){
		color=4;
		return;
	}
	else{
		color = 2;

	}
	
	if (obstaculo == 1){
		Qbot_detenerse();
				color=1;

		return;
	}
	if (Qbot_intersection == true){
		rotaciones--;
		if (rotaciones > (int)max_rotaciones/2){
			Qbot_derecha();
			return;
		}
		else if (rotaciones < 20){
			n_intersection++;
			//mySerial.write(n_intersection);
			
			Qbot_intersection = false;
			intersection_count = 0;
			if (n_intersection > 50)
				n_intersection = 0;
		}
			
		}
	if(digitalRead(infrarojo_d) == LOW && digitalRead(infrarojo_i) == LOW) {
		Qbot_adelante();
		intersection_count=0;
	
		
	}
	else if (digitalRead(infrarojo_d) == HIGH && digitalRead(infrarojo_i) == LOW){
		Qbot_derecha();
		intersection_count=0;
	}
	else if (digitalRead(infrarojo_d) == LOW && digitalRead(infrarojo_i) == HIGH){
		Qbot_izquierda();
		intersection_count=0;
			
	}
	
	else if(digitalRead(infrarojo_d) == HIGH && digitalRead(infrarojo_i) == HIGH){
		
		if (Qbot_intersection == true){
			Qbot_derecha();
			return;
			
		}
		
		if (intersection_count++ > 30){
			// CON 20 COMO VALOR, FUNCIONA BIEN. PERO PUEDE FALLAR. SE PONE TRIPLE CINTA, Y SE PONE 25 (30)
			
			Qbot_intersection = true;
			if (rotaciones < 10){
				rotaciones += max_rotaciones;
			}
			
		}
						
		Qbot_derecha();
		//mySerial.write(intersection_count);
		//mySerial.write(rotaciones);
		}
	
}
void principal_ultrasonico (void){
	pinMode(infrarrojo_frontal_derecha, INPUT);
	pinMode(infrarrojo_frontal_izquierda, INPUT);
	if (qbot_esperando == 1)
		return;
	PORTD |= (1 <<PIND3);

	_delay_us(10);
	PORTD &=~ (1<<PIND3);
	//UART_TxChar('1');
	while(((PIND>>PIND4)&1)==0);
	TCNT1 = 0;
	//UART_TxChar('2');
	while (((PIND>>PIND4)&1)==1) ;
	//UART_TxChar('3');
	temp  = TCNT1;
	tiempo = (float)(temp)*1024.0*(1.0/16000000.0);
	distancia = tiempo*330.0*(1/2.0)*10;
	//distancia = (float)(temp)*8*0.040;
	/*for (int x= 0; x < 16; x++){
		UART_TxChar(buffer[x]);
		_delay_ms(100)
	}*/
	
	
	
	if(distancia < 25 | !(digitalRead(infrarrojo_frontal_derecha) == LOW)){
		
		
		obstaculo = 1;
		
		if(PORTD&(1<<PIND2)){
			PORTD&=~(1<<PIND2); //TOGGLE THE PORTD2
		}
		else {
			PORTD|=(1<<PIND2);
		}
		
		}
	else
		obstaculo = 0;
		

			}



//UTILIZA la variable color
void principal_rgb (void){
	
	DDRD |= (1<<PIND0);
	PORTD &=~ (1<<PIND0);
	UCSR0B &=~ ((1<<RXEN0));
	
	if (downwards == 1){
		light = (light>>1);
		
		if (light == 0x01){
			downwards = 0;
			light = 0x01;	
		}
	}
	else{
		light = (light<<1);
		if (light == 0x80){
			light = 0x80;
			downwards = 1;
		}
	}
	switch (color)
	{
		
		/*
		1 = ROJO - PARARSE
		2 = AZUL - AVANZANDO
		3 = NARANJA - ESPERANDO EN ESTACIÓN DE CARGA O DESCARGA
		4 = VERDE - ESPERANDO SEÑAL BLUETOOTH
		*/
		
		case 1: //Luz roja al detectar obstáculo
		
		RGB_BYTE(0);
		RGB_BYTE(light-1);
		RGB_BYTE(0);
		
		RGB_BYTE(0);
		RGB_BYTE(light-1);
		RGB_BYTE(0);
		break;
		
		case 2: //Luz azul cuando el QBOT está avanzando
		RGB_BYTE(0);
		RGB_BYTE(0);
		RGB_BYTE(light-1);
		
		RGB_BYTE(0);
		RGB_BYTE(0);
		RGB_BYTE(light-1);
		break;
		
		case 3: //Luz naranja cuando QBOT está en estación de descarga o carga
		RGB_BYTE(light-1);
		RGB_BYTE(0x80);
		RGB_BYTE(0);
		
		RGB_BYTE(light-1);
		RGB_BYTE(0x80);
		RGB_BYTE(0);
		break;
		
		case 4: //Luz verde cuando QBOT está a la espera de instrucciones
		RGB_BYTE(light);
		RGB_BYTE(0);
		RGB_BYTE(0);
		
		RGB_BYTE(light);
		RGB_BYTE(0);
		RGB_BYTE(0);
		break;
		
	} 
}



void Qbot_adelante(void){
	digitalWrite(dir_motor_i,LOW);
	analogWrite(motor_i,velocidad);
	digitalWrite(dir_motor_d,LOW);
	analogWrite(motor_d,velocidad);
}
void Qbot_adelante_lento(void){
	digitalWrite(dir_motor_i,LOW);
	analogWrite(motor_i,10);
	digitalWrite(dir_motor_d,LOW);
	analogWrite(motor_d,10);
}

void Qbot_derecha(void){
	digitalWrite(dir_motor_i,HIGH);
	analogWrite(motor_i,0);
	digitalWrite(dir_motor_d,LOW);
	analogWrite(motor_d,velocidad+10);
}

void Qbot_izquierda(void){
	digitalWrite(dir_motor_i,LOW);
	analogWrite(motor_i,velocidad+10);
	digitalWrite(dir_motor_d,HIGH);
	analogWrite(motor_d,0);
}
void Qbot_izquierda_lento(void){
	digitalWrite(dir_motor_i,LOW);
	analogWrite(motor_i,15);
	digitalWrite(dir_motor_d,HIGH);
	analogWrite(motor_d,velocidad);
}

void Qbot_detenerse(void){
	digitalWrite(dir_motor_i,LOW);
	analogWrite(motor_i,0);
	digitalWrite(dir_motor_d,LOW);
	analogWrite(motor_d,0);
}
void Qbot_acciones(char c){
	switch(c)
	
	{
		
		////// SE INVIRTI[O EL SENTIDO DE IZQUIERA Y DERECHA
		
		case 'X': //adelante
		mensaje_bt = "none";
		rotaciones = 11;
		break;
		case 'a': //adelante
		Qbot_adelante();
		break;
		case 'a2':
		Qbot_adelante_lento();
		case 'i': //derecha
		Qbot_derecha();
		break;
		
		case 'd':
		Qbot_izquierda();
		break;
		
		case 'i2':
		Qbot_izquierda_lento();
		break;
		
		case 'e':
		Qbot_detenerse();
		break;
		
		case 'H':
		//if (rotaciones > max_rotaciones/1.5){
			//Qbot_adelante();
		//}
		//else{
			Qbot_detenerse();
			color = 3;
		
		break;
			}
}

void Left()   //Dynamic Message subroutine
{
	//mySerial.println(n_intersection);
	
	for (unsigned char m=0;m<16;m++)	
	{
		if(Memoria[p2+m]==0x0D)
		{
			p2=0;
			
			
		}
		TM1640A.sendData(m, Memoria[p2+m]);
	}
	p2+=1;
}