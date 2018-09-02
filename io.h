/******************************************************
 |			 #########   #########	 #########
 |					 #       #       #   #
 | 					 #       #       #   #
 |					 #       #########	 ######
 |					 #       #  #        #
 |					 #       #     #     #
 |				 	 #       #       #   #
 | 			Created: 20-Aug-17 12:42:33 AM
 *******************************************************/
// ATMEL ATMEGA16
//
//                 ______
//      (D  8) PB0 |    |  PA0 (AI 0)
//      (D  9) PB1 |    |  PA1 (AI 1)
//      (D 10) PB2 |    |  PA2 (AI 2)
// PWM0 (D 11) PB3 |    |  PA3 (AI 3)
//      (D 12) PB4 |    |  PA4 (AI 4)
//      (D 13) PB5 |    |  PA5 (AI 5)
//      (D 14) PB6 |    |  PA6 (AI 6)
//      (D 15) PB7 |    |  PA7 (AI 7)
//           RESET |    |  AREF
//             VCC |    |  GND
//             GND |    |  AVCC
//           XTAL2 |    |  PB7 (D 23)
//           XTAL1 |    |  PC6 (D 22)
//       (D 0) PD0 |    |  PC5 (D 21)
//       (D 1) PD1 |    |  PC4 (D 20)
//       (D 2) PD2 |    |  PC3 (D 19)
//       (D 3) PD3 |    |  PC2 (D 18)
// PWM1B (D 4) PD4 |    |  PC1 (D 17)
// PWM1A (D 5) PD5 |    |  PC0 (D 16)
//       (D 6) PD6 |    |  PD7 (D 7) PWM2
//                 ------

/*************************************/

#ifndef F_CPU
#define F_CPU 16000000UL
#endif
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <math.h>
#include <stdlib.h>
void delay(unsigned long);		//simpler form of delay function in avr
void analogWrite(uint8_t, uint8_t);		//PWM function using TIMER 0   ***NOTE:Timer 0 cannot be used if this function is used in the code***
void initADC();
uint8_t analogRead(uint8_t);
void attachInterrupt(int,void*,int);
double map(double,double,double,double,double);
double constrain(double,double,double);
void setup();
void loop();
unsigned int millis();
unsigned long pulseIn(volatile uint8_t , uint8_t );
unsigned long microsecondsToInches(unsigned long );
unsigned long microsecondsToCentimeters(unsigned long );

unsigned long microsecondsToInches(unsigned long mIcroseconds) {
  // According to Parallax's datasheet for the PING))), there are 73.746
  // microseconds per inch (i.e. sound travels at 1130 feet per second).
  // This gives the distance travelled by the ping, outbound and return,
  // so we divide by 2 to get the distance of the obstacle.
  // See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
  return (mIcroseconds*0.00669/ 2);
}

unsigned long microsecondsToCentimeters(unsigned long mIcroseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the object we
  // take half of the distance travelled.
  return (mIcroseconds*0.17/ 2);
}

unsigned long pulseIn(volatile uint8_t pInno, uint8_t vAlue)
{
  TCCR2 = (1 << WGM21) | (1 << COM21) | (1 << FOC2) | (0 << COM20) | (0 << WGM20); //initializing in CTC mode
  TCCR2 = (1 << CS20);
  unsigned long mAxloops = 500000;
  unsigned long wIdth = 0;
  // wait for any previous pulse to end
  while ( ((PIND)&&(pInno)) == vAlue)
	  {
		if (--mAxloops == 0)
		  return 0;
	  }
  // wait for the pulse to start  
  while ( ((PIND)&&(pInno)) != vAlue)
	  {
		if (--mAxloops == 0)
		return 0;
	  }
  // wait for the pulse to stop
  while ( ((PIND)&&(pInno)) == vAlue)
	  {
		if (++wIdth == mAxloops)
		  return 0;
	  }
  return wIdth;
}

const uint8_t OUTPUT=1,INPUT=0;
const uint8_t HIGH=1,LOW=0;
const uint8_t RISING=2,FALLING=3,CHANGE=4;
const uint8_t A=1,B=2,C=3,D=4,lowerNibble=8,higherNibble=9,ALL=10,D4=4,D5=5;
//variables for user interface
/***VARIABLES lowerNibble, higherNibble, ALL ARE FOR SETTING A SET OF BIT AT ONCE***/
void (*cAllisr)(void);		//function pointer used in ISR()

unsigned int millis()
{       
	float l;
	l=x*0.16+0.00000625*TCNT0;
        return l;
}

void USART_Init( unsigned int uBrr)
{
	/*Set baud rate */
	UBRRH = (unsigned char)(uBrr>>8);
	UBRRL = (unsigned char)uBrr;
	/*Enable receiver and transmitter */
        UCSRB = (1<<RXEN)|(1<<TXEN);
}
/* Set frame format: 8data, 2stop bit */
void USART_Transmit( unsigned char data )
{
	/* Wait for empty transmit buffer */
	while ( !( UCSRA & (1<<UDRE)) )
	;
	/* Put data into buffer, sends the data */
	UDR = data;
	_delay_ms(100);


}
unsigned char USART_Receive( void )
{
	/* Wait for data to be received */
	while ( !(UCSRA & (1<<RXC)) )
	;
	/* Get and return received data from buffer */
	return UDR;
}

void Dmilli(int j)
{
	TCCR0|=(1<<WGM01);
	TCCR0|=(1<<CS00);
	TIMSK|=(1<<OCIE0);
	OCR0=255;
	TCNT0=0;
        long int x=31.50*j;
        long	int i;
    for(i=0;i<x;i++)
    {
		while(!(TIFR & (1 << OCF0)))
        {

        }
        TIFR|=(1<<OCF0);
	}
}

void pinMode(uint8_t rEgister,uint8_t bIt, uint8_t mOde)		// eg: bitDefine(A,5,OUTPUT);
{
	if((bIt==lowerNibble)&&(mOde==OUTPUT))		//to set whole INPUTer nibble as output
	{
		switch(rEgister)		//to switch between i/o registers
		{
			case(1):
				DDRA|=0x0F;			//here it is OR'ed to retain the mode of higher nibble
				break;
			case(2):
				DDRB|=0x0F;
				break;
			case(3):
				DDRC|=0x0F;
				break;
			case(4):
				DDRD|=0x0F;
				break;
		}
	}

	else if((bIt==lowerNibble)&&(mOde==INPUT))		//to set whole INPUTer nibble as INPUT
	{
		switch(rEgister)		//to switch between i/o registers
		{
			case(1):
				DDRA&=0xF0;			//here it is ANDed to retain the mode of higher nibble
				break;
			case(2):
				DDRB&=0xF0;
				break;
			case(3):
				DDRC&=0xF0;
				break;
			case(4):
				DDRD&=0xF0;
				break;
		}
	}

	else if((bIt==higherNibble)&&(mOde==OUTPUT))		//to set whole higher nibble as output
	{
		switch(rEgister)		//to switch between i/o registers
		{
			case(1):
				DDRA|=0xF0;			//here it is OR'ed to retain the mode of INPUTer nibble
				break;
			case(2):
				DDRB|=0xF0;
				break;
			case(3):
				DDRC|=0xF0;
				break;
			case(4):
				DDRD|=0xF0;
				break;
		}
	}
	else if((bIt==higherNibble)&&(mOde==INPUT))		//to set whole higher nibble as INPUT
	{
		switch(rEgister)		//to switch between i/o registers
		{
			case(1):
				DDRA&=0x0F;			//here it is ANDed to retain the mode of INPUTer nibble
				break;
			case(2):
				DDRB&=0x0F;
				break;
			case(3):
				DDRC&=0x0F;
				break;
			case(4):
				DDRD&=0x0F;
				break;
		}
	}

	else if((bIt==ALL)&&(mOde==OUTPUT))		//to set whole register as output
	{
		switch(rEgister)		//to switch between i/o registers
		{
				case(1):
				DDRA=0xFF;
				break;
			case(2):
				DDRB=0xFF;
				break;
			case(3):
				DDRC=0xFF;
				break;
			case(4):
				DDRD=0xFF;
				break;

		}

	}

	else if((bIt==ALL)&&(mOde==INPUT))		//to set whole register as INPUT
	{
		switch(rEgister)		//to switch between i/o registers
		{
				case(1):
				DDRA=0x00;
				break;
			case(2):
				DDRB=0x00;
				break;
			case(3):
				DDRC|=0x00;
				break;
			case(4):
				DDRD=0x00;
				break;

		}

	}
	else if((bIt<=7)&&(mOde==OUTPUT))
	{
		switch(rEgister)		//to switch between i/o registers
		{
			//eg: if OUTPUT is passed through function, mOde=1, bIt is corresponding value passed in function
			case(1):
				DDRA|=(1<<bIt); //do bitwise or and bit wise and
				break;
			case(2):
				DDRB|=(1<<bIt);
				break;
			case(3):
				DDRC|=(1<<bIt);
				break;
			case(4):
				DDRD|=(1<<bIt);
				break;

		}
	}

	else if((bIt<=7)&&(mOde==INPUT))
	{
		switch(rEgister)		//to switch between i/o registers
		{
			//eg: if OUTPUT is passed through function, mOde=1, bIt is corresponding value passed in function
			case(1):
				DDRA&=~(1<<bIt); //do bitwise or and bit wise and
				break;
			case(2):
				DDRB&=~(1<<bIt);
				break;
			case(3):
				DDRC&=~(1<<bIt);
				break;
			case(4):
				DDRD&=~(1<<bIt);
				break;

		}
	}


	/*switch(rEgister)		//to switch between i/o registers
		{
			//eg: if OUTPUT is passed through function, mOde=1, bIt is corresponding value passed in function
			case(1):
				DDRA=bIt; //do bitwise or and bit wise and
				break;
			case(2):
				DDRB=bIt;
				break;
			case(3):
				DDRC=bIt;
				break;
			case(4):
				DDRD=bIt;
				break;

		}*/

	return;
}

void digitalWrite(uint8_t rEgister,uint8_t bIt,uint8_t mOde)	// eg: setDigital(A,5,HIGH);
{
	if((bIt==lowerNibble)&&(mOde==HIGH))		//to set whole lower nibble as high
	{
		switch(rEgister)		//to switch between i/o registers
		{
			case(1):
				PORTA|=0x0F;			//here it is OR'ed to retain the mode of higher nibble
				break;
			case(2):
				PORTB|=0x0F;
				break;
			case(3):
				PORTC|=0x0F;
				break;
			case(4):
				PORTD|=0x0F;
				break;
		}
	}

	else if((bIt==lowerNibble)&&(mOde==LOW))		//to set whole lower nibble as low
	{
		switch(rEgister)		//to switch between i/o registers
		{
			case(1):
				PORTA&=0xF0;			//here it is ANDed to retain the mode of higher nibble
				break;
			case(2):
				PORTB&=0xF0;
				break;
			case(3):
				PORTC&=0xF0;
				break;
			case(4):
				PORTD&=0xF0;
				break;
		}
	}

	else if((bIt==higherNibble)&&(mOde==HIGH))		//to set whole higher nibble as HIGH
	{
		switch(rEgister)		//to switch between i/o registers
		{
			case(1):
				PORTA|=0xF0;			//here it is OR'ed to retain the mode of lower nibble
				break;
			case(2):
				PORTB|=0xF0;
				break;
			case(3):
				PORTC|=0xF0;
				break;
			case(4):
				PORTD|=0xF0;
				break;
		}
	}
	else if((bIt==higherNibble)&&(mOde==LOW))		//to set whole higher nibble as LOW
	{
		switch(rEgister)		//to switch between i/o registers
		{
			case(1):
				PORTA&=0x0F;			//here it is ANDed to retain the mode of lower nibble
				break;
			case(2):
				PORTB&=0x0F;
				break;
			case(3):
				PORTC&=0x0F;
				break;
			case(4):
				PORTD&=0x0F;
				break;
		}
	}

	else if((bIt==ALL)&&(mOde==HIGH))		//to set whole register as HIGH
	{
		switch(rEgister)		//to switch between i/o registers
		{
				case(1):
				PORTA=0xFF;
				break;
			case(2):
				PORTB=0xFF;
				break;
			case(3):
				PORTC=0xFF;
				break;
			case(4):
				PORTD=0xFF;
				break;

		}

	}

	else if((bIt==ALL)&&(mOde==LOW))		//to set whole register as LOW
	{
		switch(rEgister)		//to switch between i/o registers
		{
				case(1):
				PORTA&=0x00;
				break;
			case(2):
				PORTB&=0x00;
				break;
			case(3):
				PORTC&=0x00;
				break;
			case(4):
				PORTD&=0x00;
				break;

		}

	}

	else if((bIt<=7)&&(mOde==HIGH))
	{
		switch(rEgister)		//to switch between i/o registers
		{
			//eg: if HIGH is passed through function, mOde=1, bIt is corresponding value passed in function
			case(1):
				PORTA|=(1<<bIt);
				break;
			case(2):
				PORTB|=(1<<bIt);
				break;
			case(3):
				PORTC|=(1<<bIt);
				break;
			case(4):
				PORTD|=(1<<bIt);
				break;

		}
	}

	else if((bIt<=7)&&(mOde==LOW))
	{
		switch(rEgister)		//to switch between i/o registers
		{
			//eg: if HIGH is passed through function, mOde=1, bIt is corresponding value passed in function
			case(1):
				PORTA&=~(1<<bIt);
				break;
			case(2):
				PORTB&=~(1<<bIt);
				break;
			case(3):
				PORTC&=~(1<<bIt);
				break;
			case(4):
				PORTD&=~(1<<bIt);
				break;

		}
	}
	return;
}

uint8_t digitalRead(uint8_t rEgister,uint8_t bIt)	//eg: int x=getDigital(A,3); ***Note: this function returns a int value 1 or 0.***
{
	int vAlue;	//stores value either 1 or 0 and returns it
	switch(rEgister)
	{
		case(1):
			vAlue=PINA & (1<<bIt);
			break;
		case(2):
			vAlue=PINB & (1<<bIt);
			break;
		case(3):
			vAlue=PINC & (1<<bIt);
			break;
		case(4):
			vAlue=PIND & (1<<bIt);
			break;
	}

	/*((vAlue==(1<<bIt))?return 1:return 0);*/

	return (vAlue==(1<<bIt));

}

void delay(unsigned long millisec)
{
	int i;
	for(i=0;i<millisec;i++)
	{
		_delay_ms(1);
	}
	return;
}
void delayMicroseconds(unsigned long microsec)
{
	int i;
	for(i=0;i<microsec;i++)
	{
		_delay_us(1);
	}
	return;
}



void initADC()
{
	ADMUX=(1<<REFS0);				//Aref=AVcc
	ADCSRA=(1<<ADEN)|(1<<ADPS2)|(1<<ADPS1);		//ADC enabled, Prescaler 64
}

uint8_t analogRead(uint8_t cHannel)
{
	initADC();
	ADMUX|=cHannel;
	ADCSRA|=(1<<ADSC);

	while(ADCSRA & (1<<ADSC))

	return(ADC);
}

int AnalogRead(int x)
{


//prescalar set to default
  ADMUX=(1<<REFS0)|(0<<REFS1);
  ADCSRA|=(1<<ADEN);
  ADMUX|=x;//chose value from 0 to 7 to chose adc pin accordingly
  ADCSRA|=(1<<ADEN);
  ADCSRA|=(1<<ADSC);
 while(ADCSRA&(1<<ADSC));
 return (ADC);
}

class Serial{
	public:
	void begin(unsigned int bAud)
	{
		int uBrr=((F_CPU)/(bAud*16)-1);
		//Set baud rate
		UBRRH=(unsigned char)(uBrr>>8);
		UBRRL=(unsigned char)uBrr;
		UCSRB = (1<<RXEN)|(1<<TXEN);            //enable receiver and transmitter
		UCSRC = (1<<URSEL)|(1<<USBS)|(3<<UCSZ0);
	}

	void write( unsigned char dAta ){		//working fine
	/* Wait for empty transmit buffer */
	/* Wait for empty transmit buffer */
	while ( !( UCSRA & (1<<UDRE)) )
	;
	/* Put data into buffer, sends the data */
	UDR = 0b00000011;
	}

	/*uint8_t available(void){	//working fine
		 if((UCSRA & (1<<RXC)))
			return 1;

		else
			return 0;
	}*/

	unsigned char read( void ){		//PROBLEM: rx frame error in proteus
	/* Wait for data to be received */
		while(!(UCSRA) & (1<<RXC));           // wait while data is being received
    return UDR;                             // return 8-bit data
	}

	void flush(void){
		unsigned char dUmmy;
		while ( UCSRA & (1<<RXC) ) dUmmy = UDR;
	}

	void end(void){
		flush();
		UCSRB&=0xe7;	//disabling RXEN & TXEN
	}

};

void attachInterrupt(int iNtpin, void (*isrfunc)(void), int cOmpare)		//cOmpare:LOW=0,HIGH1,RISING=2,FALLING=3
{
	sei();
	cAllisr=iSrfunc;
	
        switch(iNtpin)	  //enabling interrupt pin
	{
		case 0:
		GICR= 1<<INT0;
		
		switch(cOmpare)
		{
			case 2: 
		        MCUCR|=(1<<ISC00)|(1<<ISC01);
			break;

			case 3: 
			MCUCR|=(0<<ISC00)|(1<<ISC01);
			break;

			case 4:
			MCUCR|=(1<<ISC00)|(0<<ISC01);
			break;

			default:
			MCUCR|=(0<<ISC00)|(0<<ISC01);
			
		}
		break;

		case 1:
		GICR|=1<<INT1;
		switch(cOmpare)
		{
			case 2:
			MCUCR|=(1<<ISC10)|(1<<ISC11);
			break;

			case 3:
			MCUCR|=(0<<ISC10)|(1<<ISC11);
			break;

			case 4:
			MCUCR|=(1<<ISC10)|(0<<ISC11);
			break;

			default:
			MCUCR|=(0<<ISC00)|(0<<ISC01);
			
		}
		break;

		case 2:
		GICR|=1<<INT2;
		switch(cOmpare)
		{
			case 2:
			MCUCSR|=(1<<ISC2);
			break;

			case 3:
			MCUCSR|=(0<<ISC2);
			break;

			default:
			MCUCSR|=(0<<ISC2);
			
		}
		break;

		default:
		MCUCR|=(0<<ISC00)|(0<<ISC01); 
       }	 
}

ISR(INT0_vect)
{
	cAllisr();
}
ISR(INT1_vect)
{
	cAllisr();
}
ISR(INT2_vect)
{
	
	cAllisr();
}
class EEPROM{
	void write(unsigned int aDdress, unsigned char dAta)
	{
		/*wait until previous process is completed*/
		while(EECR & (1<<EEWE))

		EEAR=aDdress;	//address of eeprom
		EEDR=dAta;		//data of eeprom

		EECR |= (1<<EEMWE);

		EECR |= (1<<EEWE);	//start eeprom
	}

	char read(unsigned int aDdress)
	{
		/* Wait for completion of previous write */
		while(EECR & (1<<EEWE))		;
		/* Set up address register */
		EEAR = aDdress;
		/* Start eeprom read by writing EERE */
		EECR |= (1<<EERE);
		/* Return data from data register */
		return EEDR;
	}

};

double map(double vAlue, double fromLow, double fromHigh, double toLow, double toHigh)
{
	return ((vAlue-fromLow)/abs(fromHigh-fromLow)*abs(toHigh+toLow));
}

double constrain(double nUm,double x,double y)
{
	if(nUm<x){return x;}
	if(nUm>y){return y;}
	else return nUm;	
}

EEPROM EEPROM;
Serial Serial;
int main() {
	setup();
	while (1) {
		loop();
	}
}

int analogWrite(int pIn,int dUtycycle)
{
	//initialize TCCR0 as per requirement, say as follows
	TCCR1A |= (1<<WGM10)|(1<<COM1A1)|(1<<COM1B1);//initializing timer1
	TCCR1B |=(1<<CS10);
	TCNT1=0;
	if(pin==1)
  	{
    		OCR1A=dUtycycle;
  	}
  	else if(pin==2)
  	{
  	 	OCR1B=dUtycycle;
  	}
}
