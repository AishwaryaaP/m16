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
#include <avr/io.h>

#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <stdlib.h>
#include<math.h>

void delay(unsigned long);		//simpler form of delay function in avr
void analogWrite(uint8_t, uint8_t);		//PWM function using TIMER 0   ***NOTE:Timer 0 cannot be used if this function is used in the code***
uint16_t analogRead(uint8_t);
void attachInterrupt(int,void *,int);
void softwareInterrupt(void *);
double map(double,double,double,double,double);
double constrain(double,double,double);
float millis();
double pulseIn(volatile uint8_t , uint8_t );
double microsecondsToInches(unsigned long );
double microsecondsToCentimeters(unsigned long );
void setup();
void loop();
const uint8_t OUTPUT=1,INPUT=0;
const uint8_t HIGH=1,LOW=0;
const uint8_t RISING=2,FALLING=3,CHANGE=4;
const uint8_t A=1,B=2,C=3,D=4,lowerNibble=8,higherNibble=9,ALL=10,D4=4,D5=5;
//variables for user interface
/***VARIABLES lowerNibble, higherNibble, ALL ARE FOR SETTING A SET OF BIT AT ONCE***/
void (*cAllisr)(void);		//function pointer used in ISR()
void (*uSerfun) (void);        //function pointer used inISR() of softwareInterrupt

double microsecondsToInches(unsigned long mIcroseconds) {
  // According to Parallax's datasheet for the PING))), there are 73.746
  // microseconds per inch (i.e. sound travels at 1130 feet per second).
  // This gives the distance travelled by the ping, outbound and return,
  // so we divide by 2 to get the distance of the obstacle.
  // See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
  return (mIcroseconds*0.00669/ 2);
}

double microsecondsToCentimeters(unsigned long mIcroseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the object we
  // take half of the distance travelled.
  return (mIcroseconds*0.17/ 2);
}

double pulseIn(volatile uint8_t pInno, uint8_t vAlue)
{
  TCCR2 = (1 << WGM21) | (1 << COM21) | (1 << FOC2) | (0 << COM20) | (0 << WGM20); //initializing in CTC mode
  TCCR2 = (1 << CS20);
  unsigned long mAxloops = 500000;
  unsigned long wIdth = 0;
  // wait for any previous pulse to end
  while ( ((PIND)&&(pInno)) == vAlue)//remove PIND. It should be for every register.
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
float millis()//float and not int.
{       int x;
	float mIlli;
	mIlli=x*0.16+0.00000625*TCNT0;
        return mIlli;
}
class serial
{
public:
void start( unsigned int uBrr)
{
	/*Set baud rate */
	UBRRH = (unsigned char)(uBrr>>8);
	UBRRL = (unsigned char)uBrr;
	/*Enable receiver and transmitter */
        UCSRB = (1<<RXEN)|(1<<TXEN);
}
/* Set frame format: 8data, 2stop bit */
void send( unsigned char data )
{
	/* Wait for empty transmit buffer */
	while ( !( UCSRA & (1<<UDRE)) )
	;
	/* Put data into buffer, sends the data */
	UDR = data;
	_delay_ms(100);
}
unsigned char get( void )
{
	/* Wait for data to be received */
	while ( !(UCSRA & (1<<RXC)) )
	;
	/* Get and return received data from buffer */
	return UDR;
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


void delay(unsigned long mIllisec)
{
	int i;
	for(i=0;i<mIllisec;i++)
	{
		_delay_ms(1);
	}
	return;
}

void delayMicroseconds(unsigned long mIcrosec)
{
	int i;
	for(i=0;i<mIcrosec;i++)
	{
		_delay_us(1);
	}
	return;
}



	//ADC enabled, Prescaler 64


uint16_t analogRead(uint8_t cHannel)
{ADMUX=(1<<REFS0);				//Aref=AVcc
ADCSRA=(1<<ADEN)|(1<<ADPS2)|(1<<ADPS1);		
  
  ADMUX=(1<<REFS0)|(0<<REFS1);
  ADCSRA|=(1<<ADEN);
  ADMUX|=cHannel;//chose value from 0 to 7 to chose adc pin accordingly
  ADCSRA|=(1<<ADEN);
  ADCSRA|=(1<<ADSC);
 while(ADCSRA&(1<<ADSC));
 return (ADC);
}


	
void attachInterrupt(int iNtpin, void (*iSrfunc)(void), int cOmpare)		//cOmpare:LOW=0,HIGH1,RISING=2,FALLING=3
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

void softwareInterrupt(void *isrfun(void))
{
	sei();
	uSerfun = isrfun;
        TCCR0=(1<<WGM01)|(1<<WGM00)|(1<<CS00); //fast pwm and prescalar is 1 
	TIMSK=1<<TOIE0;//overflow interrupt flag is set
}

ISR(TIMER0_OVF_vect)
{
		uSerfun();
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

double constrain(double nUm,double uPper,double lOwer)
{
	if(nUm<uPper){
		return uPper;}
	else if(nUm>lOwer){
		return lOwer;}
	else 
	return nUm;	
}


void analogWrite(int pIn,int dUtycycle)
{
	
	
	//initialize TCCR0 as per requirement, say as follows
	TCCR1A |= (1<<WGM10)|(1<<COM1A1)|(1<<COM1B1);//initializing timer1
	TCCR1B |=(1<<CS10);
	
	TCNT1=0;
	
	
	if(pIn==1)
  	{
    		OCR1A=dUtycycle;
  	}
  	else if(pIn==2)
  	{
  	 	OCR1B=dUtycycle;
  	}
}

