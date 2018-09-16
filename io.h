/******************************************************
 |		#########   #########	#########	|
 |		    #       #       #   #		|
 | 		    #       #       #   #		|
 |		    #       #########	######		|
 |		    #       #  #        #		|
 |		    #       #     #     #		|
 |		    #       #       #   #		|
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
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <stdlib.h>
#include<math.h>

void pinMode(uint8_t , uint8_t );
static void turnOffPWM(uint8_t );
void digitalWrite(uint8_t , uint8_t );
int digitalRead(uint8_t );

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





void pinMode(uint8_t pIn, uint8_t mOde)
{
	uint8_t bit = digitalPinToBitMask(pIn);
	uint8_t port = digitalPinToPort(pIn);
	volatile uint8_t *reg, *out;

	if (port == NOT_A_PIN) return;

	// JWS: can I let the optimizer do this?
	reg = portModeRegister(port);
	out = portOutputRegister(port);

	if (mOde == INPUT) { 
		uint8_t oldSREG = SREG;
                cli();
		*reg &= ~bit;
		*out &= ~bit;
		SREG = oldSREG;
	} else if (mOde == INPUT_PULLUP) {
		uint8_t oldSREG = SREG;
                cli();
		*reg &= ~bit;
		*out |= bit;
		SREG = oldSREG;
	} else {
		uint8_t oldSREG = SREG;
                cli();
		*reg |= bit;
		SREG = oldSREG;
	}
}


static void turnOffPWM(uint8_t tImer)
{
	switch (tImer)
	{
		#if defined(TCCR1A) && defined(COM1A1)
		case TIMER1A:   cbi(TCCR1A, COM1A1);    break;
		#endif
		#if defined(TCCR1A) && defined(COM1B1)
		case TIMER1B:   cbi(TCCR1A, COM1B1);    break;
		#endif
		#if defined(TCCR1A) && defined(COM1C1)
		case TIMER1C:   cbi(TCCR1A, COM1C1);    break;
		#endif
		
		#if defined(TCCR2) && defined(COM21)
		case  TIMER2:   cbi(TCCR2, COM21);      break;
		#endif
		
		#if defined(TCCR0A) && defined(COM0A1)
		case  TIMER0A:  cbi(TCCR0A, COM0A1);    break;
		#endif
		
		#if defined(TCCR0A) && defined(COM0B1)
		case  TIMER0B:  cbi(TCCR0A, COM0B1);    break;
		#endif
		#if defined(TCCR2A) && defined(COM2A1)
		case  TIMER2A:  cbi(TCCR2A, COM2A1);    break;
		#endif
		#if defined(TCCR2A) && defined(COM2B1)
		case  TIMER2B:  cbi(TCCR2A, COM2B1);    break;
		#endif
		
		#if defined(TCCR3A) && defined(COM3A1)
		case  TIMER3A:  cbi(TCCR3A, COM3A1);    break;
		#endif
		#if defined(TCCR3A) && defined(COM3B1)
		case  TIMER3B:  cbi(TCCR3A, COM3B1);    break;
		#endif
		#if defined(TCCR3A) && defined(COM3C1)
		case  TIMER3C:  cbi(TCCR3A, COM3C1);    break;
		#endif

		#if defined(TCCR4A) && defined(COM4A1)
		case  TIMER4A:  cbi(TCCR4A, COM4A1);    break;
		#endif					
		#if defined(TCCR4A) && defined(COM4B1)
		case  TIMER4B:  cbi(TCCR4A, COM4B1);    break;
		#endif
		#if defined(TCCR4A) && defined(COM4C1)
		case  TIMER4C:  cbi(TCCR4A, COM4C1);    break;
		#endif			
		#if defined(TCCR4C) && defined(COM4D1)
		case TIMER4D:	cbi(TCCR4C, COM4D1);	break;
		#endif			
			
		#if defined(TCCR5A)
		case  TIMER5A:  cbi(TCCR5A, COM5A1);    break;
		case  TIMER5B:  cbi(TCCR5A, COM5B1);    break;
		case  TIMER5C:  cbi(TCCR5A, COM5C1);    break;
		#endif
	}
}

void digitalWrite(uint8_t pIn, uint8_t val)
{
	uint8_t tImer = digitalPinToTimer(pIn);
	uint8_t bit = digitalPinToBitMask(pIn);
	uint8_t port = digitalPinToPort(pIn);
	volatile uint8_t *out;

	if (port == NOT_A_PIN) return;

	// If the pin that support PWM output, we need to turn it off
	// before doing a digital write.
	if (tImer != NOT_ON_TIMER) turnOffPWM(tImer);

	out = portOutputRegister(port);

	uint8_t oldSREG = SREG;
	cli();

	if (val == LOW) {
		*out &= ~bit;
	} else {
		*out |= bit;
	}

	SREG = oldSREG;
}

int digitalRead(uint8_t pIn)
{
	uint8_t tImer = digitalPinToTimer(pIn);
	uint8_t bit = digitalPinToBitMask(pIn);
	uint8_t port = digitalPinToPort(pIn);

	if (port == NOT_A_PIN) return LOW;

	// If the pin that support PWM output, we need to turn it off
	// before getting a digital reading.
	if (timer != NOT_ON_TIMER) turnOffPWM(tImer);

	if (*portInputRegister(port) & bit) return HIGH;
	return LOW;
}

const uint8_t PROGMEM digital_pin_to_port_PGM[] ={
	
PB, //(XCK/T0)
PB,//(T1)
PB,//(INT2/AIN0)
PB, //(OC0/AIN1)
PB, //(SS) 
PB, //(MOSI) 
PB, //(MISO)
PB,//(SCK)
PD, //(RXD)
PD, //(TXD)
PD,//(INT0) 
PD,//(INT1)
PD,//(OC1B)
PD,//(OC1A)
PD,	//(ICP)
PD, //(OC2)
PC,//(SCL)
PC, //(SDA)
PC, //(TCK)
PC, //(TMS)
PC, //(TDO)
PC, //(TDI)
PC, //(TOSC1)
PC,// (TOSC2)

};

const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] = {
	
_BV(0), //(XCK/T0)
_BV(1),//(T1)
_BV(2),//(INT2/AIN0)
_BV(3), //(OC0/AIN1)
_BV(4), //(SS) 
_BV(5), //(MOSI) 
_BV(6), //(MISO)
_BV(7),//(SCK)
_BV(0), //(RXD)
_BV(1), //(TXD)
_BV(2),//(INT0) 
_BV(3),//(INT1)
_BV(4),//(OC1B)
_BV(5),//(OC1A)
_BV(6),	//(ICP)
_BV(7), //(OC2)
_BV(0), //(SCL)
_BV(1), //(SDA)
_BV(2), //(TCK)
_BV(3), //(TMS)
_BV(4), //(TDO)
_BV(5), //(TDI)
_BV(6), //(TOSC1)
_BV(7),// (TOSC2)


};

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
class Serial
{
	public:
	void start( unsigned int uBrr){
		/*Set baud rate */
		UBRRH = (unsigned char)(uBrr>>8);
		UBRRL = (unsigned char)uBrr;
		/*Enable receiver and transmitter */
		UCSRB = (1<<RXEN)|(1<<TXEN);
	}
	/* Set frame format: 8data, 2stop bit */
	void send( unsigned char data ){
		/* Wait for empty transmit buffer */
		while ( !( UCSRA & (1<<UDRE)) )
		;
		/* Put data into buffer, sends the data */
		UDR = data;
		_delay_ms(100);
	}
	unsigned char get( void ){
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
{	
	ADMUX=(1<<REFS0);				//Aref=AVcc
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

void softwareInterrupt(void (*isrfun)(void))
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

