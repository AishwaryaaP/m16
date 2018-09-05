This report contains libraries for AVR ATmega 16 & similar micro-controllers.
It contains function names same as that used in Arduino IDE

io.h is the main library containing all the basic functions for ATmega16 microcontroller

Index:
1]analogWrite()
2]pulseIn()
3]attachinterrupt()
4]LCD
5]
6]
7]
8]
9]
10]

1]analogWrite():

1.PARAMETERS:

The parameters for this function are the pin and duty cycle.The pins for this function are PD4 and PD5 on Atmega .For simplicity,pin parameter has only two values.These are 1 and 2.If 1 is given as parameter for pin,the duty cycle will be given to PD4 and if it is 2 the duty cycle will be given to PD5.

2.SYNTAX:

analogWrite(pin,duty cycle);

2]pulseIn():

FUNCTIONS:

1.  unsigned long pulseIn();
  	This function reads a pulse (either HIGH or LOW) on a pin. For example, if value is HIGH, pulse_In() waits for the pin to go             from LOW to HIGH, starts timing, then waits for the pin to go LOW and stops timing. Returns the length of the pulse in            	   microseconds or gives up and returns 0 if no complete pulse was received within the timeout.
    SYNTAX:
      	unsigned long pulseIn (volatile uint8_t bitno, uint8_t stateMask);
    PARAMETERS:
    •	pInno: The pin number on which you want to read the pulse. (int)
    •	vAlue: type of pulse to read: either 1(for high) or 0(for low). (int)

2.  unsigned long microsecondsToInches();
    •	There are 73.746 microseconds per inch (i.e. sound travels at 1130 feet per second).This gives the distance travelled by the     	 ping, outbound and return, so we divide by 2 to get the distance of the obstacle.
    FORMULA:    
        mIcroseconds*0.00669/ 2
    SYNTAX:    
        unsigned long microsecondsToInches(unsigned long mIcroseconds1)

3.  unsigned long microsecondsToCentimeters();
    •	The speed of sound is 340 m/s or 29 microseconds per centimetre. The ping travels out and back, so to find the distance of the           object we take half of the distance travelled.
    FORMULA:
        mIcroseconds*0.17/ 2
    SYNTAX:
        unsigned long microsecondsToCentimeters(unsigned long mIcroseconds)

EXAMPLE CODE:
#include <avr/io.h>
#include <util/delay.h>
#include <pulseIn.h>
const Pin = PIND0
const PingPin = PINC0
int main()
{
  DDRC=0b11111111;
  DDRD=0b00000000;
while(1) 
{
  // establish variables for duration of the ping, and the distance result
  // in inches and centimeters:
  unsigned long duration, inches, cm;
  // The PING is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  // trig 10
  PORTC=(1<<pingPin);
  _delay_us(2);
  PORTC=(1<<pingPin);
  _delay_us(5);
  PORTC=(1<<pingPin);

  // Pin is used to read the signal from the PING : a HIGH pulse
  // whose duration is the time (in microseconds) from the sending of the ping
  // to the reception of its echo off of an object.
  //echo11
  duration = pulseIn(Pin,1);
  
  // convert the time into a distance
  
  inches = microsecondsToInches(duration);
  cm = microsecondsToCentimeters(duration);
  PORTB=0;
  if(cm<10)
  {
	  PORTB=0b00000001;	  
  }
  if(cm>=10||cm<20)
  {
	  PORTB=0b00000010;	  
  }
  if(cm>=20||cm<30)
  {
	  PORTB=0b00000100;	  
  }
  else if(cm>=30)
  {
	  PORTB=0b00001000;	  
  }
}
}

3]attachinterrupt():

FUNCTIONS:

1.void attachInterrupt (): External interrupt enabling function
SYNTAX:
void attachInterrupt (digitalpintoInterrupt, *ISR (), mode)
PARAMETERS:
• digitalpintoInterrupt : The external interrupt request enable pin available are INT0, INT1, INT2. It takes an integer value.
     • 0 for INT0
     • 1 for INT1
     • 2 for INT2
     
• *ISR ():  This function is called if the external interrupt occurs.
      •	No parameter
      •	Returns nothing

• mode: The mode at which the interrupt is enabled. Predefined constants are used for the different modes:   
      •	RISING=2: interrupt enables on rising edge 
      •	FALLING=3: interrupt enables on falling edge 
      •	CHANGE=4: interrupt enables on any change of the edge
      
EXAMPLE CODE:
#include<avr/io.h>
#include<attachInterrupt.h>
#include<avr/interrupt.h>
void fun(void)
{
        while (PIND&&(1<<PD2)){    // while PD2(INT0) is HIGH
        PORTB|=(1<<PB7);}
        PORTB|=(1<<PB6);   // glowing a led  
}
Int main() 
{       DDRB=0XFF; // setting leds as output
	DDRD= 0X00; 
	PORTB|=0X00;
	PORTD|=0XFF;// pull-up the interrupts
	attachInterrupt (0, *fun, 2); // INT0  is enabled on a RISING edge and fun() is called
}
while(1) {             
}

softwareinterrupt LIBRARY:

FUNCTIONS:
1.void softwareInterrupt(): It enables internal interrupt.

SYNTAX:
void softwareInterrupt (*Isr ())

PARAMETERS:
*  isr(): ISR is called if internal interrupt is enabled.
	No parameter 
	Returns nothing
Ex: #define F_CPU 1000000UL
void func(void)
{
   PORTD=0b11111111; 
   _delay_ms(500);   
   PORTD=0b00000000;
   _delay_ms(500);   // blinking a led
}

int main()
{       DDRD=0X00;
	PORTD=0XFF;
	sei();
        softwareInterrupt(*func);
	while (1)
	{
        }
}

4]LCD:

FUNCTIONS:
	1.	Lcd8_Init () & Lcd4_Init (): These functions will initialize the 16×2 LCD module connected to the microcontroller
	        pins defined by the following constants.
	2.	Lcd8_Clear() & Lcd4_Clear() : Calling these functions will clear the 16×2 LCD display screen when interfaced with
	        8 bit and 4 bit mode respectively.
	3.	Lcd8_SetCursor() & Lcd4_SetCursor() : These function will set the cursor position on the LCD screen by specifying 
	        its row and column. By using these functions we can change the position of character and string displayed by the
		following functions.
	4.	Lcd8_WriteChar() & Lcd4_WriteChar() : These functions will write a single character to the LCD screen and the cursor
	        position will be incremented by one.
	5.	Lcd8_WriteString() & Lcd8_WriteString() : These function will write string or text to the LCD screen and the cursor
	        positon will be incremented by length of the string plus one.
	6.	Lcd8_ShiftLeft() & Lcd4_ShiftLeft() : This function will shift data in the LCD display without changing data in the
	        display RAM.
	7.	Lcd8_ShiftRight() & Lcd8_ShiftRight() : This function will shift data in the LCD display without changing data in 
	        the display RAM.
	8.	For Pin change:
		•	Change Lcd4_Port(data) and Lcd8_Port(data) to PORTD = data
		•	Change pinChange(EN,1) to  PORTC |= (1<<PC7)
		•	Change pinChange(EN,0) to PORTC &= ~(1<<PC7)
		•	Change pinChange(RS,1) to PORTB |= (1<<PB6)
		•	Change pinChange(RS,0) to PORTC &= ~(1<<PC6)

EXAMPLE CODE:
// Defining pins for LCD
#define D4 eS_PORTD4
#define D5 eS_PORTD5
#define D6 eS_PORTD6
#define D7 eS_PORTD7
#define RS eS_PORTC6
#define EN eS_PORTC7
//Including standard libs
#include <avr/io.h>
#define F_CPU 8000000UL
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include "lcd.h"
// Defining pins
#define trig PB0	//Output pin for sending trigger pulse
#define echo 2	//Input pin for receiving distance proportional pulse

int time=0;

void set_interrupt()
{
	GICR |= (1<<INT0);	//Enable external interrupts
	MCUCR |= (1<<ISC00); //Trigger interrupt on logical change
	MCUCR &= ~(1<<ISC01);
	sei();	//Enable global interrupts or SREG |= (1<<I)
}

void init_timer()
{
	TCCR1B = (1<<CS11); // Initialize timer1 with prescaler of 8
	TCNT1 = 0;
}

void trigger()
{
	PORTB &= ~(1<<trig);
	_delay_us(2);
	PORTB |= (1<<trig);
	_delay_us(12);
	PORTB &= ~(1<<trig);
	_delay_us(2);
}

ISR(INT0_vect)
{
	if(TCNT1 == 0)
	{
		init_timer();
	}
	else
	{
		time = TCNT1;
		TCNT1 = 0;
		TCCR1B = 0x00; 
	}
}

int main(void)
{
	DDRB |= (1<<trig);	//Setting trigger pin as output
	DDRC |= (0b11100000);	//Setting enable and RS pin as output
	DDRD &= ~(1<<echo);	//Setting echo pin as input
	DDRD |= (0b11110000);	//Setting LCD data pins as output
	set_interrupt();
	Lcd4_Init();
	int dist = 0;
	char a[10];
    while (1) 
    {
		trigger();
		dist = time/58;
		itoa(dist, a, 10);
		if(dist>400)
		{
			Lcd4_Set_Cursor(1,1);			
			Lcd4_Write_String("No object found");
		}
		else
		{
			Lcd4_Set_Cursor(1,1);
			Lcd4_Write_String("Distance : ");
			Lcd4_Write_String(a);
		}
		_delay_ms(1000);
		Lcd4_Clear();
    }
}
