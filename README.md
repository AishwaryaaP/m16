This repo contains libraries for AVR ATmega 16 & similar micro-controllers.
It contains function names same as that used in Arduino IDE

io.h is the main library containing all the basic functions for ATmega16 microcontroller

analogWrite1() for Atmega 2560:

1.PARAMETERS:
The parameters for this function are the pin and duty cycle.The pins for this function are PB5 and PB6 on Atmega and digital pin 11 and 12 on Arduino.For simplicity,pin parameter has only two values.These are 1 and 2.If 1 is given as parameter for pin,the duty cycle will be given to PB5/digital pin 11 and if it is 2 the duty cycle will be given toPB6/digital pin 12.

2.SYNTAX:

analogWrite1(pin,duty cycle);

analogWrite() for Atmega 16:

1.PARAMETERS:

The parameters for this function are the pin and duty cycle.The pins for this function are PD4 and PD5 on Atmega .For simplicity,pin parameter has only two values.These are 1 and 2.If 1 is given as parameter for pin,the duty cycle will be given to PD4 and if it is 2 the duty cycle will be given to PD5.

2.SYNTAX:

analogWrite(pin,duty cycle);

PulseIn LIBRARY:

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

attachinterrupt LIBRARY:

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
      
Ex:
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
