/*
 * GccApplication16.c
 *
 * Created: 31-07-2018 13:00:22
 * Author : acer
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
int duty;

ISR(TIMER0_COMP_vect)
{
	//PORTB^= 1;
	//_delay_ms(100);
	
	
		PORTB^=0b11111111;
		//duty=0;
	
	
	
}

void gpiopwm()
{
	
	//DDRB = 0b11111111;
	TCCR0 |= (1<<COM01)|(1<< CS01)|(1<<FOC0) | (1<<WGM01);//initializing in ctc mode
	TCNT0 = 0;//initializing counter
	//duty=0;
	TIMSK|=(1<<OCIE0);//enabling output compare interrupt
	OCR0=127;//duty cycle
	
	//GICR = 1<<INT0;
	//MCUCR = (1<<ISC01) | (1<<ISC00);
	//duty=127;//
	
}
	
	



int main(void)
{
	DDRB = 0b11111111;
	//int a;
	
	sei();
	gpiopwm();
	
    /* Replace with your application code */
    while (1) 
    {
		
		
		//OCR0=duty;
		//sei();
    }
	
}

