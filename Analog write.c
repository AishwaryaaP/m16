/*
 * GccApplication15.c
 *
 * Created: 30-07-2018 19:10:04
 * Author : acer
 */ 

#include <avr/io.h>
#include <util/delay.h>
int duty;
int pwm_init(x)
{
	int duty;
	int x;
	//initialize TCCR0 as per requirement, say as follows
	TCCR0 |= (1<<WGM00)|(1<<COM01)|(1<<CS00)|(1<<FOC0);
	DDRB =0b11111111;
	
	
	//TCCR0 |=(1<<CS11);
	
	// make sure to make OC0 pin (pin PB3 for atmega32) as output pin
	//TCNT0=0;
	duty=x;
	
	
	
}













int main(void)
{
	//DDRD=0b11111111;
	int x;
	
	
	
	/* Replace with your application code */
	while (1)
	{
		pwm_init();
		OCR0=duty;
		_delay_ms(5);
		
		
		
	}
}


