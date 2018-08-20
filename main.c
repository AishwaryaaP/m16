/*
 * GccApplication26.c
 *
 * Created: 11-08-2018 11:40:49
 * Author : acer
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include <compat/twi.h>

#define F_CPU 1000000UL							/* Define CPU clock Frequency e.g. here its 8MHz */

//I2C slave code
void slave_init(int add)//initializing slave
{
	TWAR=add;
	TWCR=(1<<TWEA)|(1<<TWEN)|(1<<TWINT);
	
}
int8_t Slave_listen()//function for slave to listen to master
{
	 
	    while(1)
		{
		 uint8_t status;
		// TWCR=(1<<TWINT)|(1<<TWEN);			
		 while(!(TWCR&(1<<TWINT)));	
		 status=TWSR&0xF8;		
		 if(status==0x60)	
		 {
			 // while(!(TWCR&(1<<TWINT)));
			 PORTD=0b11111111;
			 _delay_ms(50);
			// PORTD=0b00000000;
		 }
		 
		 
	
		 	
		 else
		 {
		   break;
		 }
		
	}		/* Else continue */
	 
 }
	

int slave_receive()//function for slave to receive data from master
{
	int status;
	TWCR=(1<<TWEN)|(1<<TWINT)|(1<<TWEA);
	while(!(TWCR&(1<<TWINT)));
	status=TWSR&0xF8;
	if(status==0x80)
	{
		//while(!(TWCR&(1<<TWINT)));
		PORTD=0b11111111;
		_delay_ms(15);
		return TWDR;
		
	}
	TWCR=(1<<TWEN)|(1<<TWINT);
	while(!(TWCR&(1<<TWINT)));
	status=TWSR&0xF8;
	if(status==0x88 || status==0x98)
	{
		//PORTB=0b11111111;
		//_delay_ms(15);
		return TWDR;
	}
	TWCR=(1<<TWEN)|(1<<TWINT);
	while(!(TWCR&(1<<TWINT)));
	status=TWSR&0xF8;
    if(status==0xA0)
	{
	   //	PORTB=0b11111111;
	}
	
	
	 
	//else if(status==0xA0)
	
		//PORTA=0b00000000;
		//TWCR=(1<<TWINT);
	

	
		//PORTB=0b00000000;
	
	//return TWDR;

	
}


int main(void)
{
	DDRD=0b11111111;
	DDRA=0b11111111;
	DDRB=0b11111111;
	long int x;
	slave_init(0x20);
	/*slave_receive();
	Slave_listen();
	x=slave_receive();
	if(x==0x20)
	{
		PORTB=0b11111111;
	}
	_delay_ms(5);
	
	
    /* Replace with your application code */
    while (1) 
    {
		Slave_listen();
		x=slave_receive();
		if(x==0x20)
		{
			PORTB=0b11111111;
			_delay_ms(50);
		}
		
		
    }
	
}                     

