/*
 * TWISLAVE.c
 *
 * Created: 10-09-2018 16:04:50
 *  Author: HP
 */ 

#include <avr/io.h>
#ifndef TWI_FREQ
#define TWI_FREQ 100000L
#define F_CPU 16000000
#endif
void I2C_Init()			/* I2C initialize function */
{
	TWBR=((F_CPU / TWI_FREQ) - 16) / 2;	/* Get bit rate register value by formula */
}
uint8_t I2C_Start(void)/* I2C start function */
{
    uint8_t status;
	int x;
	DDRB=0b11111111;		/* Declare variable */
    TWCR=(1<<TWSTA)|(1<<TWEN)|(1<<TWINT); /* Enable TWI, generate START */
    while(!(TWCR&(1<<TWINT)))
	{PORTB|=4;
}	/* Wait until TWI finish its current job */
    status=TWSR&0xF8;		/* Read TWI status register */
    if(status!=0x08)		/* Check weather START transmitted or not? */
    return 0;			/* Return 0 to indicate start condition fail */
    TWDR=0b01111110;		/* Write SLA+W in TWI data register */
    TWCR=(1<<TWEN)|(1<<TWINT);
		/* Enable TWI & clear interrupt flag */
    while(!(TWCR&(1<<TWINT)));
	/* Wait until TWI finish its current job */
    status=TWSR&0xF8;		/* Read TWI status register */	
    if(status==0x18)		/* Check for SLA+W transmitted &ack received */
    PORTB|=1;			
	    if(status==0x20)		
	   PORTB=2;	
   			
}
void I2C_Stop()			
{
    TWCR=(1<<TWSTO)|(1<<TWINT)|(1<<TWEN);
    while(TWCR&(1<<TWSTO));	
}
void I2C_Send(char x)
{ uint8_t stat;
	TWDR=x;
	TWCR=(1<<TWINT)|(1<<TWEN);
	
	while(!(TWCR&(1<<TWINT)));
	stat=TWSR&0xF8;
	if(stat==0x28)
	PORTB|=8;
	if(stat==0x30)
	PORTB|=16;
}
int main()
{I2C_Init();
	
		I2C_Start();
		I2C_Send();
		I2C_Stop();
		
	
	
}	


