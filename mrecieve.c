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
    TWDR=0b01111111;		/* Write SLA+W in TWI data register */
    TWCR=(1<<TWEN)|(1<<TWINT);
		/* Enable TWI & clear interrupt flag */
    while(!(TWCR&(1<<TWINT)));
	/* Wait until TWI finish its current job */
    status=TWSR&0xF8;		/* Read TWI status register */	
    if(status==0x18)		/* Check for SLA+W transmitted &ack received */
    PORTB|=1;			/* Return 1 to indicate ack received */
    if(status==0x20)		/* Check for SLA+W transmitted &nack received */
    PORTB=2;	/* Return 2 to indicate nack received */
   			/* Else return 3 to indicate SLA+W failed */
}
void I2C_Stop()			/* I2C stop function */
{
    TWCR=(1<<TWSTO)|(1<<TWINT)|(1<<TWEN);/* Enable TWI, generate stop */
    while(TWCR&(1<<TWSTO));	/* Wait until stop condition execution */
}
uint8_t I2C_mrecieve(void)
{ uint8_t stat;
	TWCR=(1<<TWINT)|(1<<TWEN);
	
	while(!(TWCR&(1<<TWINT)));
	stat=TWSR&0xF8;
	if(stat==0x28)
	PORTB|=8;
	if(stat==0x30)
	PORTB|=16;
	return TWDR;
}
int main()
{I2C_Init();
	DDRD=0b11111111;
	uint8_t m;
	
		I2C_Start();
		m=I2C_Send();
		I2C_Stop();
		PORTD=m;
	
	
}	


