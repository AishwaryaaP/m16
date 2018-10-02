
#include <avr/io.h>	
#define F_CPU 16000000
#define TWI_FREQ 100000L
void I2C_Init()			/* I2C initialize function */
{
	TWBR=((F_CPU / TWI_FREQ) - 16) / 2;	/* Get bit rate register value by formula */
}
void I2C_Slave_Init(uint8_t slave_address)
{
    TWAR=slave_address;		/* Assign Address in TWI address register */
    TWCR=(1<<TWEN)|(1<<TWEA)|(1<<TWINT);/* Enable TWI, Enable ack generation */
}
void listen()
{	uint8_t status;
	DDRD=0b11111111;
		
while(!(TWCR&(1<<TWINT)));
status=TWSR&0xF8;
	if(status==0x60||status==0x68)	/* Own SLA+W received &ack returned */
    PORTD|=1;
	if(status==0xA8||status==0xB0) /* Own SLA+R received &ack returned */
PORTD|=2; /* Return 0 to indicate ack returned */
if(status==0x70||status==0x78) /* General call received &ack returned */
PORTD|=3;

}
uint8_t write(char x)
{	   uint8_t sttus;
TWDR=x;
	
 TWCR=(1<<TWEN)|(1<<TWINT)|(1<<TWEA);
	while(!(TWCR&(1<<TWINT)));

	return TWDR;
	sttus=TWSR&0xF8;
	
}
void main()
{ DDRB=0b11111111;

	I2C_Init();
	int c;
uint8_t x;
I2C_Slave_Init();
listen();	
read();
	
		
	

	
	
} 
	
