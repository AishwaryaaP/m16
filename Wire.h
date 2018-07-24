/**
 *I2C Communication Library
 *
 */
#include <compat/twi.h>
#include <stdlib.h>
#include <avr/sfr_defs.h>
#include <avr/pgmspace.h>
#ifndef TWI_FREQ
#define TWI_FREQ 100000L
#endif

void (*cAllfunc)(void);

class TwoWire{
public:
//MASTER PART
  void begin(){   //Begin function for Master device
    cbi(TWSR, TWPS0);
    cbi(TWSR, TWPS1);
    TWBR = TWBR = ((F_CPU / TWI_FREQ) - 16) / 2;	// Get bit rate register value by formula
    TWCR = (1<<TWEN) | (1<<TWIE) | (1<<TWEA);
  }

  uint8_t beginTransmission(uint8_t aDdress){
    uint8_t rEgisterstatus;											/* Declare variable */
    while (1)
    {
      TWCR = (1<<TWSTA)|(1<<TWEN)|(1<<TWINT);				/* Enable TWI, generate start condition and clear interrupt flag */
      while (!(TWCR & (1<<TWINT)));						/* Wait until TWI finish its current job (start condition) */
      rEgisterstatus = TWSR & 0xF8;								/* Read TWI status register with masking lower three bits */
      if (rEgisterstatus != 0x08)									/* Check weather start condition transmitted successfully or not? */
      continue;											/* If no then continue with start loop again */
      TWDR = aDdress;								/* If yes then write SLA+W in TWI data register */
      TWCR = (1<<TWEN)|(1<<TWINT);						/* Enable TWI and clear interrupt flag */
      while (!(TWCR & (1<<TWINT)));						/* Wait until TWI finish its current job (Write operation) */
      rEgisterstatus = TWSR & 0xF8;								/* Read TWI status register with masking lower three bits */
      if (rEgisterstatus != 0x18 ){								/* Check weather SLA+W transmitted & ack received or not? */
        endTransmission();										/* If not then generate stop condition */
        continue;										/* continue with start loop again */
      }
    break;												/* If yes then break loop */
    }
  }

  uint8_t beginReTransmission(char aDdress)				/* I2C repeated start function */
  {
  	uint8_t status;											/* Declare variable */
  	TWCR = (1<<TWSTA)|(1<<TWEN)|(1<<TWINT);					/* Enable TWI, generate start condition and clear interrupt flag */
  	while (!(TWCR & (1<<TWINT)));							/* Wait until TWI finish its current job (start condition) */
  	status = TWSR & 0xF8;									/* Read TWI status register with masking lower three bits */
  	if (status != 0x10)										/* Check weather repeated start condition transmitted successfully or not? */
  	return 0;												/* If no then return 0 to indicate repeated start condition fail */
  	TWDR = aDdress;									/* If yes then write SLA+R in TWI data register */
  	TWCR = (1<<TWEN)|(1<<TWINT);							/* Enable TWI and clear interrupt flag */
  	while (!(TWCR & (1<<TWINT)));							/* Wait until TWI finish its current job (Write operation) */
  	status = TWSR & 0xF8;									/* Read TWI status register with masking lower three bits */
  	if (status == 0x40)										/* Check weather SLA+R transmitted & ack received or not? */
  	return 1;												/* If yes then return 1 to indicate ack received */
  	if (status == 0x20)										/* Check weather SLA+R transmitted & nack received or not? */
  	return 2;												/* If yes then return 2 to indicate nack received i.e. device is busy */
  	else
  	return 3;												/* Else return 3 to indicate SLA+W failed */
  }

  uint8_t write(char dAta){
    uint8_t stAtus;		/* Declare variable */
    TWDR=dAta;			/* Copy data in TWI data register */
    TWCR=(1<<TWEN)|(1<<TWINT);	/* Enable TWI and clear interrupt flag */

    while(!(TWCR&(1<<TWINT)));	/* Wait until TWI finish its current job */

    stAtus=TWSR&0xF8;		/* Read TWI status register */
    if(stAtus==0x28)		/* Check for data transmitted &ack received */
      return 0;			/* Return 0 to indicate ack received */
    if(stAtus==0x30)		/* Check for data transmitted &nack received */
      return 1;			/* Return 1 to indicate nack received */
    else
      return 2;			/* Else return 2 for data transmission failure */
  }

  char ackRead(){
    TWCR=(1<<TWEN)|(1<<TWINT)|(1<<TWEA); /* Enable TWI, generation of ack */
    while(!(TWCR&(1<<TWINT)));	/* Wait until TWI finish its current job */
    return TWDR;			/* Return received data */
  }

  char nackRead(){
    TWCR=(1<<TWEN)|(1<<TWINT);	/* Enable TWI and clear interrupt flag */
    while(!(TWCR&(1<<TWINT)));	/* Wait until TWI finish its current job */
    return TWDR;		/* Return received data */
  }

  void endTransmission(){
    TWCR=(1<<TWSTO)|(1<<TWINT)|(1<<TWEN);/* Enable TWI, generate stop */
    while(TWCR&(1<<TWSTO));	/* Wait until stop condition execution */
  }

  uint8_t requestFrom(uint8_t aDdress, uint8_t qUantity, uint32_t iAddress, uint8_t iSize, uint8_t sEndStop){
    if(iSize>0){
      // send internal address; this mode allows sending a repeated start to access
      // some devices' internal registers. This function is executed by the hardware
      // TWI module on other processors (for example Due's TWI_IADR and TWI_MMR registers)

      beginTransmission(aDdress);

      //maximum size of internal address can be 3 only
      if(iSize>3){
        iSize=3;
      }

      //write internal register address minus most significant bit
      while(iSize-->0){
        write((uint8_t)(iAddress>>(iSize*8)));
        beginReTransmission(aDdress);
      }
    }
  }

//SLAVE PART

  void begin(uint8_t slaveAddress){  //Begin function for Slave device
    TWAR=slaveAddress;		//Assign Address to Slave device
    TWCR=(1<<TWEN)|(1<<TWEA)|(1<<TWINT);/* Enable TWI, Enable ack generation */
  }

  int8_t onReceive(void (*recFunc)(void)){
    while(1)
    {
      cAllfunc=recFunc;
    	uint8_t stAtus;			/* Declare variable */
    	while(!(TWCR&(1<<TWINT)));	/* Wait to be addressed */

      stAtus=TWSR&0xF8;		/* Read TWI status register */
    	if(stAtus==0x60||stAtus==0x68)	/* Own SLA+W received &ack returned */
    	 cAllfunc();			/* Return 0 to indicate ack returned */
    	if(stAtus==0xA8||stAtus==0xB0)	/* Own SLA+R received &ack returned */
    	 return 1;			/* Return 0 to indicate ack returned */
    	if(stAtus==0x70||stAtus==0x78)	/* General call received &ack returned */
    	 return 2;			/* Return 1 to indicate ack returned */
    	else
    	 continue;			/* Else continue */
    }
  }

  char read()
  {
  	uint8_t stAtus;								/* Declare variable */
  	TWCR=(1<<TWEN)|(1<<TWEA)|(1<<TWINT);		/* Enable TWI, generation of ack and clear interrupt flag */
  	while (!(TWCR & (1<<TWINT)));				/* Wait until TWI finish its current job (read operation) */
  	stAtus = TWSR & 0xF8;						/* Read TWI stAtus register with masking lower three bits */
  	if (stAtus == 0x80 || stAtus == 0x90)		/* Check weather data received & ack returned (TWEA = 1) */
  	return TWDR;								/* If yes then return received data */
  	if (stAtus == 0x88 || stAtus == 0x98)		/* Check weather data received, nack returned and switched to not addressed slave mode */
  	return TWDR;								/* If yes then return received data */
  	if (stAtus == 0xA0)							/* Check weather STOP/REPEATED START received */
  	{
  		TWCR |= (1<<TWINT);						/* If yes then clear interrupt flag & return 0 */
  		return -1;
  	}
  	else
  	return -2;									/* Else return 1 */
  }
};

TwoWire Wire;
