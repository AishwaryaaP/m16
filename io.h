void delay(unsigned long delay) {
  volatile unsigned long i = 0;
  for (i = 0; i < delay; i++) {
      __asm__ __volatile__ ("nop");
  }
}
int main()
{

	DDRC=0b00000001;//set output
	
	while(1)
	{
		PORTC=0b00000001;
		delayMicroseconds(1000);
		PORTC=0b00000000;
	}
}
