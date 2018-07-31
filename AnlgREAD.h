#include<avr/io.h>
#define F_CPU 16000000UL

int algR(int x)
{



  ADMUX=(1<<REFS0)|(0<<REFS1);
  ADCSRA|=(1<<ADEN);
  ADMUX|=x;//chose value from 0 to 7 to chose adc pin accordingly
  ADCSRA|=(1<<ADEN);
  ADCSRA|=(1<<ADSC);
 while(ADCSRA&(1<<ADSC));
 return (ADC);
}



