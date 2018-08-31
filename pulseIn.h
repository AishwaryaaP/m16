#include <avr/io.h>
#include <util/delay.h>

#ifndef F_CPU
#define F_CPU 16000000UL
//SET CPU CLOCK
#endif

unsigned long pulse_In(volatile uint8_t bitno, uint8_t stateMask);

unsigned long microsecondsToInches(unsigned long microseconds1);
unsigned long microsecondsToCentimeters(unsigned long microseconds);

const int pingPin = PINC0;//trig
const int Pin = PIND0;//echo

unsigned long microsecondsToInches(unsigned long microseconds1) {
  // According to Parallax's datasheet for the PING))), there are 73.746
  // microseconds per inch (i.e. sound travels at 1130 feet per second).
  // This gives the distance travelled by the ping, outbound and return,
  // so we divide by 2 to get the distance of the obstacle.
  // See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
  return (microseconds1*0.00669/ 2);
}

unsigned long microsecondsToCentimeters(unsigned long microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the object we
  // take half of the distance travelled.
  return (microseconds*0.17/ 2);
}


unsigned long pulse_In(volatile uint8_t bitno, uint8_t stateMask)
{
  TCCR2 = (1 << WGM21) | (1 << COM21) | (1 << FOC2) | (0 << COM20) | (0 << WGM20); //initializing in CTC mode
  TCCR2 = (1 << CS20);
  unsigned long maxloops = 500000;
  unsigned long width = 0;
  // wait for any previous pulse to end
  while ( (PIND0) == stateMask)
	  {
		if (--maxloops == 0)
		  return 0;
	  }
  // wait for the pulse to start  
  while ((PIND0) != stateMask)
	  {
		if (--maxloops == 0)
		return 0;
	  }

  // wait for the pulse to stop
  while (PIND0 == stateMask)
	  {
		if (++width == maxloops)
		  return 0;
	  }
  //Serial.println(width);
  return width;
}