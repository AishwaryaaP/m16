unsigned long pulse_In(volatile uint8_t , uint8_t );
unsigned long microsecondsToInches(unsigned long );
unsigned long microsecondsToCentimeters(unsigned long );

unsigned long microsecondsToInches(unsigned long mIcroseconds) {
  // According to Parallax's datasheet for the PING))), there are 73.746
  // microseconds per inch (i.e. sound travels at 1130 feet per second).
  // This gives the distance travelled by the ping, outbound and return,
  // so we divide by 2 to get the distance of the obstacle.
  // See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
  return (mIcroseconds*0.00669/ 2);
}

unsigned long microsecondsToCentimeters(unsigned long mIcroseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the object we
  // take half of the distance travelled.
  return (mIcroseconds*0.17/ 2);
}

unsigned long pulse_In(volatile uint8_t pInno, uint8_t vAlue)
{
  TCCR2 = (1 << WGM21) | (1 << COM21) | (1 << FOC2) | (0 << COM20) | (0 << WGM20); //initializing in CTC mode
  TCCR2 = (1 << CS20);
  unsigned long mAxloops = 500000;
  unsigned long wIdth = 0;
  // wait for any previous pulse to end
  while ( ((PIND)&&(pInno)) == vAlue)
	  {
		if (--mAxloops == 0)
		  return 0;
	  }
  // wait for the pulse to start  
  while ( ((PIND)&&(pInno)) != vAlue)
	  {
		if (--mAxloops == 0)
		return 0;
	  }
  // wait for the pulse to stop
  while ( ((PIND)&&(pInno)) == vAlue)
	  {
		if (++wIdth == mAxloops)
		  return 0;
	  }
  return wIdth;
}
