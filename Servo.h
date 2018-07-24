void MOVE_RIGHT(int ANG)
{
	
	
		DDRD   |= (1<<PD5);
		TCCR1A |= (1<<WGM11) | (1<<COM1A1) | (1<<COM1A0);
		TCCR1B |= (1<<WGM12) | (1<<WGM13);
		TCCR1B |= (1<<CS10);
		
		ICR1 = 20000;
		
		OCR1A = ICR1-(1500-(ANG*5.55555555));
		
		
	
} 
void MOVE_LEFT(int ANG)
{
	
	
		DDRD   |= (1<<PD5);
		TCCR1A |= (1<<WGM11) | (1<<COM1A1) | (1<<COM1A0);
		TCCR1B |= (1<<WGM12) | (1<<WGM13);
		TCCR1B |= (1<<CS10);
		
		ICR1 = 20000;
		
		OCR1A = ICR1-(1500+(ANG*5.55555555));
		
		
	
} 
