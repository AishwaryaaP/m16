/*
 * AVRGCC34.cpp
 *
 * Created: 02-10-2018 17:22:00
 *  Author: HP
 */ 

#include <avr/io.h>



int x[24]={1,2,4,8,16,32,64,128,1,2,4,8,16,32,64,128,1,2,4,8,16,32,64,128};
char d[24]={'b','b','b','b','b','b','b','b','c','c','c','c','c','c','c','c','d','d','d','d','d','d','d','d'};
void pinMode(unsigned int i,short unsigned int tipe)
{
  
  switch(d[i])
  {
	case'b':
	         DDRB|=x[i];
			 break;
	case'c':
	          DDRC|=x[i];
			  break;
	case'd':
	         DDRD|=x[i];
			 break;		  		 		 			 
	  }
}
 int main()
 {
	 pinMode(11,0);
	 pinMode(12,0);
 }
