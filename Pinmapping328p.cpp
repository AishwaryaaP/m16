/*
 * AVRGCC34.cpp
 *
 * Created: 02-10-2018 17:22:00
 *  Author: HP
 */ 

#include <avr/io.h>

int l;
int c[24]={1,2,3,4,5,6,7,8,1,2,3,4,5,6,7,8,1,2,3,4,5,6,7,8};

char d[24]={'b','b','b','b','b','b','b','b','c','c','c','c','c','c','c','c','d','d','d','d','d','d','d','d'};
void pinMode(unsigned int i,int tipe)
{
  if(tipe==0)
        { switch(d[i])
         {
	 
	          case'b':
	                  DDRB|=(0<<c[i]);
			           break;
	          case'c':
	                 DDRC|=(0<<c[i]);
			          break;
	          case'd':
	                 DDRD|=(0<<c[i]);
			          break;	  		 		 			 
	     }  }
	else{          switch(d[i])
         {
	 
	          case'b':
	                  DDRB|=(1<<c[i]);
			           break;
	          case'c':
	                 DDRC|=(1<<c[i]);
			          break;
	          case'd':
	                 DDRD|=(1<<c[i]);
			          break;	  		 		 			 
	     }  
}
		 }
 int main()
 {
	 pinMode(11,0);
	 pinMode(12,1);
	 	 pinMode(10,1);
 }
