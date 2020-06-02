#define F_CPU 16000000 // cpu speed in hertz
#define TRUE 1
#define FALSE 0
#include <avr/io.h>
#include <math.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>

/***********************************************************************/
//                              tcnt1_init
//
//******************************************************************************
void tcnt1_init(void){
  TIMSK |= (1<<OCIE1A); //set tcnt1 compare match
	TCCR1B |= (1<<WGM12)|(1<<CS10); //CTC at TOP
	//OCR1A = 0x00FF; //TOP value, adjusts frequency

  //Make it sound at ~2k Hz
  OCR1A = 3999;
}
ISR(TIMER1_COMPA_vect)
{
	PORTD ^= (1<<PD0);
}
int main(){
  DDRD |= (1<<PD0);
  tcnt1_init();
  sei();
  while(1){
    //PORTE ^= (1<<PE0);
  }

}
