#include <avr/io.h>
#include <util/delay.h>

int main()
{
	DDRB |= ((1<<DDB4)|((1<<DDB5))|((1<<DDB6))| (1<<DDB7));  //set port B to all outputs (testing)
	DDRA |= 0xFF;

	
	PORTB = 0; //initialize output
	PORTB |= (1<<PORTB7);//enable pwm

	PORTB |= ((0<<PORTB4)|(0<<PORTB5)|(1<<PORTB6));//try first digit
	PORTA = 0xFF;
	
	while(1){     //do forever
		_delay_ms(2);                    //keep in loop to debounce 24ms
  	} //while
} //main
