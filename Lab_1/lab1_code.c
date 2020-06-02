// Victor Garcia Flores
// ECE 473

//This program increments a binary display of the number of button pushes on switch 
//S0 on the mega128 board.

#include <avr/io.h>
#include <util/delay.h>

//*******************************************************************************
//                            debounce_switch                                  
// Adapted from Ganssel's "Guide to Debouncing"            
// Checks the state of pushbutton S0 It shifts in ones till the button is pushed. 
// Function returns a 1 only once per debounced button push so a debounce and toggle 
// function can be implemented at the same time.  Expects active low pushbutton on 
// Port D bit zero.  Debounce time is determined by external loop delay times 12. 
//*******************************************************************************
int8_t debounce_switch() {
	static uint16_t state = 0; //holds present state
	state = (state << 1) | (! bit_is_clear(PIND, 0)) | 0xE000;
	if (state == 0xF000) return 1;
	return 0;
}

//Function that detects the upper digit
uint8_t TensVal(uint8_t DispVal){
	uint8_t upperVal = 0;
	upperVal = (DispVal >> 4);
	return upperVal;
}

//*******************************************************************************
// Check switch S0.  When found low for 12 passes of "debounce_switch(), increment
// PORTB.  This will make an incrementing count on the port B LEDS. 
//*******************************************************************************
int main()
{
	DDRB = 0xFF;  //set port B to all outputs
	PORTB = 0; //initialize output
	uint8_t counter = 0;	
	uint8_t DispVal = 0;
	
	while(1){     //do forever
		if(debounce_switch()) {
			//PORTB++;
			counter++;
			if(counter == 100){ //if the counter counts to 100, then it's time to warm back around
				PORTB = 0;
				counter = 0; //restart count
				DispVal = 0;
			}
			if((counter < 10)){ //No Offset needed for digits less than 10	
				DispVal = counter;
				PORTB = DispVal;
			}
			if((counter >= 10) && (counter < 20)){ //Offset #1 = 6
				DispVal = counter + 6;
				PORTB = DispVal;	
			}
			if((counter >= 20) && (counter < 30)){ //Offset #2 = 12
				DispVal = counter + 12;
				PORTB = DispVal;
			}
			if((counter >= 30) && (counter < 40)){ //Offset #3 = 18
				DispVal = counter + 18;
				PORTB = DispVal;
			}
			if((counter >= 40) && (counter < 50)){ //Offset #4 = 24
				DispVal = counter + 24;
				PORTB = DispVal;
			}
			if((counter >= 50) && (counter < 60)){ //Offset #5 = 30
				DispVal = counter + 30;
				PORTB = DispVal;
			}
			if((counter >= 60) && (counter < 70)){ //Offset #6 = 36
				DispVal = counter + 36;
				PORTB = DispVal;
			}
			if((counter >= 70) && (counter < 80)){ //Offset #6 = 42
				DispVal = counter + 42;
				PORTB = DispVal;
			}
			if((counter >= 80) && (counter < 90)){ //Offset #8 = 48
				DispVal = counter + 48;
				PORTB = DispVal;
			}
			if((counter >= 90) && (counter < 100)){ //Offset #9 = 54
				DispVal = counter + 54;
				PORTB = DispVal;
			}

		}  //if switch true for 12 passes, increment port B
		_delay_ms(2);                    //keep in loop to debounce 24ms
  	} //while
} //main
