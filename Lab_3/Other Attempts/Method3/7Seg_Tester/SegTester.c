#define F_CPU 16000000 // cpu speed in hertz
#define TRUE 1
#define FALSE 0
#include <avr/io.h>
#include <math.h>
#include <util/delay.h>
#include <avr/interrupt.h>

//holds data to be sent to the segments. logic zero turns segment on
uint8_t segment_data[5];

//decimal to 7-segment LED display encodings, logic "0" turns on segment
//Note: They are arranged so that the value of a possible integer matched with the position
uint8_t dec_to_7seg[12] = {0b11000000, 0b11111001,0b10100100,0b10110000,0b10011001,0b10010010,0b10000010,0b11111000,0b10000000,0b10011000,0b01111111,0b11111111};

/***********************************************************************/
//                            spi_init
//Initalizes the SPI port on the mega128. Does not do any further
//external device specific initalizations.  Sets up SPI to be:
//master mode, clock=clk/2, cycle half phase, low polarity, MSB first
//interrupts disabled, poll SPIF bit in SPSR to check xmit completion
/***********************************************************************/
void spi_init(void){
  DDRD = (1<<PD2);
  DDRB   = ((1<<PB0)|(1<<PB1)|(1<<PB2)); //output mode for MOSI, SCLK
  SPCR   = (1<<SPE) | (1<<MSTR); //master mode, clk low on idle, leading edge sample (p. 167)
  SPSR   = (1<<SPI2X); //choose double speed operation // double speed operation
 }//spi_init

 /*************************************************************************/
 //                           timer/counter0 ISR
 //*************************************************************************/
 ISR(TIMER0_OVF_vect){
  // Read_Buttons();
 }

 /***********************************************************************/
 //                              tcnt0_init
 //Initalizes timer/counter0 (TCNT0). TCNT0 is running in async mode
 //with external 32khz crystal.  Runs in normal mode with no prescaling.
 //Interrupt occurs at overflow 0xFF.
 //
 void tcnt0_init(void){
   //ASSR   |= (1<<AS0); //ext osc TOSC
   TIMSK  |=  (1<<TOIE0); //enable TCNT0 overflow interrupt
   TCCR0  |=  (1<<CS00); //normal mode, no prescale
 }

//******************************************************************************
//                            chk_buttons
//Checks the state of the button number passed to it. It shifts in ones till
//the button is pushed. Function returns a 1 only once per debounced button
//push so a debounce and toggle function can be implemented at the same time.
//Adapted to check all buttons from Ganssel's "Guide to Debouncing"
//Expects active low pushbuttons on PINA port.  Debounce time is determined by
//external loop delay times 12.
//
uint8_t chk_buttons(uint8_t button) {
	static uint16_t state[8] = {0}; //We do what we did in lab 1, but this time as an array so we can address the other buttons
	state[button] = ((state[button]<<1) | (!bit_is_clear(PINA,button)) | 0xFE00);
	if(state[button] == 0xFF00) return 1;
	return 0;
}
//******************************************************************************


//***********************************************************************************
//                                   segment_sum
//takes a 16-bit binary input value and places the appropriate equivalent 4 digit
//BCD segment code in the array segment_data for display.
//array is loaded at exit as:  |digit3|digit2|colon|digit1|digit0|
void segsum(uint16_t sum) {
	//Variables for values digit  positions
	uint8_t OnesVal;
	uint8_t TensVal;
	uint8_t HundredsVal;
	uint8_t ThousandsVal;

	//Decoder "Sel#" Positions for Digits. Note: The lower bytes are 0 because we aren't using them in PORTC. This also keeps in mind the value we desire in PWN
	//determine how many digits there are
	int NumDigits = 0;
	int tempSum = sum;
	while(sum){
		tempSum /= 10;
		NumDigits++;
	}

	//break up decimal sum into 4 digit-segments
	//---ONES---
	OnesVal = sum % 10;
	segment_data[0] = OnesVal;

	//--- Tens ---
	TensVal = (sum/10) % 10;
	segment_data[1] = TensVal;

	//--- HUNDREDS ---
	HundredsVal = (sum/100) % 10;
	segment_data[3] = HundredsVal;

	//--- THOUSANDS ---
	ThousandsVal = (sum/1000) % 10;
	segment_data[4] = ThousandsVal;

	//DDRA = 0xFF; //Make PORT A an OUTPUT
	if(sum<10){ //if there is only one digit
		//1st Set
		PORTC = 0x00;
		PORTA = dec_to_7seg[OnesVal];
		_delay_ms(2);
		PORTC = 0x04;
		PORTA = 0b11111111;
	}
	else if((sum >= 10) && (sum < 100)){ //if there are two digits
		//1st Set
		PORTC = 0x00;
		PORTA = dec_to_7seg[OnesVal];
		_delay_ms(2);

		//2nd Set
		PORTC = 0x01;
		PORTA = dec_to_7seg[TensVal];

	}
	else if((sum>=100)&&(sum<1000)){ //if there are three digits
		//1st Set
		PORTC = 0x00;
		PORTA = dec_to_7seg[OnesVal];
		_delay_ms(2);

		//2nd Set
		PORTC = 0x01;
		PORTA = dec_to_7seg[TensVal];
		_delay_ms(2);

		//3rd Set
		PORTC = 0x03;
		PORTA = dec_to_7seg[HundredsVal];
	}
	else if(sum>= 1000){ //if there are four digits
		//1st Set
		PORTC = 0x00;
		PORTA = dec_to_7seg[OnesVal];
		_delay_ms(2);

		//2nd Set
		PORTC = 0x01;
		PORTA = dec_to_7seg[TensVal];
		_delay_ms(2);

		//3rd Set
		PORTC = 0x03;
		PORTA = dec_to_7seg[HundredsVal];
		_delay_ms(2);

		//4th Set. Note: No segments need clearing.
		PORTC = 0x04;
		PORTA = dec_to_7seg[ThousandsVal];
	}
}//segment_sum
//**********************************************************************************


//***********************************************************************************
// Function Name:void AllSegments_BitClearer
// This function is put to clear previous digit values on the seven segment display.
// Goal: The goal is to avoid ghosting and help set un-used segments to zero.
void AllSegments_BitClearer(){
	DDRA = 0xFF;
        asm volatile("nop");
        asm volatile("nop");
	//Ones
	PORTC = 0x00;
	PORTA = 0b11111111;
	//_delay_ms(2);

	//Tens
	PORTC = 0x01;
	PORTA = 0b11111111;
	//_delay_ms(2);

	//Hundreds
	PORTC = 0x03;
	PORTA = 0b11111111;
//	_delay_ms(2);

	//Thousands
	PORTC = 0x04;
	PORTA = 0b11111111;
	//_delay_ms(2);
}

int main()
{
  //Set PortA as an output with pullup resistors
  DDRA = 0xFF; //sets as output
  //PORTA = 0xFF; //pulls up the resistors

  //set port bits 4-7 C as outputs
  DDRC = 0x0F;

  //counter val.
  uint16_t CurrCountVal = 1023;
  while(1){
    //Clear 4 segments
		//AllSegments_BitClearer();

    //disable tristate buffer for pushbutton switches
    PORTC = 0x00;

    //Break up digits and display them/
    segsum(1023);
    _delay_ms(2);

  }
}
