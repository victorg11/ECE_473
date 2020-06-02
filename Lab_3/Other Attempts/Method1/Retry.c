#define F_CPU 16000000 // cpu speed in hertz
#define TRUE 1
#define FALSE 0
#include <avr/io.h>
#include <math.h>
#include <util/delay.h>
#include <avr/interrupt.h>

//Temp Register Holders
uint8_t prev_PORTB;
uint8_t prev_PORTC;
uint8_t prev_DDRA;
uint8_t prev_PORTA;

//Global Button Variables
uint8_t ButtonState = 1; //increment and decrement value
uint8_t buttons[2] = {0}; // used to see which button was pressed

//Current Counter Value
uint16_t CurrCountVal = 1023;

//holds data to be sent to the segments. logic zero turns segment on
uint8_t segment_data[5];

//decimal to 7-segment LED display encodings, logic "0" turns on segment
//Note: They are arranged so that the value of a possible integer matched with the position
uint8_t dec_to_7seg[12] = {0b11000000, 0b11111001,0b10100100,0b10110000,0b10011001,0b10010010,0b10000010,0b11111000,0b10000000,0b10011000,0b01111111,0b11111111};

//fuction declaration
void segsum(uint16_t);

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

/*************************************************************************/
//                           bargraph_updater
/*************************************************************************/
void bargraph_updater(uint8_t state){
  uint8_t output = 0; //what the bargraph will display
  if(state == 0){ //when both buttons are pressed do nothing
    output = 0b00000000;
  }
  else if(state == 1){ //increment/decrement by 1
    output = 0b00000001;
  }
  else if(state == 2){//increment/decrement by 2
    output = 0b00000010;
  }
  else if(state == 4){//increment/decrement by 4
    output = 0b00000100;
  }

  /* Start transmission */
  SPDR = output;
  while (bit_is_clear(SPSR,SPIF)){} //spin till SPI data has been sent

  PORTD |= (1<<PD2);	    //send rising edge to regclk on HC595
  PORTD &= ~(1<<PD2);            //send falling edge to regclk on HC595
}

//******************************************************************************

void handle_BttnData(){
  //uint8_t temp[2] = {0};
  if(buttons[0] == 1 && buttons[1] == 1){ //if both buttons
    ButtonState = 0; //value we inc/dec by
  }
  else if(buttons[0] == 1 && buttons[1] == 0){//if first button
    ButtonState = 2; //value we inc/dec by
  }
  else if(buttons[0] == 0 && buttons[1] == 1){//if second button
    ButtonState = 4; //value we inc/dec by
  }
}

void Read_Buttons(){
  //Let's read button data
  DDRA = 0x00; //sets as input
  PORTA = 0xFF; //pulls up the resistors
  PORTC = ((1<<PC0)|(1<<PC1)|(1<<PC2)); //Select bits for the buttons

  for(int BttnNum = 0; BttnNum <= 1; BttnNum++){
    if(chk_buttons(BttnNum)){ //If we read button input
      if(BttnNum == 0){ //first button is pressed
        //ButtonState = 2; //value we inc/dec by
        buttons[0] = 1; //button array
      }
      else if(BttnNum == 1){ //second button is pressed
        //ButtonState = 4; //value we inc/dec by
        buttons[1] = 1; //button array
      }
      handle_BttnData();
    }
  }
  //handle_BttnData();
  //reset button button state
  buttons[0] = 0;
  buttons[1] = 0;
}

/*************************************************************************/
//                           timer/counter0 ISR
//*************************************************************************/
ISR(TIMER0_OVF_vect){
  //prev_DDRA = DDRA;
  asm("nop");
  asm("nop");
  //prev_PORTA = PORTA;
  //prev_PORTC = PORTC;

  Read_Buttons();

  //DDRA = prev_DDRA;
  asm("nop");
  asm("nop");
  //PORTA = prev_PORTA;
  //PORTC = prev_PORTC;

}

//***********************************************************************************
//                                   segment_sum
//takes a 16-bit binary input value and places the appropriate equivalent 4 digit
//BCD segment code in the array segment_data for display.
//array is loaded at exit as:  |digit3|digit2|colon|digit1|digit0|
void segsum(uint16_t sum) {
  //makre sure that butons are off
  PORTC = 0x00;
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
	_delay_ms(2);

	//Tens
	PORTC = 0x01;
	PORTA = 0b11111111;
	_delay_ms(2);

	//Hundreds
	PORTC = 0x03;
	PORTA = 0b11111111;
	_delay_ms(2);

	//Thousands
	PORTC = 0x04;
	PORTA = 0b11111111;
	_delay_ms(2);
}

int main()
{
  tcnt0_init();  //initalize counter timer zero
  spi_init();    //initalize SPI port
  DDRD = (1<<PD2); //REG_CLK output for bargraph
  PORTB |= (0<<PB7); //OE for bargraph (it's DDRx is set in spi_init())

  //set port bits 0-3 C as outputs
  DDRC = 0x0F;
  PORTC = (0<<PC3); //Set PWM to zero

  //enable interrupts before entering loop
  sei();

  //counter val.
  uint16_t CurrCountVal = 1023;
  while(1){
    //Clear 4 segments
		//AllSegments_BitClearer();

    // Let's display what the state is on the bargraph
    bargraph_updater(ButtonState);

    //Set PortA as an output with pullup resistors
    DDRA = 0xFF; //sets as output

    //disable tristate buffer for pushbutton switches
    PORTC = ((0<<PC0)|(0<<PC1)|(0<<PC2));

    //Break up digits and display them/
    segsum(CurrCountVal);
    _delay_ms(1);

  }
}
