// lab3.c
// Victor Garcia Flores
// 11.01.2019

//  HARDWARE SETUP:
//  PORTA is connected to the segments of the LED display. and to the pushbuttons.
//  PORTA.0 corresponds to segment a, PORTA.1 corresponds to segement b, etc.
//  PORTB bit 1 goes RCLK and CLK on the 74HC595 and 74HC165 respectively.
//  PORTC bits 0-2 go to a,b,c inputs of the 74HC138.
//  PORTC.3 goes to the PWM transistor base.
// 	PORTE.6 goes to SH/LD for the 74HC165
// 	PORTE.7 goes to CLK_INH for the 74HC165

#define F_CPU 16000000 // cpu speed in hertz
#define TRUE 1
#define FALSE 0
#include <avr/io.h>
#include <math.h>
#include <util/delay.h>
#include <avr/interrupt.h>

//holds data to be sent to the segments. logic zero turns segment on
uint8_t segment_data[5]={0,0,0b11111100,0,0,0};

//decimal to 7-segment LED display encodings, logic "0" turns on segment
//Note: They are arranged so that the value of a possible integer matched with the position
uint8_t dec_to_7seg[12] = {0b11000000, 0b11111001,0b10100100,0b10110000,0b10011001,0b10010010,0b10000010,0b11111000,0b10000000,0b10011000,0b01111111,0b11111111};

//Real-time clock counters
uint8_t OneSecCount;
uint8_t seconds;
uint8_t minutes = 2;
uint8_t hours = 0;

//Colon Variable
uint8_t colon = 0x01;

/***********************************************************************/
//                              tcnt0_init
//Initalizes timer/counter0 (TCNT0). TCNT0 is running in async mode
//with external 32khz crystal.  Runs in normal mode with no prescaling.
//Interrupt occurs at overflow 0xFF.
//******************************************************************************
void tcnt0_init(void){
  //ASSR   |= (1<<AS0); //ext osc TOSC
  TIMSK  |=  (1<<TOIE0); //enable TCNT0 overflow interrupt
  TCCR0  |=  (1<<CS00); //normal mode, no prescale
}

uint8_t segMapper(uint8_t val){
  uint8_t mapped_val;
  mapped_val = dec_to_7seg[val];
  return mapped_val;
}

void digParser(uint8_t hrs, uint8_t mins){
  uint8_t mins_OnesVal;
  uint8_t mins_TensVal;
  uint8_t hrs_OnesVal;
  uint8_t hrs_TensVal;

  //minutes
  mins_OnesVal = mins % 10;
  segment_data[0] = segMapper(mins_OnesVal);

  mins_TensVal = (mins/10) % 10;
  segment_data[1] = segMapper(mins_TensVal);

  //hours
  hrs_OnesVal = hrs % 10;
  segment_data[3] = segMapper(hrs_OnesVal);

  hrs_TensVal = (hrs/10) % 10;
  segment_data[4] = segMapper(hrs_TensVal);
}

void testCLK_disp(int hrs, int mins){
  //if hrs is zero (military)
  uint8_t mins_OnesVal;
  uint8_t mins_TensVal;
  uint8_t hrs_OnesVal;
  uint8_t hrs_TensVal;

  //minutes
  mins_OnesVal = mins % 10;
  segment_data[0] = mins_OnesVal;

  mins_TensVal = (mins/10) % 10;
  segment_data[1] = mins_TensVal;

  //hours
  hrs_OnesVal = hrs % 10;
  segment_data[3] = hrs_OnesVal;

  hrs_TensVal = (hrs/10) % 10;
  segment_data[4] = hrs_TensVal;

  //minute values
  if(mins < 10){
    //1st Set
    PORTC = 0x00;
    PORTA = dec_to_7seg[mins_OnesVal];
    _delay_ms(1);

    //2nd set (used for a zero place-holder)
    PORTC = 0x01;
		PORTA = dec_to_7seg[mins_TensVal];
    _delay_ms(1);
  }
  else if(mins >= 10){
    PORTC = 0x00;
    PORTA = dec_to_7seg[mins_OnesVal];
    _delay_ms(1);

    //2nd set
    PORTC = 0x01;
    PORTA = dec_to_7seg[mins_TensVal];
    _delay_ms(1);
  }

  //handle zero hours
  if(hrs==0){
    PORTC = 0x03;
    PORTA = 0b11000000; //value for zero
    _delay_ms(2);
  }
  else if((hrs>0) && (hrs<10)){
    PORTC = 0x03;
    PORTA = dec_to_7seg[hrs_OnesVal];
    _delay_ms(2);

    //make the fourth position blank
    PORTC = 0x04;
    PORTA = 0b11111111;
    _delay_ms(2);
  }
  else{
    //third dig.
    PORTC = 0x03;
    PORTA = dec_to_7seg[hrs_OnesVal];
    _delay_ms(2);

    //fourth dig.
    PORTC = 0x04;
    PORTA = dec_to_7seg[hrs_TensVal];
    _delay_ms(2);
  }
  if(colon == 0x01){
    PORTC = 0x02;
    PORTA = segment_data[2];
    _delay_ms(1);
  }
}

/***********************************************************************/
//                              Display_Seg
//Configure ports so that we can display on the 7-seg, and then call function
//that puts their values in the right position
//******************************************************************************
void Display_Seg(uint8_t hrs, uint8_t mins){
  //Makre PORTA an output
  DDRA = 0xFF;
  asm volatile("nop");
  asm volatile("nop");
  //disable tristate buffer for pushbutton switches
  PORTC = 0x00;

  //Parse Values and display them
  //Clock_Disp(hrs,mins);
  testCLK_disp(hrs, mins);
}

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
	PORTC = 0x04;
	PORTA = 0b11111111;
	_delay_ms(2);

	//Thousands
	PORTC = 0x04;
	PORTA = 0b11111111;
	_delay_ms(2);
}
ISR(TIMER0_OVF_vect){
  OneSecCount++;
  if(OneSecCount == 128){
    seconds ++;
    OneSecCount = 0;
    colon ^= 0x01;
    //If seconds is 60
    if(seconds == 60){
      minutes++; //then increase minutes
      seconds = 0; //and reset seconds

      //check to see if minute is 60
      if(minutes == 60){
        hours++; // increment the hour
        minutes = 0; // reset minutes

        // check to see hours
        if(hours == 24){
            hours = 0; //then it's back to start the day at 0 hours
        }
      }
    }
  }
  //Change displayed time
  Display_Seg(hours, minutes);
}
void SevnSgDisp(uint8_t select){
  DDRA = 0xFF;
  //Adjust the select bits
  if(select == 0){ //first digit
    PORTC = 0x00;
  }
  else if(select == 1){ //second digit
    PORTC = 0x01;
  }
  else if(select == 2){ //colon
    PORTC = 0x02;
  }
  else if(select == 3){ //third digit
    PORTC = 0x03;
  }
  else if(select == 4){ //4th dig
    PORTC = 0x04;
  }

  //Send values to display
  PORTA = segment_data[select];
}
int main(){
  //tcnt0_init();  //initalize counter timer zero
  DDRC = 0x0F; //set port bits 0-3 C as outputs
  asm volatile("nop");
  asm volatile("nop");
  DDRE = ((1<<PE7) | (1<<PE6)); //Outputs for CLK_INH and SHIFT_LN_N
  PORTE = ((1<<PE7) | (1<<PE6)); //By default, disable CLK_INH (don't want an output to QH yet) and SH/LD (active low)
	//sei();         //enable interrupts before entering loop

  uint8_t digSel=0x00;
  while(1){
    _delay_ms(2);
    digParser(hours, minutes);

    //display on seven segment
    if(digSel>4){
      digSel = 0;
    }
    SevnSgDisp(digSel);
    digSel++;
  }
}
