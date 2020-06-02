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
#include "hd44780.h"
#include <string.h>


//holds data to be sent to the segments. logic zero turns segment on
uint8_t segment_data[5]={0,0,0b11111100,0,0,0};

//decimal to 7-segment LED display encodings, logic "0" turns on segment
//Note: They are arranged so that the value of a possible integer matched with the position
uint8_t dec_to_7seg[12] = {0b11000000, 0b11111001,0b10100100,0b10110000,0b10011001,0b10010010,0b10000010,0b11111000,0b10000000,0b10011000,0b01111111,0b11111111};

//Real-time clock counters
uint8_t seconds;
int8_t hours = 0;
int8_t minutes = 0;

//Colon Variable
uint8_t colon = 0x01;

//ADC Variables
uint16_t last_adcVal;

//General Encoder Variables
uint8_t raw_encoder = 0; //raw data from break out board
//Encoder #1
uint8_t prevL_Encoder=0;
uint8_t currL_Encoder=0;
//Encoder #2
uint8_t prevR_Encoder=0;
uint8_t currR_Encoder=0;

//volatile raw segment data
uint16_t volatile encoder_test;

//Global Button Variables
uint8_t ButtonState = 1; //increment and decrement value
uint8_t buttons[8] = {0}; // used to see which button was pressed

//startup flag
uint8_t start_flag=0; //used for encoder

//variable for current value
int16_t CurrCountVal = 0;

//Button Variables
uint8_t ChangeTime = 0;
uint8_t ChangeAlarmTime = 0;
uint8_t AlarmOnOff = 0;
uint8_t Snooze = 0;
uint8_t Volumeup = 0;
uint8_t Volumedown = 0;
uint8_t buttonsToggled = 0;


//Alarm Managing
int8_t AlarmHrs = 12;
int8_t AlarmMins = 0;
uint8_t AlarmSounding = 0;
uint8_t SnoozeSecCounter = 0;

void adc_init(){
  //Initalize ADC and its ports
  DDRF  &= ~(_BV(DDF7)); //make port F bit 7 is ADC input
  PORTF &= ~(_BV(PF7));  //port F bit 7 pullups must be off

  ADMUX |= (0<<ADLAR) | (1<<REFS0)|(1<<MUX2)|(1<<MUX1)|(1<<MUX0) ; //single-ended, input PORTF bit 7, right adjusted, 10 bits

  ADCSRA |= (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);  //ADC enabled, don't start yet, single shot mode
                             //division factor is 128 (125khz)
}

void adc_read(){
  uint8_t adc_result;
  ADCSRA |= (1<<ADSC); //poke ADSC and start conversion
  while(bit_is_clear(ADCSRA,ADIF)){}; //spin while interrupt flag not set
  ADCSRA |= (1<<ADIF);//its done, clear flag by writing a one
  adc_result = ADC;                      //read the ADC output as 16 bits
  last_adcVal = div(adc_result, 205);
  OCR2 = adc_result;
}

/***********************************************************************/
//                            spi_init
//Initalizes the SPI port on the mega128. Does not do any further
//external device specific initalizations.  Sets up SPI to be:
//master mode, clock=clk/2, cycle half phase, low polarity, MSB first
//interrupts disabled, poll SPIF bit in SPSR to check xmit completion
/***********************************************************************/
void spi_init(void){
  //DDRD |= (1<<PD1); //regclk
  DDRB   |= ((1<<PB0)|(1<<PB1)|(1<<PB2)| (0<<PB3)); //output mode for MOSI, SCLK
  SPCR   = (1<<SPE) | (1<<MSTR); //master mode, clk low on idle, leading edge sample (p. 167)
  SPSR   = (1<<SPI2X); //choose double speed operation // double speed operation

  /* Run this code before attempting to write to the LCD.*/
  DDRF  |= 0x08;  //port F bit 3 is enable for LCD
  PORTF &= 0xF7;  //port F bit 3 is initially low

 }//spi_init

 /*********************************************************************/
 // 															spi_read
 //Reads the SPI port.
 /*********************************************************************/
 uint8_t spi_read(void){
 	SPDR = 0x00; //"dummy" write to SPDR
 	while (bit_is_clear(SPSR,SPIF)){} //wait till 8 clock cycles are done
 	return(SPDR); //return incoming data from SPDR
 }

/***********************************************************************/
//                              tcnt0_init
//Initalizes timer/counter0 (TCNT0). TCNT0 is running in async mode
//with external 32khz crystal.  Runs in normal mode with no prescaling.
//Interrupt occurs at overflow 0xFF.
//******************************************************************************
void tcnt0_init(void){
  ASSR   |= (1<<AS0); //ext osc TOSC
  TIMSK  |=  (1<<TOIE0); //enable TCNT0 overflow interrupt
  TCCR0  |=  (1<<CS00); //normal mode, no prescale
}

/***********************************************************************/
//                              tcnt1_init
// Initializes the configuration for the sound pins. I have selected
// CTC mode, no pre-scalar, with a frequency of 2k Hz
//******************************************************************************
void tcnt1_init(void){
  DDRD |= (1<<PD0);
	TCCR1B |= (1<<WGM12)|(1<<CS10); //CTC at TOP
  //Initialize the tone to be off
  OCR1A = 3999;
  TIMSK |= (1<<OCIE1A); //set tcnt1 compare match
}

/***********************************************************************/
//                              tcnt2_init
//Initalizes timer/counter0 (TCNT2). This is used to drive the PWM pin for the
//7-segment display
//******************************************************************************
void tcnt2_init(void){
  TIMSK  |=  (1<<TOIE2); //enable TCNT2 overflow interrupt
  TCCR2  |=  (1<<CS20) | (0<<CS21)|(1<<WGM20)|(1<<WGM21)| (1<<COM20) | (1<<COM21); //normal mode, no prescale
  OCR2 = 200;
}

/*************************************************************************/
//                           section_tester
//This is is used to test to see if we get to certain places in the code
//Whatever value is passed into this function will be presented onto the graph
/*************************************************************************/
void section_tester(uint8_t state){
  /* Start transmission */
  SPDR = state;
  while (bit_is_clear(SPSR,SPIF)){} //spin till SPI data has been sent

  PORTD |= (1<<PD1);	    //send rising edge to regclk on HC595
  PORTD &= ~(1<<PD1);            //send falling edge to regclk on HC595
}

void LCDUpdater(){
  clear_display();
  cursor_home();
  if(AlarmOnOff == 1){
    string2lcd("ALARM");
  }
}

void AlarmHandler(){
  //If the alarm isn't be sounding
  if(AlarmSounding == 0){
    //but is enabled
    if(AlarmOnOff){
      //check to see if the alarm should be going off
      if((hours == AlarmHrs) && (minutes == AlarmMins)){
        AlarmSounding = 1;
      }
    }
  }
  //if alarm is off, make it so that no sound plays
  else if(AlarmOnOff == 0){
    AlarmSounding = 0;
  }
  //If snooze was turned on
  if(Snooze){
    AlarmSounding = 0;
  }
  //if we have reached 10 sec of snooze, enable sound
  if(SnoozeSecCounter == 10){
    AlarmSounding = 1;
    Snooze = 0;
    SnoozeSecCounter = 0;
  }
  if(AlarmSounding){
    //make sound come up
    //section_tester(0b11111111);
    TIMSK |= (1<<OCIE1A);
    OCR1A = 3999; //What makes the sound go off
  }
  else if(AlarmSounding == 0){
    //make sure sound is off
    //section_tester(0b00000000);
    TIMSK |= (1<<OCIE1A);
    OCR1A = 0;
  }
}

//******************************************************************************
//                            chk_buttons
//Checks the state of the button number passed to it. It shifts in ones till
//the button is pushed. Function returns a 1 only once per debounced button
//push so a debounce and toggle function can be implemented at the same time.
//Adapted to check all buttons from Ganssel's "Guide to Debouncing"
//Expects active low pushbuttons on PINA port.  Debounce time is determined by
//external loop delay times 12.
//******************************************************************************
uint8_t chk_buttons(uint8_t button) {
	static uint16_t state[8] = {0}; //We do what we did in lab 1, but this time as an array so we can address the other buttons
	state[button] = ((state[button]<<1) | (!bit_is_clear(PINA,button)) | 0xE000);
	if(state[button] == 0xFF00) return 1;
	return 0;
}
void Read_ButtonsV2(){
	int BttnNum = 0;
	//Let's read button data
  DDRA = 0x00; //sets as input
  PORTA = 0xFF; //pulls up the resistors
  PORTB |= ((1<<PB4)|(1<<PB5)|(1<<PB6)); //Select bits for the buttons

  for(BttnNum = 0; BttnNum <= 7; BttnNum++){
    if(chk_buttons(BttnNum)){ //If we read button input
      if(BttnNum == 7){ //first button is pressed
        buttons[7] = 1; //button array
        ChangeTime ^= 1;

        //Clear other condition involving time
        ChangeAlarmTime = 0;
      }
      else if(BttnNum == 6){ //second button is pressed
        buttons[6] = 1; //button array
        ChangeAlarmTime ^= 1;

        //Clear other condition involving time
        ChangeTime = 0;
      }
      else if(BttnNum == 5){ //third button is pressed
        buttons[5] = 1; //button array
        AlarmOnOff ^= 1;
        buttonsToggled = 1;
      }
      else if(BttnNum == 4){ //fourth button is pressed
        buttons[4] = 1; //button array
        Snooze ^= 1;
      }
      else if(BttnNum == 3){ //fifth button is pressed
        buttons[3] = 1; //button array
        //Volumeup ^= 1;
      }
      else if(BttnNum == 2){ //sixth button is pressed
        buttons[2] = 1; //button array
        //Volumedown ^= 1;

      }
    }
  }
  //reset button state
  int i;
  for (i=0;i<=8;i++){
    buttons[i] = 0;
  }
}

void CLKBounds(){
  //If minutes is set to be 60+
  if(minutes>59){
    minutes = 0;
    hours++;
    if(hours > 23){
      hours == 0;
    }
  }
  //If hours is set to be 24+
  if(hours > 23){
    hours = 0;
  }

  if(hours<0){
    hours = 23;
  }
  if(minutes < 0){
    minutes = 59;
    hours--;
    if(hours<0){
      hours = 23;
    }
  }
}

void AlarmBounds(){
  //If minutes is set to be 60+
  if(AlarmMins>59){
    AlarmMins = 0;
    AlarmHrs++;
    if(AlarmHrs > 23){
      AlarmHrs == 0;
    }
  }
  //If hours is set to be 24+
  if(AlarmHrs > 23){
    AlarmHrs = 0;
  }

  if(AlarmHrs<0){
    AlarmHrs = 23;
  }
  if(AlarmMins < 0){
    AlarmMins = 59;
    AlarmHrs--;
    if(AlarmHrs<0){
      AlarmHrs = 23;
    }
  }
}

/*************************************************************************/
//                           bargraph_updater
//Used to update bargraph values with inc/decrement value
//The scalar inc/dec value will be displayed in binary
/*************************************************************************/
void bargraph_updater(){
  uint8_t output = 0; //what the bargraph will display
  if(ChangeTime == 1){ //when both buttons are pressed do nothing
    output = 0b00000001;
  }
  else if(ChangeAlarmTime == 1){ //increment/decrement by 1
    output = 0b00000010;
  }
  //commented out because the armed
  // else if(Set_Alarm == 1){//increment/decrement by 2
  //   output = 0b00000100;
  // }

  //Commented Out because Snooze should be on LCD display
  // else if(Snooze == 1){//increment/decrement by 4
  //   output = 0b00000100;
  // }]
  else if(Volumeup == 1){//increment/decrement by 4
    output = 0b00001000;
  }
  else if(Volumedown == 1){//increment/decrement by 4
    output = 0b00010000;
  }
  else{
    output = 0b00000000;
  }

  /* Start transmission */
  SPDR = output;
  while (bit_is_clear(SPSR,SPIF)){} //spin till SPI data has been sent

  PORTD |= (1<<PD1);	    //send rising edge to regclk on HC595
  PORTD &= ~(1<<PD1);            //send falling edge to regclk on HC595
}

/*********************************************************************/
// 															Encoder_Data
//Toggles SHIFT_LN_N on parallel shift register to get data into the flip flops
//Sets CLK_INH to low so we can read from QH.
//Remember: Most significant bit is at position H
/*********************************************************************/
void Encoder_Data(){
	int i;
	//Remember: PE6-> SHIFT_LN_N and PE7-> CLK_INH
	//Toggle SH_LD to get their values into the flip flops
	PORTE ^= (1<<PE6);
	PORTE ^= (1<<PE6);

	//Output to through QH by changing CLK_INH
	PORTE ^= (1<<PE7);//CLK_INH
	raw_encoder = spi_read();

	//Stop the output
	PORTE ^= (1<<PE7);//CLK_INH

	//left Encoder
	currL_Encoder = raw_encoder;
	// get rid of LHS bits
	// what we want: 0bxx
  for (i=7; i>1; i--) {
    currL_Encoder &= ~(1<<i);
  }

	//Right encoder
	currR_Encoder = (raw_encoder>>2);
	//get rid of LHS bits
	// what we want (0bxx)
	for (i=7; i>1;i--) {
		currR_Encoder &= ~(1<<i);
	}

	//If it's a first time start-up
	if(start_flag == 0){
		prevL_Encoder = currL_Encoder;//set them equal
		prevR_Encoder = currR_Encoder;//set them equal
		start_flag = 1;
	}

  // --------- LEFT ENCODER ----------//
  if(currL_Encoder == 0b11 && prevL_Encoder == 0b01){
    if(ChangeTime){
      hours += 1;
      seconds = 0;
    }
    if(ChangeAlarmTime){AlarmHrs += 1;}
    prevL_Encoder = currL_Encoder;
  }
  else if(currL_Encoder == 0b11 && prevL_Encoder == 0b10){
    if(ChangeTime){
      hours -= 1;
      seconds = 0;
    }
    if(ChangeAlarmTime){AlarmHrs -= 1;}
    prevL_Encoder = currL_Encoder;
  }
  else{
    prevL_Encoder = currL_Encoder;
  }
  // --------- RIGHT ENCODER ----------//
  if(currR_Encoder == 0b11 && prevR_Encoder == 0b01){
    if(ChangeTime){
      minutes += 1;
      seconds = 0;
    }
    if(ChangeAlarmTime){AlarmMins += 1;}
    prevR_Encoder = currR_Encoder;
  }
  else if(currR_Encoder == 0b11 && prevR_Encoder == 0b10){
    if(ChangeTime){
      minutes -= 1;
      seconds = 0;
    }
    if(ChangeAlarmTime){AlarmMins -= 1;}
    prevR_Encoder = currR_Encoder;
  }
  else{
    prevR_Encoder = currR_Encoder;
  }
  //Make sure the alarm time and clock time are bounded to military time
  CLKBounds();
  AlarmBounds();
}

// void testCLK_disp(int hrs, int mins){
//   //if hrs is zero (military)
//   uint8_t mins_OnesVal;
//   uint8_t mins_TensVal;
//   uint8_t hrs_OnesVal;
//   uint8_t hrs_TensVal;
//
//   //minutes
//   mins_OnesVal = mins % 10;
//   segment_data[0] = mins_OnesVal;
//
//   mins_TensVal = (mins/10) % 10;
//   segment_data[1] = mins_TensVal;
//
//   //hours
//   hrs_OnesVal = hrs % 10;
//   segment_data[3] = hrs_OnesVal;
//
//   hrs_TensVal = (hrs/10) % 10;
//   segment_data[4] = hrs_TensVal;
//
//   //minute values
//   if(mins < 10){
//     //1st Set
//     PORTC = 0x00;
//     PORTA = dec_to_7seg[mins_OnesVal];
//     _delay_ms(1);
//
//     //2nd set (used for a zero place-holder)
//     PORTC = 0x01;
// 		PORTA = dec_to_7seg[mins_TensVal];
//     _delay_ms(1);
//   }
//   else if(mins >= 10){
//     PORTC = 0x00;
//     PORTA = dec_to_7seg[mins_OnesVal];
//     _delay_ms(1);
//
//     //2nd set
//     PORTC = 0x01;
//     PORTA = dec_to_7seg[mins_TensVal];
//     _delay_ms(1);
//   }
//
//   //handle zero hours
//   if(hrs==0){
//     PORTC = 0x03;
//     PORTA = 0b11000000; //value for zero
//     _delay_ms(1);
//   }
//   else if((hrs>0) && (hrs<10)){
//     PORTC = 0x03;
//     PORTA = dec_to_7seg[hrs_OnesVal];
//     _delay_ms(1);
//
//     //make the fourth position blank
//     PORTC = 0x04;
//     PORTA = 0b11111111;
//     _delay_ms(1);
//   }
//   else{
//     //third dig.
//     PORTC = 0x03;
//     PORTA = dec_to_7seg[hrs_OnesVal];
//     _delay_ms(1);
//
//     //fourth dig.
//     PORTC = 0x04;
//     PORTA = dec_to_7seg[hrs_TensVal];
//     _delay_ms(1);
//   }
//   if(colon == 0x01){
//     PORTC = 0x02;
//     PORTA = segment_data[2];
//     _delay_ms(1);
//   }
// }


uint8_t segMapper(uint8_t val){
  uint8_t mapped_val;
  mapped_val = dec_to_7seg[val];
  return mapped_val;
}

void TimedigParser(uint8_t hrs, uint8_t mins){
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

void SevnSgDisp(uint8_t select){
  DDRA = 0xFF;
  //Adjust the select bits
  if(select == 0){ //first digit
    PORTB = 0x00;
  }
  else if(select == 1){ //second digit
    PORTB = 0x10;
  }
  else if(select == 2){ //colon
    PORTB = 0x20;
  }
  else if(select == 3){ //third digit
    PORTB = 0x30;
  }
  else if(select == 4){ //4th dig
    PORTB = 0x40;
  }

  //Send values to display
  PORTA = segment_data[select];
}

void AllSegments_BitClearer(){
	DDRA = 0xFF;
  asm volatile("nop");
  asm volatile("nop");

	//Ones
	PORTB = 0x00;
	PORTA = 0b11111111;
	_delay_ms(1);

	//Tens
	PORTB = 0x10;
	PORTA = 0b11111111;
	_delay_ms(1);

	//Hundreds
	PORTB = 0x30;
	PORTA = 0b11111111;
	_delay_ms(1);

	//Thousands
	PORTB = 0x40;
	PORTA = 0b11111111;
	_delay_ms(1);
}
ISR(TIMER0_OVF_vect){
  static uint8_t OneSecTempCount=0;
  OneSecTempCount++;

  if((OneSecTempCount % 128) == 0){
    seconds ++;
    //colon handler
    colon ^= 0x01;
    if(colon == 0x01){
      segment_data[2] = 0b11111100;
    }
    else{
      segment_data[2] = 0b00000111;
    }

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
    //handle snooze count if enabled
    if(Snooze == 1){
      SnoozeSecCounter++;
    }
  }
  //Handle LCD
  if(buttonsToggled){
    LCDUpdater();
    buttonsToggled = 0;
  }
  //Used for brightness adjusting
  if((OneSecTempCount % 32) == 0){
   adc_read();
  }
  Encoder_Data();
}
ISR(TIMER1_COMPA_vect){
  PORTD ^= (1<<PD0);
}
ISR(TIMER2_OVF_vect){
}
int main(){
  DDRD |= (1<<PD1) | (1<<PD0);
  DDRB |= 0xF0; //set port bits 4-7 B as outputs
  DDRC |= (1<<PC0);
  DDRE = ((1<<PE7) | (1<<PE6)); //Outputs for CLK_INH and SHIFT_LN_N
  tcnt0_init();  //initalize counter timer zero
  tcnt1_init(); //Alarm initializer
  tcnt2_init(); //Diming initializer
  spi_init();    //initalize SPI port
  adc_init(); // adc initializer
  lcd_init(); //lcd initializer
  // DDRD |= (1<<PD1) | (1<<PD0);
  // DDRB |= 0xF0; //set port bits 4-7 B as outputs
  // DDRC |= (1<<PC0);
  asm volatile("nop");
  asm volatile("nop");
  // DDRE = ((1<<PE7) | (1<<PE6)); //Outputs for CLK_INH and SHIFT_LN_N
  PORTE = ((1<<PE7) | (1<<PE6)); //By default, disable CLK_INH (don't want an output to QH yet) and SH/LD (active low)
	sei();         //enable interrupts before entering loop

  uint8_t digSel=0x00;
  while(1){
    _delay_ms(1);
    //OCR1A = 4000;
    Read_ButtonsV2();
    // ------- display on seven segment ------- //
    //If we aren't changing alarm time, then display regular time
    if(ChangeAlarmTime != 1){
      TimedigParser(hours, minutes);
    }

    //If we are changing alarm time, show the alarm time on 7-seg
    else if(ChangeAlarmTime == 1){
      TimedigParser(AlarmHrs, AlarmMins);
    }
    if(digSel>4){
      digSel = 0;
    }
    SevnSgDisp(digSel);
    digSel++;
  // ---------------------------------------- //
    bargraph_updater();
    AlarmHandler();
  }
}
