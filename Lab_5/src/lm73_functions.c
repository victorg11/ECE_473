// lm73_functions.c
// Roger Traylor 11.28.10

#include <util/twi.h>
#include "lm73_functions.h"
#include <util/delay.h>

//TODO: remove volatile type modifier?  I think so.
//TODO: initalize with more resolution and disable the smb bus timeout
//TODO: write functions to change resolution, alarm etc.

volatile uint8_t lm73_wr_buf[2];
volatile uint8_t lm73_rd_buf[2];

//********************************************************************************
//                    lm73_temp_convert(char, uint16_t, uint8_t)
// This functions takes its paramenters, including the raw data from the
// temperature sensor and performs the necessary operations to interpret the data.
// Note: This function was put together with the guidence of Jose Manuel Lopez Alcala
// on 12/04/2019.
//******************************************************************************
uint8_t lm73_temp_convert(char temp_digits[], uint16_t lm73_temp, uint8_t f_not_c){
//given a temperature reading from an LM73, the address of a buffer
//array, and a format (deg F or C) it formats the temperature into ascii in
//the buffer pointed to by the arguement.
//TODO:Returns what???(uint8_t)??? Probably a BUG?

//Yeah, this is for you to do! ;^)
  uint16_t Short_data_register  = (lm73_temp>>5);//data register with only 15:5
  temp_digits[0]=0;//set initial value to zero to denote empty array
  double temperature_C = 0;
  double temperature_F = 0;
  double outputVal=0;

  //check bits to get value
  uint8_t pos;//"for" loop variable
  for(pos=0;pos<8;pos++){
    //check to see if a bit in the data register is set and perform arithmetic
    if(Short_data_register & (1<<pos)){
      if(pos == 0){
        temperature_C += 0.25;
      }
      else if(pos == 1){
        temperature_C += 0.5;
      }
      else if(pos == 2){
        temperature_C += 1;
      }
      else if(pos == 3){
        temperature_C += 2;
      }
      else if(pos == 4){
        temperature_C += 4;
      }
      else if(pos == 5){
        temperature_C += 8;

      }
      else if(pos == 6){
        temperature_C += 16;
      }
      else if(pos == 7){
        temperature_C += 32;
      }
    }
  }
  outputVal = temperature_C;

  //Convert value to either Celsuis or Faranheit
  //f_not_c = 1 -> means to convert to Faranheit
  //f_not_c = 0 -> means to convert to leave in Celsuis
  if(f_not_c){
    //Formula: °F = ([temperature _value]°C × 9/5) + 32
    temperature_F = (temperature_C*1.8) + 32;
    outputVal = temperature_F;
  }
  //outputVal = -22.55; //test value for LCD display
  //take care of the display
  dtostrf(outputVal,5,2,temp_digits);//convert integer part to ascii

  //deal with unit symbol
  if (f_not_c == TRUE) {
    strcat(temp_digits,"F");
  }
  else if(f_not_c == FALSE){
    strcat(temp_digits, "C");
  }


}//lm73_temp_convert
//******************************************************************************

uint16_t get_rawData(){
  uint16_t raw_data = 0;
  twi_start_rd(LM73_ADDRESS, lm73_rd_buf, 2);//read temperature data from LM73 (2 bytes)
  _delay_ms(2);    //wait for it to finish
  raw_data = lm73_rd_buf[0];//save high temperature byte into lm73_temp
  raw_data = (raw_data<<8);//shift it into upper byte (pg. 17 of datasheet)
  raw_data |= lm73_rd_buf[1];//"OR" in the low temp byte to lm73_temp
  return raw_data;
}
