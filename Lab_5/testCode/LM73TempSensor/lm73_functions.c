// lm73_functions.c
// Roger Traylor 11.28.10

#include <util/twi.h>
#include "lm73_functions.h"
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>

//Bit Macros
#define BIT0 0
#define BIT1 1
#define BIT2 2
#define BIT3 3
#define BIT4 4
#define BIT5 5
#define BIT6 6
#define BIT7 7

//TODO: remove volatile type modifier?  I think so.
//TODO: initalize with more resolution and disable the smb bus timeout
//TODO: write functions to change resolution, alarm etc.

uint8_t lm73_wr_buf[2];
uint8_t lm73_rd_buf[2];

//********************************************************************************


/*********************************************************
Function: lm73_temp_convert()
Description:
  This function will take a tempearature reading, a Farenheit or Celcius
  flag, and a pointer to an array and then place the ASCII equivalent
  of the temperature reading in the array that is pointed to by the
  char pointer.
Parameters:
  - char temp_digits[]:
    This is the pointer that points to the array where the ASCII
    equivalent will be placed.
  - uint16_t lm73_temp:
    This is the temperature reading that is being passed to function
    as a value
  - uint8_t f_not_c:
      This is the flag that determines if the conversion is sent out
      as Farenheit or Celcius. 0x00=Celcius and 0x01=Farenheit
Return: VOID 
***********************************************************/
void lm73_temp_convert(char temp_digits[], uint16_t lm73_temp, uint8_t f_not_c)
{
  //clear the character array
  temp_digits[0]=0;//set initial value to zero to denote empty array

  //clean the register value
  uint16_t cleaned_temp   = (lm73_temp>>5);//copy RS value by 5 to get rid of 0:4

  //setup variables to store additions
  uint8_t integer_part    = 0;//whole number
  uint8_t fractional_part = 0;//decimal part

  //setup arrays to hold string equivalent of IP(integer part) and FP(fractional part)
  char ip_arr[16];
  char fp_arr[16];

  //integer used to keep track of farenheit decimal
  uint8_t farenheit_carry = 0;

  //check each bit to get temperature
  for (uint8_t bit_t = 0; bit_t <11; bit_t++) {
    if (((cleaned_temp>>bit_t)&1)==1) {//bit is a one
      if (bit_t == 0) {//0.25 degrees
        fractional_part+=25;
      }else if (bit_t==1) {//0.50 degrees
        fractional_part+=50;
      }else if (bit_t==10) {//sign of temperature
        strcat(temp_digits, "-");
      }else{
        integer_part+=(1<<(bit_t-2));//bits 2:9(-2 due to 1st two bits)
      }
    }
  }
  //Farenheit conversion
  if (f_not_c==1) {
    //Convert decimal part 1st exclusively
    uint8_t farenheit_decimal = (fractional_part*9)/5;
    if (farenheit_decimal >=100) {
      farenheit_carry++;
      fractional_part = farenheit_decimal-100;
    }else{
      fractional_part = farenheit_decimal;
    }
    // clear_display();//for testing
    // cursor_home();//for testing
    // lcd_int16(integer_part,16,0,0);//for testing
    // _delay_ms(900);//for testing
    // clear_display();//for testing
    // cursor_home();//for testing
    //Convert integer part
    uint16_t farenheit_integer  = (integer_part*90)/5 + 320;
    uint8_t f_decimal_from_int = (farenheit_integer%10) * 10;
    fractional_part +=f_decimal_from_int;//update fraction part
    if (fractional_part>=100) {
      farenheit_carry++;//increase the farenheit carry
      fractional_part-=100;//update fractional part
    }
    integer_part =((integer_part*9)/5 +32)+farenheit_carry;//update int part

  }

  //write value to char array
  itoa(integer_part,ip_arr,10);//convert integer part to ascii
  itoa(fractional_part,fp_arr,10);//convert fractionla part to ascii
  strcat(temp_digits, ip_arr);//concatenate with the main array
  strcat(temp_digits, ".");//add decimal point
  strcat(temp_digits, fp_arr);//concatenate with main array
  if (f_not_c) {
    strcat(temp_digits,"F");//Add DEG F
  }else{
    strcat(temp_digits, "C");//ADD DEG C
  }


}//lm73_temp_convert

/*********************************************************
Function: read_temperature()
Description:
  This function will tell the LM73 Temperature sensor
  to take a reading. This will be returned in two bytes.
  The sensor is configured in the defualt mode:
  11-Bit (10-Bit Plus Sign) mode.
Parameters:
  NONE
Return:
  -uint16_t lm73_temperature:
    This variable holds the temperature value that was read
    from the LM73 temperature sensor
***********************************************************/
uint16_t read_temperature(){
  uint16_t lm73_temperature;//var to hold retun value
  twi_start_rd(LM73_ADDRESS, lm73_rd_buf, 2);
  lm73_temperature = lm73_rd_buf[0];//
  lm73_temperature = lm73_temperature<<8;//LS by 8
  lm73_temperature |= lm73_rd_buf[1];

  return lm73_temperature;
}
