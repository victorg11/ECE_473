// Victor Garcia Flores

#include <avr/io.h>
#include <avr/interrupt.h>
#include "hd44780.h"
#include <stdlib.h>
#include <util/delay.h>
#include "twi_master.h"
#include "lm73_functions.h"


//Bit Macros
#define BIT0 0
#define BIT1 1
#define BIT2 2
#define BIT3 3
#define BIT4 4
#define BIT5 5
#define BIT6 6
#define BIT7 7

char     lcd_str_h[16];  //holds string to send to lcd
char     lcd_str_l[16];  //holds string to send to lcd
div_t    fp_temp_result, fp_low_result;

uint16_t temp_reading = 11;//dummmy number

// extern uint8_t lm73_wr_buf[2];
// extern uint8_t lm73_rd_buf[2];

//***********************************************************************
//                            spi_init
//**********************************************************************
void spi_init(){
  //PORTB ouput: ss(pb0), MOSI(pb2), sclk(pb1)
  DDRB  |= (1<<BIT0)|(1<<BIT1)|(1<<BIT2);
  SPCR   = (1<<SPE) | (1<<MSTR) ; //master mode, clk low on idle, leading edge sample
  SPSR   = (1<<SPI2X); //choose double speed operation
 }

 /***************************************************************
 Function: lcd_ports()
 Description:
   This function will set the correct bits used for the LCD
   display to work correctly
 Parameters: NONE
 Return:void
 ***************************************************************/
 void lcd_ports(){
   DDRF  |= 0x08;  //port F bit 3 is enable for LCD
   PORTF &= 0xF7;  //port F bit 3 is initially low
 }


int main()
{
  uint16_t lm73_temp;
  spi_init();//initialize SPI protocol
  lcd_ports();//initialize LCD ports
  lcd_init();//initialize LCD protocols
  init_twi();//initialize twi

  sei();

  // lm73_wr_buf[0] = LM73_PTR_TEMP;
  // twi_start_wr(LM73_ADDRESS,lm73_wr_buf,1);
  // _delay_ms(2);//wait for xfer to finish
  clear_display();


  while (1) {
    _delay_ms(100);
    set_cursor(2,6);
    lm73_temp = read_temperature();
    //lm73_temp = 0x860;//(-16.75C)for testing
    lm73_temp_convert(lcd_str_h,lm73_temp,0);//Farenheit EN
    string2lcd(lcd_str_h);

  }

  return 0;
}
