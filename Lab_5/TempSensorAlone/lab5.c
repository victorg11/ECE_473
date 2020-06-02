#define F_CPU 16000000 // cpu speed in hertz
#define TRUE 1
#define FALSE 0
#include <avr/io.h>
#include "uart_functions.h"
#include "lm73_functions.h"
#include "twi_master.h"
#include <math.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>

char    lcd_string_array[16];  //holds a string to refresh the LCD
char     lcd_string_C[16];  //holds string to send to lcd
char     lcd_string_F[16];  //holds string to send to lcduint8_t i;
uint8_t i;                     //general purpose index
char     lcd_output[32];  //holds output string

extern uint8_t lm73_wr_buf[2];
extern uint8_t lm73_rd_buf[2];

//********************************************************************
//                            spi_init
//Initalizes the SPI port on the mega128. Does not do any further
// external device specific initalizations.
//********************************************************************
void spi_init(void){
  DDRB |=  0x07;  //Turn on SS, MOSI, SCLK
  //mstr mode, sck=clk/2, cycle 1/2 phase, low polarity, MSB 1st,
  //no interrupts, enable SPI, clk low initially, rising edge sample
  SPCR=(1<<SPE) | (1<<MSTR);
  SPSR=(1<<SPI2X); //SPI at 2x speed (8 MHz)
}//spi_init

void temptrPlacer(){
  //handle Farenheit display
  for(int j=16;j<22;j++){
    lcd_output[j] = lcd_string_F[j-16];
  }
  //handle Celcius display
  for(int k=23;k<29;k++){
    lcd_output[k] = lcd_string_C[k-23];
  }
}

/***********************************************************************/
/*                                main                                 */
/***********************************************************************/
int main ()
{
  uint16_t lm73_temp;  //a place to assemble the temperature from the lm73

  spi_init();//initalize SPI
  lcd_init(); //initalize LCD (lcd_functions.h)
  init_twi();//initalize TWI (twi_master.h)

  sei();//enable interrupts before entering loop

  //set LM73 mode for reading temperature by loading pointer register
  lm73_wr_buf[0] = 0;//load lm73_wr_buf[0] with temperature pointer address
  twi_start_wr(LM73_ADDRESS,lm73_wr_buf,1);//start the TWI write process
  //^the address value for "LM73_ADDRESS" is in lm73_functions.h
  _delay_ms(2); //wait for the xfer to finish

  clear_display(); //clean up the display
  int index1=0;
  int F_complegeFlag=0;
  int C_complegeFlag=0;
  while(1){          //main while loop
    _delay_ms(1000); //tenth second wait
    clear_display();                  //wipe the display
    lm73_temp = get_rawData();
    temptrPlacer();
    //call function that perform the rest of the operations
    // lm73_temp_convert(lcd_string_array,lm73_temp,1);
    // string2lcd(lcd_string_array);
    //Display in Farenheit
    lm73_temp_convert(lcd_string_F,lm73_temp,1);

    if(index1==6){
      index1 = 0;
    }
    set_cursor(2,index1);
    char2lcd(lcd_string_F[index1]);
    // string2lcd(lcd_string_F);

    index1++;
    //Display in Celcius
    lm73_temp_convert(lcd_string_C,lm73_temp,0);
    //set_cursor(2,7);
    // string2lcd(lcd_string_C);
    // refresh_lcd(lcd_output);

  } //while
} //main
