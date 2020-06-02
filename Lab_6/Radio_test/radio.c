//Victor Garcia Flores

#define F_CPU 16000000 // cpu speed in hertz
#define TRUE 1
#define FALSE 0
#include <avr/io.h>
#include "uart_functions.h"
#include "twi_master.h"
#include "si4734.h"
#include <math.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>

char    lcd_string_array[16];  //holds a string to refresh the LCD
char     lcd_string_C[16];  //holds string to send to lcd
char     lcd_string_F[16];  //holds string to send to lcduint8_t i;
uint8_t i;                     //general purpose index
char     lcd_output[32];  //holds output string

volatile uint16_t  current_fm_freq =  9990; //0x2706, arg2, arg3; 99.9Mhz, 200khz steps
extern uint8_t  si4734_wr_buf[9];
extern uint8_t  si4734_rd_buf[9];
extern uint8_t  si4734_tune_status_buf[8];
extern volatile uint8_t STC_interrupt;     //indicates tune or seek is done

ISR(INT7_vect){STC_interrupt = TRUE;}

void tcnt7_init(void){
  EIMSK  |= (1<INT7); //ext osc TOSC
  EICRB |= (1<<ISC71)|(1<<ISC70);
}

/***********************************************************************/
/*                                main                                 */
/***********************************************************************/
int main ()
{
  init_twi();

  tcnt7_init();
  // DDRB = 0xFF;
  // PORTB = 0x00;
  sei();

  // PORTB = (1<<PB1);
  // _delay_ms(500);
  // PORTB = (0<<PB1);
  // _delay_ms(500);

  DDRE  |= 0x04; //Port E bit 2 is active high reset for radio
  DDRE  |= 0x40; //Port E bit 6 is shift/load_n for encoder 74HC165
  DDRE  |= 0x08; //Port E bit 3 is TCNT3 PWM output for volume
  PORTE |= 0x04; //radio reset is on at powerup (active high)
  PORTE |= 0x40; //pulse low to load switch values, else its in shift mode

  //hardware reset of Si4734
  PORTE &= ~(1<<PE7); //int2 initially low to sense TWI mode
  DDRE  |= 0x80;      //turn on Port E bit 7 to drive it low
  PORTE |=  (1<<PE2); //hardware reset Si4734
  _delay_us(200);     //hold for 200us, 100us by spec
  PORTE &= ~(1<<PE2); //release reset
  _delay_us(30);      //5us required because of my slow I2C translators I suspect
                      //Si code in "low" has 30us delay...no explaination given
  DDRE  &= ~(0x80);   //now Port E bit 7 becomes input from the radio interrupt

  for(int i = 0; i < 5; i++){
  fm_pwr_up(); //powerup the radio as appropriate
  }
  //while(twi_busy()){} //spin while TWI is busy
  fm_tune_freq();     //tune to frequency

  while(1){          //main while loop

  } //while
} //main
