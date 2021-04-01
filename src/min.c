#include "libmspware/driverlib.h"
#include <libmspware/gpio.h>
#include <libmsp/mem.h>
#include <libmsp/periph.h>
#include <libio/console.h>
#include <stdint.h>
#include <stdio.h>
#include "proximity.h"
#include "pins.h"
#include <libfxl/fxl6408.h>
#include <libpacarana/pacarana.h>

extern volatile __nv int photo_status;
/*
 *@brief Sets up pin 3.0 to read from a photoresistor
 *@details Written based on egauge code found at github.com/CMUAbstract/egauge
 */
void enable_photoresistor(void){
#if BOARD_MAJOR == 1 && BOARD_MINOR ==0
  // Set pin 3.0 to GPIO OUT,HIGH
  P3OUT |= BIT0;
  P3DIR |= BIT0;
  // Set pin 3.1 to ADC comparator mode
  P3SEL0 |= BIT1;
  P3SEL1 |= BIT1;
#elif BOARD_MAJOR == 1 && BOARD_MINOR == 1
  // Set GPIO HIGH to power photoresistor
  fxl_set(BIT_PHOTO_SW);
  // Config the ADC on the comparator pin
  GPIO(PORT_PHOTO_SENSE,SEL0) |= BIT(PIN_PHOTO_SENSE);
  GPIO(PORT_PHOTO_SENSE,SEL1) |= BIT(PIN_PHOTO_SENSE);
#elif BOARD_MAJOR == 2
  // Set GPIO HIGH to power photoresistor
  fxl_set(BIT_PHOTO_SW);
  // Config the ADC on the comparator pin
  GPIO(PORT_PHOTO_SENSE,SEL0) |= BIT(PIN_PHOTO_SENSE);
  GPIO(PORT_PHOTO_SENSE,SEL1) |= BIT(PIN_PHOTO_SENSE);
  __delay_cycles(1000);                   // Delay for Ref to settle
  STATE_CHANGE(photo,1);
#else
#error Unsupported photoresistor config
#endif// BOARD_{MAJOR,MINOR}
}

void disable_photoresistor(void){
#if BOARD_MAJOR == 1 && BOARD_MINOR ==0
  P3OUT &= ~BIT0;
#elif BOARD_MAJOR == 1 && BOARD_MINOR == 1
  fxl_clear(BIT_PHOTO_SW);
#elif BOARD_MAJOR == 2
  fxl_clear(BIT_PHOTO_SW);
#else
#error Unsupported photoresistor config
#endif// BOARD_{MAJOR,MINOR}
  STATE_CHANGE(photo,0);
}
/*
void photores_setup(void){
  P3SEL0 |= BIT0;
  P3SEL1 |= BIT1;
  PM5CTL0 &= ~LOCKLPM5;
}
*/
/*
 *@brief starts reading from the photoresistor ADC comparator pins
 *
 */
float read_photoresistor_fl(void){
   ADC12CTL0 &= ~ADC12ENC;           // Disable conversions

  ADC12CTL1 = ADC12SHP;
#if BOARD_MAJOR == 1 && BOARD_MINOR ==0
  ADC12MCTL0 = ADC12VRSEL_1 | ADC12INCH_13;
#elif BOARD_MAJOR == 1 && BOARD_MINOR == 1
  ADC12MCTL0 = ADC12VRSEL_1 | ADC12INCH_6;
#elif BOARD_MAJOR == 2
  ADC12MCTL0 = ADC12VRSEL_1 | ADC12INCH_12;
#else
#error Unsupported photoresistor config
#endif// BOARD_{MAJOR,MINOR}
  ADC12CTL0 |= ADC12SHT03 | ADC12ON;

  while( REFCTL0 & REFGENBUSY );

  REFCTL0 = REFVSEL_2 | REFON;            //Set reference voltage to 2.5

  __delay_cycles(1000);                   // Delay for Ref to settle

  ADC12CTL0 |= ADC12ENC;                  // Enable conversions
  ADC12CTL0 |= ADC12SC;                   // Start conversion
  while (ADC12CTL1 & ADC12BUSY) ;

  signed long output = (signed long) ADC12MEM0;

  ADC12CTL0 &= ~ADC12ENC;                 // Disable conversions
  ADC12CTL0 &= ~(ADC12ON);                // Shutdown ADC12
  REFCTL0 &= ~REFON;
  float new_ret = output;
  return new_ret;
}

int16_t read_photoresistor(void){
   ADC12CTL0 &= ~ADC12ENC;           // Disable conversions

  ADC12CTL1 = ADC12SHP;
#if BOARD_MAJOR == 1 && BOARD_MINOR ==0
  ADC12MCTL0 = ADC12VRSEL_1 | ADC12INCH_13;
#elif BOARD_MAJOR == 1 && BOARD_MINOR == 1
  ADC12MCTL0 = ADC12VRSEL_1 | ADC12INCH_6;
#elif BOARD_MAJOR == 2
  ADC12MCTL0 = ADC12VRSEL_1 | ADC12INCH_12;
#else
#error Unsupported photoresistor config
#endif// BOARD_{MAJOR,MINOR}
  ADC12CTL0 |= ADC12SHT03 | ADC12ON;

  while( REFCTL0 & REFGENBUSY );

  REFCTL0 = REFVSEL_2 | REFON;            //Set reference voltage to 2.5

  __delay_cycles(1000);                   // Delay for Ref to settle

  ADC12CTL0 |= ADC12ENC;                  // Enable conversions
  ADC12CTL0 |= ADC12SC;                   // Start conversion
  while (ADC12CTL1 & ADC12BUSY) ;

  signed long output = (signed long) ADC12MEM0;

  ADC12CTL0 &= ~ADC12ENC;                 // Disable conversions
  ADC12CTL0 &= ~(ADC12ON);                // Shutdown ADC12
  REFCTL0 &= ~REFON;
  float new_ret = output;
  return new_ret;
}
