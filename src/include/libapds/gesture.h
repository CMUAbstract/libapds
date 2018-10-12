#ifndef _GESTURE_H_
#define _GESTURE_H_

#include "proximity.h"

#define WAIT_PHOTORESISTOR_DELAY __delay_cycles(1000)
#define WAIT_APDS_DELAY __delay_cycles(160000)

// Board specific function to turn on the apds
// Note: for capybara v2 this enables the entire sensor rail
void apds_init(void);

// Board specific function to turn off the apds
// Note: for capybara v2 this disables the entire rail and resets the fxl6408
void apds_disable(void);

// Function for capturing gesture direction once apds is already initialized
gest_dir apds_get_gesture(void);
void apds_get_raw_gesture(gesture_data_t *gesture_data_, uint8_t *num_samps);

// Board specific function to clear out the apds registers after running a gesture
// Note: primarily necessary for testing under continuous power
void apds_settle(void);

#endif
