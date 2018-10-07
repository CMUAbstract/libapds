#include <stdint.h>
#include <libfxl/fxl6408.h>
#include <libcapybara/board.h>
#include "gesture.h"
#include "proximity.h"

#if BOARD_MAJOR != 2
#error Functions undefined for board
#endif

void apds_init() {
  fxl_set(BIT_SENSE_SW);
  __delay_cycles(5000);
  fxl_set(BIT_APDS_SW);
  __delay_cycles(5000);
  proximity_init();
  enableGesture();
  return;
}

void apds_disable() {
  disableGesture();
#if (BOARD_MAJOR == 2 && BOARD_MINOR == 0)
  fxl_clear(BIT_APDS_SW);
  fxl_clear(BIT_SENSE_SW);
#else
#error apds_disable undefined for board
#endif
  fxl_reset();
  return;
}

gest_dir apds_get_gesture() {
  gest_dir gest_out;
  uint8_t num_samps = 0;
  gesture_data_t gesture_data_;
  reenableGesture();
  resetGestureFields(&gesture_data_);
  getGestureLoop(&gesture_data_, &num_samps);
  if(num_samps > MIN_DATA_SETS) {
    gest_out = decodeGesture();
  }
  else { 
    gest_out = DIR_NONE;
  }
  apds_disable();
  return gest_out;
}

void apds_settle() {
#if (BOARD_MAJOR == 2 && BOARD_MINOR == 0)
  fxl_set(BIT_SENSE_SW);
  __delay_cycles(20000);
  disableGesture();
  fxl_set(BIT_APDS_SW);
  __delay_cycles(20000);
  fxl_clear(BIT_APDS_SW);
#else
#error apds_settle undefined for board
#endif
  return;
}


