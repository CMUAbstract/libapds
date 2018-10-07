#ifndef PIN_ASSIGN_H
#define PIN_ASSIGN_H

// Ugly workaround to make the pretty GPIO macro work for OUT register
// (a control bit for TAxCCTLx uses the name 'OUT')
#undef OUT

#define BIT_INNER(idx) BIT ## idx
#define BIT(idx) BIT_INNER(idx)

#define GPIO_INNER(port, reg) P ## port ## reg
#define GPIO(port, reg) GPIO_INNER(port, reg)


#define PORT_DEBUG							3
#define PIN_DEBUG_1							4
#define PIN_DEBUG_2							5
#define PIN_DEBUG_3							6
#define PIN_DEBUG               6

#define PORT_LOAD 3
#define PIN_LOAD  4

#define PORT_CAPYBARA_CFG 3
#define PIN_CAPYBARA_CFG  5

#if BOARD_MAJOR == 1 && BOARD_MINOR == 0
#define PORT_SENSE_SW 3
#define PIN_SENSE_SW  7


#define PORT_RADIO_SW 3
#define PIN_RADIO_SW  2

#elif BOARD_MAJOR == 1 && BOARD_MINOR == 1
// TODO: I'm pretty sure this will break if we include pins.h and
// capy_board_init.h on v1.1, but that's uncofirmed at the moment

#define PORT_PHOTO_SENSE 2
#define PIN_PHOTO_SENSE 3 // GPIO extender pins
#define BIT_CCS_WAKE  (1 << 2)
#define BIT_SENSE_SW  (1 << 3)
#define BIT_PHOTO_SW  (1 << 4)
#define BIT_APDS_SW   (1 << 5)
#define BIT_RADIO_RST (1 << 6)
#define BIT_RADIO_SW  (1 << 7)

#elif BOARD_MAJOR == 2
#define BIT_HMC_DRDY (1 << 0)
#define BIT_LSM_INT1 (1 << 1)
#define BIT_LSM_INT2 (1 << 2)
#define BIT_LPS_INT  (1 << 3)
#define BIT_APDS_SW  (1 << 4)
#define BIT_APDS_INT (1 << 5)
#define BIT_PHOTO_SW (1 << 6)
#define BIT_SENSE_SW (1 << 7)
#define PORT_PHOTO_SENSE 3
#define PIN_PHOTO_SENSE 0


#endif // BOARD.{MAJOR,MINOR}


#endif

