#include <stdint.h>
#include <libio/console.h>
#include "color.h"
#include "proximity.h"



void apds_color_init() {
  restartTransmitAPDS();
	writeSingleByte(APDS9960_ID);
  uint8_t sensorID;
	sensorID = readDataByte();
  if(sensorID != 0xAB) {
    PRINTF("error initializing APDS! id = %x\r\n",sensorID);
    while(sensorID != 0xAB) {
      restartTransmitAPDS();
      writeSingleByte(APDS9960_ID);
      sensorID = readDataByte();
    }
  }
	restartTransmitAPDS();
  // Turn off apds while we're setting it up
  writeDataByte(APDS9960_ENABLE, 0);
  // Set integration time
	writeDataByte(APDS9960_ATIME, DEFAULT_ATIME);
  // Set adc gain
	writeDataByte(APDS9960_CONTROL,DEFAULT_AGAIN);
  // Turn power back on and enable color while we're at i
  // PON = 0b1, AEN = 0b10
  writeDataByte(APDS9960_ENABLE, 3);
}


void apds_read_color(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c) {
  uint8_t temp_l, temp_h;
  uint8_t status;

  restartTransmitAPDS();
  writeSingleByte(APDS9960_STATUS);
  status = readDataByte();
  while(!(status & COLOR_MASK)) {
    __delay_cycles(500);
    restartTransmitAPDS();
    writeSingleByte(APDS9960_STATUS);
    status = readDataByte();
  }

  restartTransmitAPDS();
  writeSingleByte(APDS9960_RDATAL);
  temp_l = readDataByte();

  restartTransmitAPDS();
  writeSingleByte(APDS9960_RDATAH);
  temp_h = readDataByte();

  *r = (temp_h << 8) + temp_l;

  restartTransmitAPDS();
  writeSingleByte(APDS9960_GDATAL);
  temp_l = readDataByte();

  restartTransmitAPDS();
  writeSingleByte(APDS9960_GDATAH);
  temp_h = readDataByte();

  *g = (temp_h << 8) + temp_l;

  restartTransmitAPDS();
  writeSingleByte(APDS9960_BDATAL);
  temp_l = readDataByte();

  restartTransmitAPDS();
  writeSingleByte(APDS9960_BDATAH);
  temp_h = readDataByte();

  *b = (temp_h << 8) + temp_l;

  restartTransmitAPDS();
  writeSingleByte(APDS9960_CDATAL);
  temp_l = readDataByte();

  restartTransmitAPDS();
  writeSingleByte(APDS9960_CDATAH);
  temp_h = readDataByte();

  *c = (temp_h << 8) + temp_l;

return;
}


