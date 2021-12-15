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

/*
 *@brief consecutively writes register address and value over i2c
 *@details analogous to wirewritedatabyte in wire.h
 */
static void writeByte(uint8_t reg, uint8_t val){
  EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);
  EUSCI_B_I2C_masterSendStart(EUSCI_B0_BASE);
  EUSCI_B_I2C_masterSendMultiByteNext(EUSCI_B0_BASE, reg);
  EUSCI_B_I2C_masterSendMultiByteNext(EUSCI_B0_BASE, val);
  EUSCI_B_I2C_masterSendMultiByteStop(EUSCI_B0_BASE);
  while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));
	return;
}
/*
 *@brief writes a single value over i2c with appropriate waiting etc
 */
static void writeSingle(uint8_t val){
  EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);
  EUSCI_B_I2C_masterSendSingleByte(EUSCI_B0_BASE, val);
  while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));
	return;
}

/*
 *@brief reads a byte over i2c
 */
static uint8_t readByte(){
	uint8_t val;
	EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_RECEIVE_MODE);
  EUSCI_B_I2C_masterReceiveStart(EUSCI_B0_BASE);
  val = EUSCI_B_I2C_masterReceiveSingle(EUSCI_B0_BASE);
  EUSCI_B_I2C_masterReceiveMultiByteStop(EUSCI_B0_BASE);
  while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));
	return val;
}

/*
 *@brief handles flip from transmit to receive
 */
static void restartTransmit(){
	EUSCI_B_I2C_disable(EUSCI_B0_BASE);
  EUSCI_B_I2C_setSlaveAddress(EUSCI_B0_BASE, APDS9960_I2C_ADDR);
  EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);
  EUSCI_B_I2C_enable(EUSCI_B0_BASE);
//  EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);

	while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));
}

void proximity_init(void) {
	uint8_t sensorID = 0;
	/*Transmit address and read back returned value*/
	restartTransmit();
	writeSingle(APDS9960_ID);
	sensorID = readByte();
  if(sensorID != 0xAB) {
    PRINTF("error initializing APDS! id = %x\r\n",sensorID);
    while(sensorID != 0xAB) {
      restartTransmit();
      writeSingle(APDS9960_ID);
      sensorID = readByte();
    }
  }
	restartTransmit();
  writeByte(APDS9960_ENABLE, 0);
	/*Run through and set a bundle of defaults*/
	writeByte(APDS9960_ATIME, DEFAULT_ATIME);
	writeByte(APDS9960_WTIME, DEFAULT_WTIME);
	writeByte(APDS9960_PPULSE, DEFAULT_PROX_PPULSE);
	writeByte(APDS9960_POFFSET_UR, DEFAULT_POFFSET_UR);
	writeByte(APDS9960_POFFSET_DL, DEFAULT_POFFSET_DL);
	writeByte(APDS9960_CONFIG1, DEFAULT_CONFIG1);
#if 1
	restartTransmit();
	writeSingle(APDS9960_CONTROL);
	uint8_t val = readByte();
	/*Set led drive*/
	uint8_t drive = DEFAULT_LDRIVE;
	drive = drive << 6;
	val &= 0x3F;
	val |= drive;
	/*Set proximity gain*/
	drive = DEFAULT_PGAIN;
	drive &= 0x3;
	drive = drive << 2;
	val &= 0xF3;
	val |= drive;
	/*Set ambient light gain*/
	drive = DEFAULT_AGAIN;
	drive &= 0x3;
	val &= 0xFC;
	val |= drive;
	/*Write all changes*/
	restartTransmit();
	writeByte(APDS9960_CONTROL,val);
	writeByte(APDS9960_PILT, DEFAULT_PILT);
	writeByte(APDS9960_PIHT, DEFAULT_PIHT);
	/*Set low thresh for ambient light interrupts*/
	uint16_t thresh = DEFAULT_AILT;
	uint8_t lowByte = thresh & 0x00FF;
	uint8_t highByte = (thresh & 0xFF00) >> 8;
	writeByte(APDS9960_AILTL, lowByte);
	writeByte(APDS9960_AILTH, highByte);
	/*Set high threshold for ambient light interrupts*/
	thresh = DEFAULT_AIHT;
	lowByte = thresh & 0x00FF;
	highByte = (thresh & 0xFF00 >> 8);
	writeByte(APDS9960_AIHTL, lowByte);
	writeByte(APDS9960_AIHTH, highByte);
	writeByte(APDS9960_PERS, DEFAULT_PERS);
	writeByte(APDS9960_CONFIG2, DEFAULT_CONFIG2);
#endif
	writeByte(APDS9960_CONFIG3, DEFAULT_CONFIG3);
  // Use this for high power, set if 1 on line 81 to 0
#if 0
  restartTransmit();
  writeByte(APDS9960_CONTROL,0b11001100);
  restartTransmit();
  writeByte(APDS9960_ENABLE, 0x5);
#endif
#if 0
	 /* switching to the gesture stuff*/
	writeByte(APDS9960_GPENTH, DEFAULT_GPENTH);
	writeByte(APDS9960_GEXTH, DEFAULT_GEXTH);
	writeByte(APDS9960_GCONF1, DEFAULT_GCONF1);
	/*Tell device which reg to return, read vals,  modify a couple bits, and write it back*/
	/*This sequence sets gain for gesture engine*/
	restartTransmit();
	writeSingle(APDS9960_GCONF2);
	val = readByte();
	//LOG("start val = %x \r\n",val);
	uint8_t gain = DEFAULT_GGAIN;
	gain &= 0x03;
	gain = gain << 5;
	val &= 0x9F;
	val |= gain;
	drive = DEFAULT_GLDRIVE;
	drive &= 0x03;
	drive = drive << 3;
	val &= 0xD7;
	val |= drive;
	uint8_t time = DEFAULT_GWTIME;
	time &= 0x07;
	val &= 0xF8;
	val |= time;
	//LOG("final val = %x \r\n",val);
	restartTransmit();
	writeByte(APDS9960_GCONF2, val);
	/*Now write a bunch of values in the usual way...*/
	writeByte(APDS9960_GCONF2, val);
	writeByte(APDS9960_GOFFSET_U, DEFAULT_GOFFSET);
	writeByte(APDS9960_GOFFSET_D, DEFAULT_GOFFSET);
	writeByte(APDS9960_GOFFSET_L, DEFAULT_GOFFSET);
	writeByte(APDS9960_GOFFSET_R, DEFAULT_GOFFSET);
	writeByte(APDS9960_GPULSE, DEFAULT_GPULSE);
	writeByte(APDS9960_GCONF3, DEFAULT_GCONF3);
	/*Sanity check on GCONF2*/
	restartTransmit();
	writeSingle(APDS9960_GCONF2);
	//uint8_t test = readByte();
	//LOG("Gconf = %x \r\n",test);
	/*enable gesture interrupt with default val*/
	restartTransmit();
	writeSingle(APDS9960_GCONF4);
	val = readByte();
	uint8_t enable = DEFAULT_GIEN;
	enable &= 0x01;
	enable = enable << 1;
	val &= 0xFD;
	val |= enable;
	restartTransmit();
	writeByte(APDS9960_GCONF4, val);


  LOG2("Proximity sensor set up. ID:  %x\r\n", proximityID);
	//	enableProximitySensor();
	restartTransmit();
	writeSingle(APDS9960_ID);
	val = readByte();
	//loG("Val after prox enable = %x \r\n", val);
#endif
	return;
}

void proximity_init_slim(void) {
	uint8_t sensorID = 0;
	restartTransmit();
	writeSingle(APDS9960_ID);
	sensorID = readByte();
  if(sensorID != 0xAB) {
    PRINTF("error initializing APDS! id = %x\r\n",sensorID);
    while(sensorID != 0xAB) {
      restartTransmit();
      writeSingle(APDS9960_ID);
      sensorID = readByte();
    }
  }
	restartTransmit();
  writeByte(APDS9960_ENABLE, 0);
  restartTransmit();
  writeByte(APDS9960_CONTROL,0b11001100);
  restartTransmit();
  writeByte(APDS9960_ENABLE, 0x5);
  return;
}

void enableProximitySensor(void){
	/*Set proximity gain*/
	restartTransmit();
	writeSingle(APDS9960_CONTROL);
	uint8_t val = readByte();
	uint8_t drive = DEFAULT_PGAIN;
	drive &= 0x3;
	drive = drive << 2;
	val &= 0xF3;
	val |=drive;
	restartTransmit();
	writeByte(APDS9960_CONTROL, val);
	/*Set LED drive*/
	/*Set led drive*/
	drive = DEFAULT_LDRIVE;
	drive = drive << 6;
	val &= 0x3F;
	val |= drive;
	restartTransmit();
	writeByte(APDS9960_CONTROL, val);
	/*Disable interrupt*/
	restartTransmit();
	writeSingle(APDS9960_ENABLE);
	val = readByte();
	val &= 0xDF;
	restartTransmit();
	writeByte(APDS9960_CONTROL, val);
	/*Enable power*/
	restartTransmit();
	writeSingle(APDS9960_ENABLE);
	val = readByte();
  val |= (1 << POWER);
#ifdef NO_PERIPHS
  /*Disable the power if we're turning off the peripherals*/
  val &= ~(1 << POWER);
#endif
  val |= (1 << PROXIMITY);
	restartTransmit();
	writeByte(APDS9960_ENABLE,val);
	return;
}

#if 0
float proximity_read() {
	uint8_t val = 0;
	restartTransmit();
	writeSingle(APDS9960_PDATA);
	val = readByte();
  float ret_val = val;
	return ret_val;
}
#endif

uint8_t proximity_read_byte() {
	uint8_t val = 0;
	restartTransmit();
	writeSingle(APDS9960_PDATA);
	val = readByte();
	return val;
}

void apds_proximity_reenable() {
  restartTransmit();
  writeByte(APDS9960_ENABLE, 0x5);
}

void apds_proximity_disable() {
  restartTransmit();
  writeByte(APDS9960_ENABLE, 0x1);
}

