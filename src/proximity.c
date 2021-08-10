#include "libmspware/driverlib.h"
#include <libmsp/mem.h>
#include <libmsp/periph.h>
#include <libio/console.h>
#include <stdint.h>
#include <stdio.h>
#include "proximity.h"
#include "pins.h"

#define DEBUG 0
#if (BOARD_MAJOR == 1 && BOARD_MINOR == 1) \
  || (BOARD_MAJOR == 2 && BOARD_MINOR == 0)
#include <libfxl/fxl6408.h>
#endif //BOARD.{MAJOR,MINOR}

void delay(uint32_t cycles)
{
    unsigned i;
    for (i = 0; i < cycles / (1U << 15); ++i)
        __delay_cycles(1U << 15);
}


static int16_t abs16(int16_t input){
	int16_t output;
	if(input < 0){
		input = 0 - input;
	}
	output = (int16_t) input;
	return output;
}

static int16_t div(int16_t num, int16_t denom){
	int sign;
#if DEBUG
	LOG("NUM = %i DENOM = %i ", num, denom);
#endif
	sign = (num < 0 && denom < 0) || (num > 0 && denom > 0) ;
	if(!sign)
		sign = -1;
	num = abs16(num);
	denom = abs16(denom);
	int16_t quo = 0;
	while((num -= denom) > 0)
		quo++;
#if DEBUG
	LOG("Quo first = %i \r\n", quo);
#endif

	quo = quo * sign;
#if DEBUG
	LOG("Quo = %i \r\n", quo);
#endif
	return quo;
}

void proximity_init_ldrive(uint8_t drive) {
	uint8_t sensorID = 0;
	/*Transmit address and read back returned value*/
	restartTransmitAPDS();
	writeSingleByte(APDS9960_ID);
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
  writeDataByte(APDS9960_ENABLE, 0);
	/*Run through and set a bundle of defaults*/
	writeDataByte(APDS9960_ATIME, DEFAULT_ATIME);
	writeDataByte(APDS9960_WTIME, DEFAULT_WTIME);
	writeDataByte(APDS9960_PPULSE, DEFAULT_PROX_PPULSE);
	writeDataByte(APDS9960_POFFSET_UR, DEFAULT_POFFSET_UR);
	writeDataByte(APDS9960_POFFSET_DL, DEFAULT_POFFSET_DL);
	writeDataByte(APDS9960_CONFIG1, DEFAULT_CONFIG1);
	restartTransmitAPDS();
	writeSingleByte(APDS9960_CONTROL);
	uint8_t val = readDataByte();
	/*Set led drive*/
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
	restartTransmitAPDS();
	writeDataByte(APDS9960_CONTROL,val);
	writeDataByte(APDS9960_PILT, DEFAULT_PILT);
	writeDataByte(APDS9960_PIHT, DEFAULT_PIHT);
	/*Set low thresh for ambient light interrupts*/
	uint16_t thresh = DEFAULT_AILT;
	uint8_t lowByte = thresh & 0x00FF;
	uint8_t highByte = (thresh & 0xFF00) >> 8;
	writeDataByte(APDS9960_AILTL, lowByte);
	writeDataByte(APDS9960_AILTH, highByte);
	/*Set high threshold for ambient light interrupts*/
	thresh = DEFAULT_AIHT;
	lowByte = thresh & 0x00FF;
	highByte = (thresh & 0xFF00 >> 8);
	writeDataByte(APDS9960_AIHTL, lowByte);
	writeDataByte(APDS9960_AIHTH, highByte);
	writeDataByte(APDS9960_PERS, DEFAULT_PERS);
	writeDataByte(APDS9960_CONFIG2, DEFAULT_CONFIG2);
	writeDataByte(APDS9960_CONFIG3, DEFAULT_CONFIG3);


	 /* switching to the gesture stuff*/
	writeDataByte(APDS9960_GPENTH, DEFAULT_GPENTH);
	writeDataByte(APDS9960_GEXTH, DEFAULT_GEXTH);
	writeDataByte(APDS9960_GCONF1, DEFAULT_GCONF1);
	/*Tell device which reg to return, read vals,  modify a couple bits, and write it back*/
	/*This sequence sets gain for gesture engine*/
	restartTransmitAPDS();
	writeSingleByte(APDS9960_GCONF2);
	val = readDataByte();
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
	restartTransmitAPDS();
	writeDataByte(APDS9960_GCONF2, val);
	/*Now write a bunch of values in the usual way...*/
	writeDataByte(APDS9960_GCONF2, val);
	writeDataByte(APDS9960_GOFFSET_U, DEFAULT_GOFFSET);
	writeDataByte(APDS9960_GOFFSET_D, DEFAULT_GOFFSET);
	writeDataByte(APDS9960_GOFFSET_L, DEFAULT_GOFFSET);
	writeDataByte(APDS9960_GOFFSET_R, DEFAULT_GOFFSET);
	writeDataByte(APDS9960_GPULSE, DEFAULT_GPULSE);
	writeDataByte(APDS9960_GCONF3, DEFAULT_GCONF3);
	/*Sanity check on GCONF2*/
	restartTransmitAPDS();
	writeSingleByte(APDS9960_GCONF2);
	//uint8_t test = readDataByte();
	//LOG("Gconf = %x \r\n",test);
	/*enable gesture interrupt with default val*/
	restartTransmitAPDS();
	writeSingleByte(APDS9960_GCONF4);
	val = readDataByte();
	uint8_t enable = DEFAULT_GIEN;
	enable &= 0x01;
	enable = enable << 1;
	val &= 0xFD;
	val |= enable;
	restartTransmitAPDS();
	writeDataByte(APDS9960_GCONF4, val);


  LOG2("Proximity sensor set up. ID:  %x\r\n", proximityID);
	//	enableProximitySensor();
	restartTransmitAPDS();
	writeSingleByte(APDS9960_ID);
	val = readDataByte();
	//loG("Val after prox enable = %x \r\n", val);
	return;
}


void enableProximitySensor_ldrive(uint8_t ldrive){
	/*Set proximity gain*/
	restartTransmitAPDS();
	writeSingleByte(APDS9960_CONTROL);
	uint8_t val = readDataByte();
	uint8_t drive = DEFAULT_PGAIN;
	drive &= 0x3;
	drive = drive << 2;
	val &= 0xF3;
	val |=drive;
	restartTransmitAPDS();
	writeDataByte(APDS9960_CONTROL, val);
	/*Set LED drive*/
	restartTransmitAPDS();
	writeSingleByte(APDS9960_CONTROL);
	val = readDataByte();
	/*Set led drive*/
	drive = ldrive;
	drive = drive << 6;
	val &= 0x3F;
	val |= drive;
	restartTransmitAPDS();
	writeDataByte(APDS9960_CONTROL, val);
	/*Disable interrupt*/
	restartTransmitAPDS();
	writeSingleByte(APDS9960_ENABLE);
	val = readDataByte();
	val &= 0xDF;
	restartTransmitAPDS();
	writeDataByte(APDS9960_CONTROL, val);
	/*Enable power*/
	restartTransmitAPDS();
	writeSingleByte(APDS9960_ENABLE);
	val = readDataByte();
  val |= (1 << POWER);
#ifdef NO_PERIPHS
  /*Disable the power if we're turning off the peripherals*/
  val &= ~(1 << POWER);
#endif
	restartTransmitAPDS();
	writeDataByte(APDS9960_ENABLE,val);
	/*Set proximity mode*/
	restartTransmitAPDS();
  writeSingleByte(APDS9960_ENABLE);
	val = readDataByte();
	val |= (1 << PROXIMITY);
	restartTransmitAPDS();
	writeDataByte(APDS9960_ENABLE,val);
	return;
}


float proximity_read() {
	uint8_t val = 0;
	restartTransmitAPDS();
	writeSingleByte(APDS9960_PDATA);
	val = readDataByte();
  printf("Val is: %u\r\n",val);
	return (float)val;
}

	uint16_t raw_index_internal;
	__nv uint8_t raw_sample_array_internal[4][512];
	
void redirectRawGesture(uint8_t *** sample_array, uint16_t *sample_idx) {
	*sample_array = &raw_sample_array_internal;
	*sample_idx = raw_index_internal;	
	return;
}

int8_t  getGestureLoop(gesture_data_t *gesture_data_, uint8_t *num_samps){
	raw_index_internal = 0;
	/*Test if a gesture is available*/
//	restartTransmitAPDS();
  //LOG("Inside get gesture! \r\n");
  EUSCI_B_I2C_disable(EUSCI_B0_BASE);
	EUSCI_B_I2C_setSlaveAddress(EUSCI_B0_BASE, APDS9960_I2C_ADDR);
  while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));
	EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);
  EUSCI_B_I2C_enable(EUSCI_B0_BASE);
	writeSingleByte(APDS9960_ID);
	readDataByte();
	//LOG("ID Check = %x \r\n",check);
	restartTransmitAPDS();
	writeSingleByte(APDS9960_GSTATUS); //Check gstatus!
	uint8_t test = readDataByte();
	test &= APDS9960_GVALID;
	//This NEEDS to be here, TODO turn this into a delay instead
  //PRINTF("Gvalid  val= %x \r\n",test);
	restartTransmitAPDS();
	writeSingleByte(APDS9960_ENABLE);
	uint8_t enable = readDataByte();
	LOG("enable val = %x \r\n",enable);
	enable &= 0x41;
	test &= enable;
	if(!test){
		return DIR_NONE;
	}
	uint8_t loop_count = 0;
	/*while loop start*/
  uint16_t its = 0;
	while(1){
	//for(int loop_cnt = 1; loop_cnt < 100; loop_cnt++){
		delay(240000);
		uint8_t fifo_level = 0;
		restartTransmitAPDS();
		writeSingleByte(APDS9960_GSTATUS);
		uint8_t gstatus = readDataByte();
		LOG2("gstatus val 1 = %x \r\n",gstatus);
		if((gstatus & APDS9960_GVALID) == APDS9960_GVALID && its < 5){
      its++;
			restartTransmitAPDS();
			writeSingleByte(APDS9960_GFLVL);
			fifo_level = readDataByte();
			uint8_t fifo_data[MAX_DATA_SETS << 2], i;
			LOG("Fifo level = %u %u\r\n",fifo_level,its);
			if(fifo_level > 7){
				loop_count++;
				/*Read in all of the bytes from the fifo*/
				restartTransmitAPDS();
				writeSingleByte(APDS9960_GFIFO_U);
				EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_RECEIVE_MODE);
				EUSCI_B_I2C_masterReceiveStart(EUSCI_B0_BASE);

				/*Note: we multiply by 4 to get UP,DOWN,LEFT,RIGHT captured*/
				for(i = 0; i < fifo_level << 2; i++){
					fifo_data[i] =  EUSCI_B_I2C_masterReceiveSingle(EUSCI_B0_BASE);
				}
				EUSCI_B_I2C_masterReceiveMultiByteStop(EUSCI_B0_BASE);
				while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));
        //printf("Total fifo_level %u, multiplied: %u\r\n",
        //                                      fifo_level, fifo_level<< 2);
				for(i = 0; i < fifo_level << 2; i+=4){
          gesture_data_->u_data[gesture_data_->index] =  fifo_data[i + 0];
					gesture_data_->d_data[gesture_data_->index] =  fifo_data[i + 1];
					gesture_data_->l_data[gesture_data_->index] =  fifo_data[i + 2];
					gesture_data_->r_data[gesture_data_->index] =  fifo_data[i + 3];
					gesture_data_->index++;
					gesture_data_->total_gestures++;
					//printf("fifo iter: %u, index:%u, total:%u\r\n",
          //          i,gesture_data_->index,gesture_data_->total_gestures);
				}
			}
				/*LOG("ESCAPED!\r\n");*/
				restartTransmitAPDS();
				writeSingleByte(APDS9960_GCONF1);
				fifo_level = readDataByte();
				/*LOG("GCONF1 = %x \r\n", fifo_level);
				restartTransmitAPDS();
				writeSingleByte(APDS9960_GEXTH);
				fifo_level = readDataByte();
				LOG("GEXTH = %u \r\n", fifo_level);*/
				//TODO fix the reversed logic in processGestureData
        processGestureData(*gesture_data_);
				/*if(processGestureData(*gesture_data_) >= 0){
						//LOG("Decoding gesture! \r\n");
						decodeGesture();
					}*/
				//LOG("Guess = %u \r\n", gesture_motion_);
				gesture_data_->index = 0;
				gesture_data_->total_gestures = 0;
			}
			else{
				delay(FIFO_PAUSE);
				LOG2("ESCAPED ALL!\r\n");
				*num_samps = loop_count;
				return  gesture_motion_;
			}

		}
    PRINTF("Fell out\r\n");
}

int8_t  getGestureSingle(gesture_data_t *gesture_data_, uint8_t *num_samps){
	raw_index_internal = 0;
  EUSCI_B_I2C_disable(EUSCI_B0_BASE);
	EUSCI_B_I2C_setSlaveAddress(EUSCI_B0_BASE, APDS9960_I2C_ADDR);
  while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));
	EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);
  EUSCI_B_I2C_enable(EUSCI_B0_BASE);
	writeSingleByte(APDS9960_ID);
	readDataByte();
	//LOG("ID Check = %x \r\n",check);
	restartTransmitAPDS();
	writeSingleByte(APDS9960_GSTATUS); //Check gstatus!
	uint8_t test = readDataByte();
	test &= APDS9960_GVALID;
	restartTransmitAPDS();
	writeSingleByte(APDS9960_ENABLE);
	uint8_t enable = readDataByte();
	//This NEEDS to be here, TODO turn this into a delay instead
	LOG("enable val = %x \r\n",enable);
	enable &= 0x41;
	test &= enable;
	if(!test){
		return DIR_NONE;
	}
  uint8_t fifo_level = 0;
  restartTransmitAPDS();
  writeSingleByte(APDS9960_GSTATUS);
  uint8_t gstatus = readDataByte();
  LOG2("gstatus val 1 = %x \r\n",gstatus);
  restartTransmitAPDS();
  writeSingleByte(APDS9960_GFLVL);
  fifo_level = readDataByte();
  uint8_t fifo_data[MAX_DATA_SETS << 2], i;
  LOG2("Fifo level = %u \r\n",fifo_level);
  if(fifo_level > 1 ){
    /*Read in all of the bytes from the fifo*/
    restartTransmitAPDS();
    writeSingleByte(APDS9960_GFIFO_U);
    EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_RECEIVE_MODE);
    EUSCI_B_I2C_masterReceiveStart(EUSCI_B0_BASE);

    /*Note: we multiply by 4 to get UP,DOWN,LEFT,RIGHT captured*/
    for(i = 0; i < fifo_level << 2; i++){
      fifo_data[i] =  EUSCI_B_I2C_masterReceiveSingle(EUSCI_B0_BASE);
    }
    EUSCI_B_I2C_masterReceiveMultiByteStop(EUSCI_B0_BASE);
    while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));
    for(i = 0; i < fifo_level << 2; i+=4){
      gesture_data_->u_data[gesture_data_->index] =  fifo_data[i + 0];
      gesture_data_->d_data[gesture_data_->index] =  fifo_data[i + 1];
      gesture_data_->l_data[gesture_data_->index] =  fifo_data[i + 2];
      gesture_data_->r_data[gesture_data_->index] =  fifo_data[i + 3];
      gesture_data_->index++;
      gesture_data_->total_gestures++;
    }
  }
    int8_t temp;
    //TODO fix the reversed logic in processGestureData
    temp = processGestureData(*gesture_data_);
    if( temp ) {
      *num_samps = 0;
      return gesture_motion_;
    }
    *num_samps = 1;
    return  gesture_motion_;
}



void resetGestureFields(gesture_data_t *gesture){
	gesture->index = 0;
	gesture->total_gestures = 0;
	return;
}

void enableGesture_boost(uint8_t boost){
	uint8_t val, enable, mode;
	restartTransmitAPDS();
	/*Write 0 to ENABLE*/
	//writeDataByte(APDS9960_ENABLE, 0);
	writeDataByte(APDS9960_WTIME,0xFF);
	writeDataByte(APDS9960_PPULSE, DEFAULT_GESTURE_PPULSE);
	/*Set LED boost*/
	restartTransmitAPDS();
	writeSingleByte(APDS9960_CONFIG2);
	val = readDataByte();
	//LOG("CONFIG2 = %x \r\n", val);
	boost &=0x3;
	boost = boost << 4;
	val &= 0xCF;
	val |= boost;
	//LOG("Writing %x to CONFIG2 \r\n", val);
	restartTransmitAPDS();
	writeDataByte(APDS9960_CONFIG2, val);
	restartTransmitAPDS();
	writeSingleByte(APDS9960_CONFIG2);
	readDataByte();
	//LOG("Confirmed Config2 = %x \r\n",test);

	/*Enable gesture interrupt... for now*/
	enable = 1;
	restartTransmitAPDS();
	writeSingleByte(APDS9960_GCONF4);
	val = readDataByte();
	//LOG("GCONF4 = %x \r\n", val);
	enable &= 0x1;
	enable = enable << 1;
	val &= 0xFD;
	val |= enable;
#ifdef NO_PERIPHS
  val &= ~enable;
#endif
  /*set gesture mode*/
	mode = 1;
	mode  &= 0x1;
	val &= 0xFE;
	val |= mode;
	//LOG("Writing %x to GCONF4 \r\n", val);
	writeDataByte(APDS9960_GCONF4, val);
	/*Enable power*/
	restartTransmitAPDS();
	writeSingleByte(APDS9960_ENABLE);
	val = readDataByte();
	//LOG("ENABLE = %x \r\n", val);
	val |= (1 << POWER);
#ifdef NO_PERIPHS
  val &= ~(1 << POWER);
#endif
  restartTransmitAPDS();
	//LOG("Writing %x to ENABLE \r\n", val);
	writeDataByte(APDS9960_ENABLE,val);
	/*Enable wait mode*/
	restartTransmitAPDS();
	writeSingleByte(APDS9960_ENABLE);
	val = readDataByte();
	//LOG("ENABLE = %x \r\n", val);
	val |= (1 << WAIT);
	restartTransmitAPDS();
	//LOG("Writing %x to ENABLE \r\n", val);
	writeDataByte(APDS9960_ENABLE,val);
	/*Enable proximity mode*/
	restartTransmitAPDS();
	writeSingleByte(APDS9960_ENABLE);
	val = readDataByte();
	//LOG("ENABLE = %x \r\n", val);
	val |= (1 << PROXIMITY);
	restartTransmitAPDS();
	//LOG("Writing %x to ENABLE \r\n", val);
	writeDataByte(APDS9960_ENABLE,val);
	/*Enable gesture mode*/
	restartTransmitAPDS();
	writeSingleByte(APDS9960_ENABLE);
	val = readDataByte();
	//LOG("ENABLE = %x \r\n", val);
	val |= (1 << GESTURE);
	//LOG("Writing %x to ENABLE \r\n", val);
	restartTransmitAPDS();
	writeDataByte(APDS9960_ENABLE, val);
	restartTransmitAPDS();
	writeSingleByte(APDS9960_ENABLE);
	readDataByte();
	//LOG("Confirmed write %x to ENABLE \r\n");
	//EUSCI_B_I2C_disable(EUSCI_B0_BASE);
	return ;
}

void enableGesture(void){
	uint8_t val, boost, enable, mode;
	restartTransmitAPDS();
	/*Write 0 to ENABLE*/
	//writeDataByte(APDS9960_ENABLE, 0);
	writeDataByte(APDS9960_WTIME,0xFF);
	writeDataByte(APDS9960_PPULSE, DEFAULT_GESTURE_PPULSE);
	/*Set LED boost*/
	//boost = LED_BOOST_300;
	boost = LED_BOOST_200;
	restartTransmitAPDS();
	writeSingleByte(APDS9960_CONFIG2);
	val = readDataByte();
	//LOG("CONFIG2 = %x \r\n", val);
	boost &=0x3;
	boost = boost << 4;
	val &= 0xCF;
	val |= boost;
	//LOG("Writing %x to CONFIG2 \r\n", val);
	restartTransmitAPDS();
	writeDataByte(APDS9960_CONFIG2, val);
	restartTransmitAPDS();
	writeSingleByte(APDS9960_CONFIG2);
	readDataByte();
	//LOG("Confirmed Config2 = %x \r\n",test);

	/*Enable gesture interrupt... for now*/
	enable = 1;
	restartTransmitAPDS();
	writeSingleByte(APDS9960_GCONF4);
	val = readDataByte();
	//LOG("GCONF4 = %x \r\n", val);
	enable &= 0x1;
	enable = enable << 1;
	val &= 0xFD;
	val |= enable;
#ifdef NO_PERIPHS
  val &= ~enable;
#endif
  /*set gesture mode*/
	mode = 1;
	mode  &= 0x1;
	val &= 0xFE;
	val |= mode;
	//LOG("Writing %x to GCONF4 \r\n", val);
	writeDataByte(APDS9960_GCONF4, val);
	/*Enable power*/
	restartTransmitAPDS();
	writeSingleByte(APDS9960_ENABLE);
	val = readDataByte();
	//LOG("ENABLE = %x \r\n", val);
	val |= (1 << POWER);
#ifdef NO_PERIPHS
  val &= ~(1 << POWER);
#endif
  restartTransmitAPDS();
	//LOG("Writing %x to ENABLE \r\n", val);
	writeDataByte(APDS9960_ENABLE,val);
	/*Enable wait mode*/
	restartTransmitAPDS();
	writeSingleByte(APDS9960_ENABLE);
	val = readDataByte();
	//LOG("ENABLE = %x \r\n", val);
	val |= (1 << WAIT);
	restartTransmitAPDS();
	//LOG("Writing %x to ENABLE \r\n", val);
	writeDataByte(APDS9960_ENABLE,val);
	/*Enable proximity mode*/
	restartTransmitAPDS();
	writeSingleByte(APDS9960_ENABLE);
	val = readDataByte();
	//LOG("ENABLE = %x \r\n", val);
	val |= (1 << PROXIMITY);
	restartTransmitAPDS();
	//LOG("Writing %x to ENABLE \r\n", val);
	writeDataByte(APDS9960_ENABLE,val);
	/*Enable gesture mode*/
	restartTransmitAPDS();
	writeSingleByte(APDS9960_ENABLE);
	val = readDataByte();
	//LOG("ENABLE = %x \r\n", val);
	val |= (1 << GESTURE);
	LOG("Writing %x to ENABLE \r\n", val);
	restartTransmitAPDS();
	writeDataByte(APDS9960_ENABLE, val);
	restartTransmitAPDS();
	writeSingleByte(APDS9960_ENABLE);
	readDataByte();
	//LOG("Confirmed write %x to ENABLE \r\n");
	//EUSCI_B_I2C_disable(EUSCI_B0_BASE);
	return ;
}
void disableGesture(){
	restartTransmitAPDS();
  writeDataByte(APDS9960_ENABLE,0x1);
	/*writeSingleByte(APDS9960_ENABLE);
	uint8_t val = readDataByte();
	val &= ~(1 << GESTURE);
	restartTransmitAPDS();
	writeDataByte(APDS9960_ENABLE, val);
//	LOG("New enable = %x \r\n", val);*/
	return;
}

void reenableGesture(){
	restartTransmitAPDS();
  writeDataByte(APDS9960_ENABLE,0x41);
	/*writeSingleByte(APDS9960_ENABLE);
	uint8_t val = readDataByte();
	val |= (1 << GESTURE);
	restartTransmitAPDS();
	writeDataByte(APDS9960_ENABLE, val);*/
	return;
}

void check_mode(void){
	uint8_t val ;
	restartTransmitAPDS();
	writeSingleByte(APDS9960_ENABLE);
	val = readDataByte();
	LOG("enable reg = %x \r\n", val);
	return;
}

/**
 *@brief decides if a sample is outside of the allowed window.
 *@details really just a stand in for something more complicated
 */
int8_t anomalyCheck(uint8_t sample, uint8_t baseline, uint8_t allowedDev){
	//Bounds limits set by max/min values of uint8_t (0x0 and 0xFF)
	if(baseline >= allowedDev){
		//check out of bounds low
		if(sample < baseline - allowedDev)
			return -1;
	}
	if(baseline + allowedDev <= 0xFF){
		//check out of bounds high
		if(sample > baseline + allowedDev)
			return -1;
	}
	return 1;
}


/*
 *@brief consecutively writes register address and value over i2c
 *@details analogous to wirewritedatabyte in wire.h
 */
void writeDataByte(uint8_t reg, uint8_t val){
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
void writeSingleByte(uint8_t val){
  EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);
  EUSCI_B_I2C_masterSendSingleByte(EUSCI_B0_BASE, val);
  while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));
	return;
}

/*
 *@brief reads a byte over i2c
 */
uint8_t readDataByte(){
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
void restartTransmitAPDS(){
	EUSCI_B_I2C_disable(EUSCI_B0_BASE);
  EUSCI_B_I2C_setSlaveAddress(EUSCI_B0_BASE, APDS9960_I2C_ADDR);
  EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);
  EUSCI_B_I2C_enable(EUSCI_B0_BASE);
//  EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);

	while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));
}

int8_t processGestureData(gesture_data_t gesture_data_) {
	uint8_t u_first = 0;
	uint8_t d_first = 0;
	uint8_t l_first = 0;
	uint8_t r_first = 0;
	uint8_t u_last = 0;
	uint8_t d_last = 0;
	uint8_t l_last = 0;
	uint8_t r_last = 0;


	int ud_ratio_first;
	int lr_ratio_first;
	int ud_ratio_last;
	int lr_ratio_last;
	int ud_delta;
	int lr_delta;
	int i;

	gesture_ud_count_ = 0;
	gesture_lr_count_ = 0;

	gesture_near_count_ = 0;
	gesture_far_count_ = 0;

	gesture_state_ = 0;
	gesture_motion_ = DIR_NONE;
	__delay_cycles(200);
	LOG2("PROCESSING GESTURE \r\n");
	/* If we have less than 4 total gestures, that's not enough */
	if( gesture_data_.total_gestures <= 4 ) {
		LOG2("TOO FEW GESTURES got %u  \r\n", gesture_data_.total_gestures);
		return -1;
	}

	/* Check to make sure our data isn't out of bounds */
	if( (gesture_data_.total_gestures <= 32) && \
			(gesture_data_.total_gestures > 0) ) {
#if 0
		LOG("IN BOUNDS \r\n");
#endif
		/* Find the first value in U/D/L/R above the threshold */
		for( i = 0; i < gesture_data_.total_gestures ; i++ ) {
			if( (gesture_data_.u_data[i] > GESTURE_THRESHOLD_OUT) &&
					(gesture_data_.d_data[i] > GESTURE_THRESHOLD_OUT) &&
					(gesture_data_.l_data[i] > GESTURE_THRESHOLD_OUT) &&
					(gesture_data_.r_data[i] > GESTURE_THRESHOLD_OUT) ) {

					u_first = gesture_data_.u_data[i];
					d_first = gesture_data_.d_data[i];
					l_first = gesture_data_.l_data[i];
					r_first = gesture_data_.r_data[i];
					break;
			}
		}

		/* If one of the _first values is 0, then there is no good data */
		if( (u_first == 0) || (d_first == 0) || \
				(l_first == 0) || (r_first == 0) ) {
#if 0
				LOG("NO GOOD DATA :( \r\n");
#endif
				return -1;
		}
			LOG("total gestures = %u\r\n", gesture_data_.total_gestures);
#if 0
			LOG("total gestures = %u\r\n", gesture_data_.total_gestures);
      LOG("U:");
			for( i = 0 ; i < gesture_data_.total_gestures; i++ ) {
						LOG("%u,", gesture_data_.u_data[i]);
      }
			LOG("\r\nD:");
			for( i = 0 ; i < gesture_data_.total_gestures; i++ ) {
						LOG("%u,", gesture_data_.d_data[i]);
      }
			LOG("\r\nL:");
			for( i = 0 ; i < gesture_data_.total_gestures; i++ ) {
						LOG("%u,", gesture_data_.l_data[i]);
      }
			LOG("\r\nR:");
			for( i = 0 ; i < gesture_data_.total_gestures; i++ ) {
						LOG("%u,", gesture_data_.r_data[i]);
      }
			LOG("\r\n");
			//global index writes into array passed by programmer
#endif
			//global index writes into array passed by programmer
			for(int i = 0; i < gesture_data_.total_gestures; i++)
			{
				raw_sample_array_internal[0][i + raw_index_internal] = gesture_data_.u_data[i];
				raw_sample_array_internal[1][i + raw_index_internal] = gesture_data_.d_data[i];
				raw_sample_array_internal[2][i + raw_index_internal] = gesture_data_.l_data[i];
				raw_sample_array_internal[3][i + raw_index_internal] = gesture_data_.r_data[i];
			}
			raw_index_internal += gesture_data_.total_gestures;

			/* Find the last value in U/D/L/R above the threshold */
			for( i = gesture_data_.total_gestures - 1; i >= 0; i-- ) {
				if( (gesture_data_.u_data[i] > GESTURE_THRESHOLD_OUT) &&
						(gesture_data_.d_data[i] > GESTURE_THRESHOLD_OUT) &&
						(gesture_data_.l_data[i] > GESTURE_THRESHOLD_OUT) &&
						(gesture_data_.r_data[i] > GESTURE_THRESHOLD_OUT) ) {
						u_last = gesture_data_.u_data[i];
						d_last = gesture_data_.d_data[i];
						l_last = gesture_data_.l_data[i];
						r_last = gesture_data_.r_data[i];
						break;
				}
      }
    }

  gesture_ud_delta_ = 0;
	gesture_lr_delta_ = 0;


    /* Calculate the first vs. last ratio of up/down and left/right */
	int16_t num, denom;
	LOG2("calculating ratios \r\n");
	num = (u_first - d_first)*100;
  denom = (u_first + d_first);
	ud_ratio_first = div(num, denom);

	num = (l_first - r_first)*100;
  denom = (l_first + r_first);
	lr_ratio_first = div(num, denom);

	num = (u_last - d_last)*100;
  denom = (u_last + d_last);
	ud_ratio_last = div(num, denom);

	num = (l_last - r_last)*100;
  denom = (l_last + r_last);
	lr_ratio_last = div(num, denom);

  /* Determine the difference between the first and last ratios */
  ud_delta = ud_ratio_last - ud_ratio_first;
  lr_delta = lr_ratio_last - lr_ratio_first;

#if 0
    LOG("Deltas: \r\n");
    LOG("UD: %i \r\n", ud_delta);
    LOG(" LR: %i \r\n", lr_delta);
#endif

    /* Accumulate the UD and LR delta values */
    gesture_ud_delta_ += ud_delta;
    gesture_lr_delta_ += lr_delta;

#if 0
    LOG("Accumulations: \r\n");
    LOG("UD: %i \r\n", gesture_ud_delta_);
    LOG(" LR:  %i \r\n", gesture_lr_delta_);
#endif

    return 0;
}

gest_dir decodeGesture(void){
    /* Return if near or far event is detected */
    if( gesture_state_ == NEAR_STATE ) {
        gesture_motion_ = DIR_NEAR;
        PRINTF("Error! returning near\r\n");
        return true;
    } else if ( gesture_state_ == FAR_STATE ) {
        gesture_motion_ = DIR_FAR;
        PRINTF("Error! returning far\r\n");
        return true;
    }
    __delay_cycles(2000);
    LOG2("ABS(UD):%u,  ABS(LR):%u, raw %i %i\r\n",
    abs16(gesture_ud_delta_),abs16(gesture_lr_delta_),
    gesture_ud_delta_,gesture_lr_delta_);
		/*New way to determine swipe direction... */
		if( abs16(gesture_ud_delta_) > abs16(gesture_lr_delta_)){
			gesture_motion_ = gesture_ud_delta_ > 0 ? DIR_DOWN : DIR_UP;
		}
		else if(gesture_lr_delta_ != 0){
			gesture_motion_ = gesture_lr_delta_ > 0 ? DIR_LEFT : DIR_RIGHT;
		}
		else{
			gesture_motion_ = DIR_NONE;
		}
	 return gesture_motion_;
}

void proximity_init_forGest() {
	uint8_t sensorID = 0;
	/*Transmit address and read back returned value*/
	restartTransmitAPDS();
	writeSingleByte(APDS9960_ID);
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
  writeDataByte(APDS9960_ENABLE, 0);
	/*Run through and set a bundle of defaults*/
	writeDataByte(APDS9960_ATIME, DEFAULT_ATIME);
	writeDataByte(APDS9960_WTIME, DEFAULT_WTIME);
	writeDataByte(APDS9960_PPULSE, DEFAULT_PROX_PPULSE);
	writeDataByte(APDS9960_POFFSET_UR, DEFAULT_POFFSET_UR);
	writeDataByte(APDS9960_POFFSET_DL, DEFAULT_POFFSET_DL);
	writeDataByte(APDS9960_CONFIG1, DEFAULT_CONFIG1);
#if 1
	restartTransmitAPDS();
	writeSingleByte(APDS9960_CONTROL);
	uint8_t val = readDataByte();
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
	restartTransmitAPDS();
	writeDataByte(APDS9960_CONTROL,val);
	writeDataByte(APDS9960_PILT, DEFAULT_PILT);
	writeDataByte(APDS9960_PIHT, DEFAULT_PIHT);
	/*Set low thresh for ambient light interrupts*/
	uint16_t thresh = DEFAULT_AILT;
	uint8_t lowByte = thresh & 0x00FF;
	uint8_t highByte = (thresh & 0xFF00) >> 8;
	writeDataByte(APDS9960_AILTL, lowByte);
	writeDataByte(APDS9960_AILTH, highByte);
	/*Set high threshold for ambient light interrupts*/
	thresh = DEFAULT_AIHT;
	lowByte = thresh & 0x00FF;
	highByte = (thresh & 0xFF00 >> 8);
	writeDataByte(APDS9960_AIHTL, lowByte);
	writeDataByte(APDS9960_AIHTH, highByte);
	writeDataByte(APDS9960_PERS, DEFAULT_PERS);
	writeDataByte(APDS9960_CONFIG2, DEFAULT_CONFIG2);
#endif
	writeDataByte(APDS9960_CONFIG3, DEFAULT_CONFIG3);
  // Use this for high power, set if 1 on line 81 to 0
#if 0
  restartTransmitAPDS();
  writeDataByte(APDS9960_CONTROL,0b11001100);
  restartTransmitAPDS();
  writeDataByte(APDS9960_ENABLE, 0x5);
#endif
#if 1
	 /* switching to the gesture stuff*/
	writeDataByte(APDS9960_GPENTH, DEFAULT_GPENTH);
	writeDataByte(APDS9960_GEXTH, DEFAULT_GEXTH);
	writeDataByte(APDS9960_GCONF1, DEFAULT_GCONF1);
	/*Tell device which reg to return, read vals,  modify a couple bits, and write it back*/
	/*This sequence sets gain for gesture engine*/
	restartTransmitAPDS();
	writeSingleByte(APDS9960_GCONF2);
	val = readDataByte();
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
	restartTransmitAPDS();
	writeDataByte(APDS9960_GCONF2, val);
	/*Now write a bunch of values in the usual way...*/
	writeDataByte(APDS9960_GCONF2, val);
	writeDataByte(APDS9960_GOFFSET_U, DEFAULT_GOFFSET);
	writeDataByte(APDS9960_GOFFSET_D, DEFAULT_GOFFSET);
	writeDataByte(APDS9960_GOFFSET_L, DEFAULT_GOFFSET);
	writeDataByte(APDS9960_GOFFSET_R, DEFAULT_GOFFSET);
	writeDataByte(APDS9960_GPULSE, DEFAULT_GPULSE);
	writeDataByte(APDS9960_GCONF3, DEFAULT_GCONF3);
	/*Sanity check on GCONF2*/
	restartTransmitAPDS();
	writeSingleByte(APDS9960_GCONF2);
	//uint8_t test = readDataByte();
	//LOG("Gconf = %x \r\n",test);
	/*enable gesture interrupt with default val*/
	restartTransmitAPDS();
	writeSingleByte(APDS9960_GCONF4);
	val = readDataByte();
	uint8_t enable = DEFAULT_GIEN;
	enable &= 0x01;
	enable = enable << 1;
	val &= 0xFD;
	val |= enable;
	restartTransmitAPDS();
	writeDataByte(APDS9960_GCONF4, val);


  LOG2("Proximity sensor set up. ID:  %x\r\n", proximityID);
	//	enableProximitySensor();
	restartTransmitAPDS();
	writeSingleByte(APDS9960_ID);
	val = readDataByte();
	//loG("Val after prox enable = %x \r\n", val);
#endif
	return;
}
