#include "libmspware/driverlib.h"
#include <libmsp/mem.h>
#include <libmsp/periph.h>
#include <stdint.h>
#include <stdio.h>
#include <libchain/chain.h>
#include <libio/console.h>
#include "proximity.h"
//TODO make the pin defs defined in a makefile so this can be packaged as a lib
#include "pins.h"

#if BOARD_MAJOR == 1 && BOARD_MINOR == 1
#include <libfxl/fxl6408.h>
#endif //BOARD.{MAJOR,MINOR}

static uint8_t abs(int8_t input){
	uint8_t output;
	if(input < 0){
		input = 0 - input;
	}
	output = (uint8_t) input;
	return output;
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


volatile unsigned char proximityId = 0;
void proximity_init(void) {
	uint8_t proximityID = 0;
	uint8_t sensorID = 0;
	//Already init'd i2c, now init connection to APDS
  EUSCI_B_I2C_setSlaveAddress(EUSCI_B0_BASE, APDS9960_I2C_ADDR);
  EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);

  EUSCI_B_I2C_enable(EUSCI_B0_BASE);
  //Might not need from here:
  while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));

  //EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);

  EUSCI_B_I2C_masterSendSingleByte(EUSCI_B0_BASE, APDS9960_ID);

	while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));

	EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_RECEIVE_MODE);

  EUSCI_B_I2C_masterReceiveStart(EUSCI_B0_BASE);

  proximityID = EUSCI_B_I2C_masterReceiveSingle(EUSCI_B0_BASE);

	EUSCI_B_I2C_masterReceiveMultiByteStop(EUSCI_B0_BASE);

  while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));
	/*Transmit address and read back returned value*/
	restartTransmit();
	writeSingleByte(APDS9960_ID);
	sensorID = readDataByte();

	LOG2("I2C ID = %x \r\n",proximityID);
//	LOG("I2C ID2 = %x \r\n",sensorID);
/*
	EUSCI_B_I2C_disable(EUSCI_B0_BASE);

	EUSCI_B_I2C_setSlaveAddress(EUSCI_B0_BASE, APDS9960_I2C_ADDR);
  EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);

  EUSCI_B_I2C_enable(EUSCI_B0_BASE);
*/
	restartTransmit();
  writeDataByte(APDS9960_ENABLE, 0);
	/*Run through and set a bundle of defaults*/
	writeDataByte(APDS9960_ATIME, DEFAULT_ATIME);
	writeDataByte(APDS9960_WTIME, DEFAULT_WTIME);
	writeDataByte(APDS9960_PPULSE, DEFAULT_PROX_PPULSE);
	writeDataByte(APDS9960_POFFSET_UR, DEFAULT_POFFSET_UR);
	writeDataByte(APDS9960_POFFSET_DL, DEFAULT_POFFSET_DL);
	writeDataByte(APDS9960_CONFIG1, DEFAULT_CONFIG1);
	restartTransmit();
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
	restartTransmit();
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
	restartTransmit();
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
	restartTransmit();
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
	restartTransmit();
	writeSingleByte(APDS9960_GCONF2);
	uint8_t test = readDataByte();
	//LOG("Gconf = %x \r\n",test);
	/*enable gesture interrupt with default val*/
	restartTransmit();
	writeSingleByte(APDS9960_GCONF4);
	val = readDataByte();
	uint8_t enable = DEFAULT_GIEN;
	enable &= 0x01;
	enable = enable << 1;
	val &= 0xFD;
	val |= enable;
	restartTransmit();
	writeDataByte(APDS9960_GCONF4, val);


  LOG2("Proximity sensor set up. ID:  %x\r\n", proximityID);
	//	enableProximitySensor();
	restartTransmit();
	writeSingleByte(APDS9960_ID);
	val = readDataByte();
	//loG("Val after prox enable = %x \r\n", val);
	return;
}


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
#else
#error Unsupported photoresistor config
#endif// BOARD_{MAJOR,MINOR}

}

void disable_photoresistor(void){
#if BOARD_MAJOR == 1 && BOARD_MINOR ==0
  P3OUT &= ~BIT0;
#elif BOARD_MAJOR == 1 && BOARD_MINOR == 1
  fxl_clear(BIT_PHOTO_SW);
#else
#error Unsupported photoresistor config
#endif// BOARD_{MAJOR,MINOR}
}

void photores_setup(void){
  P3SEL0 |= BIT0;
  P3SEL1 |= BIT1;
  PM5CTL0 &= ~LOCKLPM5;
}

/*
 *@brief starts reading from the photoresistor ADC comparator pins
 *
 */
int16_t read_photoresistor(void){
   ADC12CTL0 &= ~ADC12ENC;           // Disable conversions

  ADC12CTL1 = ADC12SHP;
#if BOARD_MAJOR == 1 && BOARD_MINOR ==0
  ADC12MCTL0 = ADC12VRSEL_1 | ADC12INCH_13;
#elif BOARD_MAJOR == 1 && BOARD_MINOR == 1
  ADC12MCTL0 = ADC12VRSEL_1 | ADC12INCH_6;
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
  return output;
}

void enableProximitySensor(void){
	/*Set proximity gain*/
	restartTransmit();
	writeSingleByte(APDS9960_CONTROL);
	uint8_t val = readDataByte();
	uint8_t drive = DEFAULT_PGAIN;
	drive &= 0x3;
	drive = drive << 2;
	val &= 0xF3;
	val |=drive;
	restartTransmit();
	writeDataByte(APDS9960_CONTROL, val);
	/*Set LED drive*/
	restartTransmit();
	writeSingleByte(APDS9960_CONTROL);
	val = readDataByte();
	/*Set led drive*/
	drive = DEFAULT_LDRIVE;
	drive = drive << 6;
	val &= 0x3F;
	val |= drive;
	restartTransmit();
	writeDataByte(APDS9960_CONTROL, val);
	/*Disable interrupt*/
	restartTransmit();
	writeSingleByte(APDS9960_ENABLE);
	val = readDataByte();
	uint8_t enable = 0;
	val &= 0xDF;
	restartTransmit();
	writeDataByte(APDS9960_CONTROL, val);
	/*Enable power*/
	restartTransmit();
	writeSingleByte(APDS9960_ENABLE);
	val = readDataByte();
  val |= (1 << POWER);
#ifdef NO_PERIPHS
  /*Disable the power if we're turning off the peripherals*/
  val &= ~(1 << POWER);
#endif
	restartTransmit();
	writeDataByte(APDS9960_ENABLE,val);
	/*Set proximity mode*/
	restartTransmit();
//Only enable proximity detection if we're NOT using the photoresistor
#ifndef USE_PHOTORES
  writeSingleByte(APDS9960_ENABLE);
	val = readDataByte();
	val |= (1 << PROXIMITY);
	restartTransmit();
	writeDataByte(APDS9960_ENABLE,val);
#endif
	return;
}

uint8_t readProximity(){
	uint8_t val = 0;
	restartTransmit();
	//LOG("Waiting on write2 \r\n");
	writeSingleByte(APDS9960_PDATA);
	//LOG("Waiting on read2 \r\n");
	/*Begin added section*/
	EUSCI_B_I2C_disable(EUSCI_B0_BASE);
  EUSCI_B_I2C_setSlaveAddress(EUSCI_B0_BASE, APDS9960_I2C_ADDR);
  EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_RECEIVE_MODE);
  EUSCI_B_I2C_enable(EUSCI_B0_BASE);
  EUSCI_B_I2C_masterReceiveStart(EUSCI_B0_BASE);
//	LOG("Waiting on read3 \r\n");
  val = EUSCI_B_I2C_masterReceiveSingle(EUSCI_B0_BASE);
//	LOG("Waiting on read4 \r\n");
  EUSCI_B_I2C_masterReceiveMultiByteStop(EUSCI_B0_BASE);

	/*End added section*/
	//val = readDataByte();
	return val ;
}

int8_t  getGestureLoop(gesture_data_t *gesture_data_, uint8_t *num_samps){
	/*Test if a gesture is available*/
//	restartTransmit();
  //LOG("Inside get gesture! \r\n");
  EUSCI_B_I2C_disable(EUSCI_B0_BASE);
	EUSCI_B_I2C_setSlaveAddress(EUSCI_B0_BASE, APDS9960_I2C_ADDR);
  while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));
	EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);
  EUSCI_B_I2C_enable(EUSCI_B0_BASE);
	writeSingleByte(APDS9960_ID);
	uint8_t check = readDataByte();
	//LOG("ID Check = %x \r\n",check);
	restartTransmit();
	writeSingleByte(APDS9960_GSTATUS); //Check gstatus!
	uint8_t test = readDataByte();
	test &= APDS9960_GVALID;
	//This NEEDS to be here, TODO turn this into a delay instead
  PRINTF("Gvalid  val= %x \r\n",test);
	restartTransmit();
	writeSingleByte(APDS9960_ENABLE);
	uint8_t enable = readDataByte();
	//LOG("enable val = %x \r\n",enable);
	enable &= 0x41;
	test &= enable;
	if(!test){
		return DIR_NONE;
	}
	uint8_t loop_count = 0;
	/*while loop start*/
	while(1){
		delay(240000);
		uint8_t fifo_level = 0;
		restartTransmit();
		writeSingleByte(APDS9960_GSTATUS);
		uint8_t gstatus = readDataByte();
		LOG("gstatus val 1 = %x \r\n",gstatus);
		if((gstatus & APDS9960_GVALID) == APDS9960_GVALID){
			restartTransmit();
			writeSingleByte(APDS9960_GFLVL);
			fifo_level = readDataByte();
			uint8_t fifo_data[MAX_DATA_SETS * 4], i;
			//LOG("Fifo level = %u \r\n",fifo_level);
			if(fifo_level > 1 ){
				loop_count++;
				/*Read in all of the bytes from the fifo*/
				restartTransmit();
				writeSingleByte(APDS9960_GFIFO_U);
				EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_RECEIVE_MODE);
				EUSCI_B_I2C_masterReceiveStart(EUSCI_B0_BASE);

				/*Note: we multiply by 4 to get UP,DOWN,LEFT,RIGHT captured*/
				for(i = 0; i < fifo_level * 4; i++){
					fifo_data[i] =  EUSCI_B_I2C_masterReceiveSingle(EUSCI_B0_BASE);
				}
				EUSCI_B_I2C_masterReceiveMultiByteStop(EUSCI_B0_BASE);
				while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));
				for(i = 0; i < fifo_level*4; i+=4){
					gesture_data_->u_data[gesture_data_->index] =  fifo_data[i + 0];
					gesture_data_->d_data[gesture_data_->index] =  fifo_data[i + 1];
					gesture_data_->l_data[gesture_data_->index] =  fifo_data[i + 2];
					gesture_data_->r_data[gesture_data_->index] =  fifo_data[i + 3];
					gesture_data_->index++;
					gesture_data_->total_gestures++;
				}
				//LOG("Fifo level = %u , total gestures = %u\r\n", fifo_level,
				/*																								gesture_data_->total_gestures);
				for(i = 0; i < fifo_level * 4; i++){
					LOG("%u ", fifo_data[i]);
				}
          */
				//LOG("\r\n");
			}

				LOG("ESCAPED!\r\n");
				restartTransmit();
				writeSingleByte(APDS9960_GCONF1);
				fifo_level = readDataByte();
				LOG("GCONF1 = %x \r\n", fifo_level);
				restartTransmit();
				writeSingleByte(APDS9960_GEXTH);
				fifo_level = readDataByte();
				LOG("GEXTH = %u \r\n", fifo_level);
				//TODO fix the reversed logic in processGestureData
				if(processGestureData(*gesture_data_) >= 0){
						//LOG("Decoding gesture! \r\n");
						decodeGesture();
					}
				//LOG("Guess = %u \r\n", gesture_motion_);
				gesture_data_->index = 0;
				gesture_data_->total_gestures = 0;
			}
			else{
				delay(FIFO_PAUSE);
				//LOG("ESCAPED ALL!\r\n");
				*num_samps = loop_count;
				return  gesture_motion_;
			}

		}
}

int8_t  getGesture(gesture_data_t *gesture_data_, uint8_t *num_samps){
	/*Test if a gesture is available*/
//	restartTransmit();
  LOG("Inside get gesture! \r\n");
  EUSCI_B_I2C_disable(EUSCI_B0_BASE);
	EUSCI_B_I2C_setSlaveAddress(EUSCI_B0_BASE, APDS9960_I2C_ADDR);
  while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));
	EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);
  EUSCI_B_I2C_enable(EUSCI_B0_BASE);
	writeSingleByte(APDS9960_ID);
	uint8_t check = readDataByte();
	LOG("ID Check = %x \r\n",check);
	restartTransmit();
	/*while loop start*/

	writeSingleByte(APDS9960_GSTATUS); //Check gstatus!
	uint8_t test = readDataByte();
	test &= APDS9960_GVALID;
	LOG("Gvalid  val= %x \r\n",test);
	restartTransmit();
	writeSingleByte(APDS9960_ENABLE);
	uint8_t enable = readDataByte();
	LOG("enable val = %x \r\n",enable);
	enable &= 0x41;
	test &= enable;
	if(!test){
		return DIR_NONE;
	}
	uint8_t fifo_level = 0;
	restartTransmit();
	writeSingleByte(APDS9960_GSTATUS);
	uint8_t gstatus = readDataByte();
	LOG("gstatus val 1 = %x \r\n",gstatus);
	if((gstatus & APDS9960_GVALID) == APDS9960_GVALID){
		restartTransmit();
		writeSingleByte(APDS9960_GFLVL);
		fifo_level = readDataByte();
		uint8_t fifo_data[MAX_DATA_SETS * 4], i;
		LOG("Fifo level = %u \r\n",fifo_level);
		if(fifo_level > 1 ){
			/*Read in all of the bytes from the fifo*/
			restartTransmit();
			writeSingleByte(APDS9960_GFIFO_U);
			EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_RECEIVE_MODE);
			EUSCI_B_I2C_masterReceiveStart(EUSCI_B0_BASE);

			/*Note: we multiply by 4 to get UP,DOWN,LEFT,RIGHT captured*/
			for(i = 0; i < fifo_level * 4; i++){
				fifo_data[i] =  EUSCI_B_I2C_masterReceiveSingle(EUSCI_B0_BASE);
			}
			EUSCI_B_I2C_masterReceiveMultiByteStop(EUSCI_B0_BASE);
			while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));
			for(i = 0; i < fifo_level; i++){
				gesture_data_->u_data[gesture_data_->index] =  fifo_data[i + 0];
				gesture_data_->d_data[gesture_data_->index] =  fifo_data[i + 1];
				gesture_data_->l_data[gesture_data_->index] =  fifo_data[i + 2];
				gesture_data_->r_data[gesture_data_->index] =  fifo_data[i + 3];
				gesture_data_->index++;
				gesture_data_->total_gestures++;
			}
			LOG("Fifo level = %u \r\n", fifo_level);
			for(i = 0; i < fifo_level * 4; i++){
				LOG("%u ", fifo_data[i]);
				}
				LOG("\r\n");
		}

			LOG("ESCAPED!\r\n");
		}
		LOG("ESCAPED ALL!\r\n");
		restartTransmit();
		/*Add in processing stuff here*/
	*num_samps = fifo_level * 4;
	return DIR_UP;
}


void resetGestureFields(gesture_data_t *gesture){
	gesture->index = 0;
	gesture->total_gestures = 0;
	return;
}

void enableGesture(void){
	uint8_t val, boost, enable, mode,test;
	restartTransmit();
	/*Write 0 to ENABLE*/
	//writeDataByte(APDS9960_ENABLE, 0);
	writeDataByte(APDS9960_WTIME,0xFF);
	writeDataByte(APDS9960_PPULSE, DEFAULT_GESTURE_PPULSE);
	/*Set LED boost*/
	//boost = LED_BOOST_300;
	boost = LED_BOOST_200;
	restartTransmit();
	writeSingleByte(APDS9960_CONFIG2);
	val = readDataByte();
	//LOG("CONFIG2 = %x \r\n", val);
	boost &=0x3;
	boost = boost << 4;
	val &= 0xCF;
	val |= boost;
	//LOG("Writing %x to CONFIG2 \r\n", val);
	restartTransmit();
	writeDataByte(APDS9960_CONFIG2, val);
	restartTransmit();
	writeSingleByte(APDS9960_CONFIG2);
	test = readDataByte();
	//LOG("Confirmed Config2 = %x \r\n",test);

	/*Enable gesture interrupt... for now*/
	enable = 1;
	restartTransmit();
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
	restartTransmit();
	writeSingleByte(APDS9960_ENABLE);
	val = readDataByte();
	//LOG("ENABLE = %x \r\n", val);
	val |= (1 << POWER);
#ifdef NO_PERIPHS
  val &= ~(1 << POWER);
#endif
  restartTransmit();
	//LOG("Writing %x to ENABLE \r\n", val);
	writeDataByte(APDS9960_ENABLE,val);
	/*Enable wait mode*/
	restartTransmit();
	writeSingleByte(APDS9960_ENABLE);
	val = readDataByte();
	//LOG("ENABLE = %x \r\n", val);
	val |= (1 << WAIT);
	restartTransmit();
	//LOG("Writing %x to ENABLE \r\n", val);
	writeDataByte(APDS9960_ENABLE,val);
	/*Enable proximity mode*/
	restartTransmit();
	writeSingleByte(APDS9960_ENABLE);
	val = readDataByte();
	//LOG("ENABLE = %x \r\n", val);
	val |= (1 << PROXIMITY);
	restartTransmit();
	//LOG("Writing %x to ENABLE \r\n", val);
	writeDataByte(APDS9960_ENABLE,val);
	/*Enable gesture mode*/
	restartTransmit();
	writeSingleByte(APDS9960_ENABLE);
	val = readDataByte();
	//LOG("ENABLE = %x \r\n", val);
	val |= (1 << GESTURE);
	//LOG("Writing %x to ENABLE \r\n", val);
	restartTransmit();
	writeDataByte(APDS9960_ENABLE, val);
	restartTransmit();
	writeSingleByte(APDS9960_ENABLE);
	test = readDataByte();
	LOG("Confirmed write %x to ENABLE \r\n");
	//EUSCI_B_I2C_disable(EUSCI_B0_BASE);
	return ;
}

void disableGesture(){
	restartTransmit();
	writeSingleByte(APDS9960_ENABLE);
	uint8_t val = readDataByte();
	val &= ~(1 << GESTURE);
	restartTransmit();
	writeDataByte(APDS9960_ENABLE, val);
//	LOG("New enable = %x \r\n", val);
	return;
}

void reenableGesture(){
	restartTransmit();
	writeSingleByte(APDS9960_ENABLE);
	uint8_t val = readDataByte();
	val |= (1 << GESTURE);
	restartTransmit();
	writeDataByte(APDS9960_ENABLE, val);
	return;
}

void check_mode(void){
	uint8_t val ;
	restartTransmit();
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
void restartTransmit(void){
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
	gesture_ud_delta_ = 0;
	gesture_lr_delta_ = 0;

	gesture_ud_count_ = 0;
	gesture_lr_count_ = 0;

	gesture_near_count_ = 0;
	gesture_far_count_ = 0;

	gesture_state_ = 0;
	gesture_motion_ = DIR_NONE;
	__delay_cycles(200);
	LOG("PROCESSING GESTURE \r\n");
	/* If we have less than 4 total gestures, that's not enough */
	if( gesture_data_.total_gestures <= 4 ) {
#if DEBUG
		LOG("TOO FEW GESTURES got %u  \r\n", gesture_data_.total_gestures);
#endif
		return -1;
	}

	/* Check to make sure our data isn't out of bounds */
	if( (gesture_data_.total_gestures <= 32) && \
			(gesture_data_.total_gestures > 0) ) {
#if DEBUG
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
#if DEBUG
				LOG("NO GOOD DATA :( \r\n");
#endif
				return -1;
		}
#if MY_DEBUG
			for( i = 0 ; i < gesture_data_.total_gestures; i++ ) {
						LOG("U: %u ", gesture_data_.u_data[i]);
      }
			LOG("\r\n");
			for( i = 0 ; i < gesture_data_.total_gestures; i++ ) {
						LOG("D: %u ", gesture_data_.d_data[i]);
      }
			LOG("\r\n");
			for( i = 0 ; i < gesture_data_.total_gestures; i++ ) {
						LOG("L: %u ", gesture_data_.l_data[i]);
      }
			LOG("\r\n");
			for( i = 0 ; i < gesture_data_.total_gestures; i++ ) {
						LOG("R: %u ", gesture_data_.r_data[i]);
      }
			LOG("\r\n");

			LOG("total gestures = %u, hunting for last! \r\n", gesture_data_.total_gestures);
#endif
			/* Find the last value in U/D/L/R above the threshold */
			for( i = gesture_data_.total_gestures - 1; i >= 0; i-- ) {
				if( (gesture_data_.u_data[i] > GESTURE_THRESHOLD_OUT) &&
						(gesture_data_.d_data[i] > GESTURE_THRESHOLD_OUT) &&
						(gesture_data_.l_data[i] > GESTURE_THRESHOLD_OUT) &&
						(gesture_data_.r_data[i] > GESTURE_THRESHOLD_OUT) ) {
#if  0//used to be MY_DEBUG
						LOG("U: %u \r\n", gesture_data_.u_data[i]);
						LOG("D: %u \r\n", gesture_data_.d_data[i]);
						LOG("L: %u \r\n", gesture_data_.l_data[i]);
						LOG("R: %u \r\n", gesture_data_.r_data[i]);
#endif
						u_last = gesture_data_.u_data[i];
						d_last = gesture_data_.d_data[i];
						l_last = gesture_data_.l_data[i];
						r_last = gesture_data_.r_data[i];
						break;
				}
      }
#if 0 //used to be MY_DEBUG
						LOG("Found first: \r\n");
						LOG(" U: %u \r\n", u_first);
						LOG(" D: %u \r\n",d_first);
						LOG(" L: %u \r\n",l_first );
						LOG(" R: %u \r\n",r_first);

						LOG("Found last: \r\n ");
						/*
						LOG("U: %u \r\n",gesture_data_.u_data[i]);
						LOG(" D: %u \r\n",gesture_data_.d_data[i]);
						LOG(" L: %u \r\n", gesture_data_.l_data[i]);
						LOG(" R: %u \r\n",gesture_data_.r_data[i]);
						*/
						LOG("U: %u \r\n", u_last);
						LOG(" D: %u \r\n",d_last);
						LOG(" L: %u \r\n",l_last );
						LOG(" R: %u \r\n",r_last);

#endif
    }

    /* Calculate the first vs. last ratio of up/down and left/right */
	int16_t num, denom;
	LOG("calculating ratios \r\n");
	num = (u_first - d_first)*100; denom = (u_first + d_first);
	ud_ratio_first = div(num, denom);

	num = (l_first - r_first)*100; denom = (l_first + r_first);
	lr_ratio_first = div(num, denom);

	num = (u_last - d_last)*100; denom = (u_last + d_last);
	ud_ratio_last = div(num, denom);

	num = (l_last - r_last)*100; denom = (l_last + r_last);
	lr_ratio_last = div(num, denom);


#if 0
    LOG("Last Values: ");
    LOG("U: %u \r\n", u_last);
    LOG(" D: %u \r\n", d_last);
    LOG(" L: %u \r\n",l_last);
    LOG(" R: %u \r\n",r_last);

    LOG("Ratios: \r\n");
    LOG(" UD Fi: %i \r\n",ud_ratio_first);
    LOG(" UD La: %i \r\n", ud_ratio_last);
    LOG(" LR Fi: %i \r\n", lr_ratio_first);
    LOG(" LR La: %i \r\n", lr_ratio_last);
#endif

    /* Determine the difference between the first and last ratios */
		ud_delta = ud_ratio_last - ud_ratio_first;
    lr_delta = lr_ratio_last - lr_ratio_first;

#if DEBUG
    LOG("Deltas: \r\n");
    LOG("UD: %i \r\n", ud_delta);
    LOG(" LR: %i \r\n", lr_delta);
#endif

    /* Accumulate the UD and LR delta values */
    gesture_ud_delta_ += ud_delta;
    gesture_lr_delta_ += lr_delta;

#if DEBUG
    LOG("Accumulations: \r\n");
    LOG("UD: %i \r\n", gesture_ud_delta_);
    LOG(" LR:  %i \r\n", gesture_lr_delta_);
#endif


    /* Determine U/D gesture */
    if( gesture_ud_delta_ >= GESTURE_SENSITIVITY_1 ) {
        gesture_ud_count_ = 1;
    } else if( gesture_ud_delta_ <= -GESTURE_SENSITIVITY_1 ) {
        gesture_ud_count_ = -1;
    } else {
        gesture_ud_count_ = 0;
    }

    /* Determine L/R gesture */
    if( gesture_lr_delta_ >= GESTURE_SENSITIVITY_1 ) {
        gesture_lr_count_ = 1;
    } else if( gesture_lr_delta_ <= -GESTURE_SENSITIVITY_1 ) {
        gesture_lr_count_ = -1;
    } else {
        gesture_lr_count_ = 0;
    }

    /* Determine Near/Far gesture */
    if( (gesture_ud_count_ == 0) && (gesture_lr_count_ == 0) ) {
        if( (abs(ud_delta) < GESTURE_SENSITIVITY_2) && \
            (abs(lr_delta) < GESTURE_SENSITIVITY_2) ) {

            if( (ud_delta == 0) && (lr_delta == 0) ) {
                gesture_near_count_++;
            } else if( (ud_delta != 0) || (lr_delta != 0) ) {
                gesture_far_count_++;
            }

            if( (gesture_near_count_ >= 10) && (gesture_far_count_ >= 2) ) {
                if( (ud_delta == 0) && (lr_delta == 0) ) {
                    gesture_state_ = NEAR_STATE;
                } else if( (ud_delta != 0) && (lr_delta != 0) ) {
                    gesture_state_ = FAR_STATE;
                }
                return 0;
            }
        }
    } else {
        if( (abs(ud_delta) < GESTURE_SENSITIVITY_2) && \
            (abs(lr_delta) < GESTURE_SENSITIVITY_2) ) {

            if( (ud_delta == 0) && (lr_delta == 0) ) {
                gesture_near_count_++;
            }

            if( gesture_near_count_ >= 10 ) {
                gesture_ud_count_ = 0;
                gesture_lr_count_ = 0;
                gesture_ud_delta_ = 0;
                gesture_lr_delta_ = 0;
            }
        }
    }

#if DEBUG
    LOG("UD_CT: %i \r\n",gesture_ud_count_);
    LOG(" LR_CT: %i \r\n", gesture_lr_count_);
    LOG(" NEAR_CT: %i \r\n", gesture_near_count_);
    LOG(" FAR_CT: %i \r\n",gesture_far_count_);
    LOG("----------\r\n");
#endif

    return -1;
}

gest_dir decodeGesture(void){
    /* Return if near or far event is detected */
    if( gesture_state_ == NEAR_STATE ) {
        gesture_motion_ = DIR_NEAR;
        return true;
    } else if ( gesture_state_ == FAR_STATE ) {
        gesture_motion_ = DIR_FAR;
        return true;
    }

    /* Determine swipe direction */
#ifdef DUMMY
  if( (gesture_ud_count_ == -1) && (gesture_lr_count_ == 0) ) {
        gesture_motion_ = DIR_UP;
    } else if( (gesture_ud_count_ == 1) && (gesture_lr_count_ == 0) ) {
        gesture_motion_ = DIR_DOWN;
    } else if( (gesture_ud_count_ == 0) && (gesture_lr_count_ == 1) ) {
        gesture_motion_ = DIR_RIGHT;
    } else if( (gesture_ud_count_ == 0) && (gesture_lr_count_ == -1) ) {
        gesture_motion_ = DIR_LEFT;
    } else if( (gesture_ud_count_ == -1) && (gesture_lr_count_ == 1) ) {
        if( abs(gesture_ud_delta_) > abs(gesture_lr_delta_) ) {
            gesture_motion_ = DIR_UP;
        } else {
            gesture_motion_ = DIR_RIGHT;
        }
    } else if( (gesture_ud_count_ == 1) && (gesture_lr_count_ == -1) ) {
        if( abs(gesture_ud_delta_) > abs(gesture_lr_delta_) ) {
            gesture_motion_ = DIR_DOWN;
        } else {
            gesture_motion_ = DIR_LEFT;
        }
    } else if( (gesture_ud_count_ == -1) && (gesture_lr_count_ == -1) ) {
        if( abs(gesture_ud_delta_) > abs(gesture_lr_delta_) ) {
            gesture_motion_ = DIR_UP;
        } else {
            gesture_motion_ = DIR_LEFT;
        }
    } else if( (gesture_ud_count_ == 1) && (gesture_lr_count_ == 1) ) {
        if( abs(gesture_ud_delta_) > abs(gesture_lr_delta_) ) {
            gesture_motion_ = DIR_DOWN;
        } else {
            gesture_motion_ = DIR_RIGHT;
        }
    } else {
						gesture_motion_ =  DIR_NONE;
    }
#else
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
#endif
  #if DEBUG
//		LOG("--------------Dir = %u---------------\r\n", gesture_motion_);
//		delay(50000000);
	#endif
	 return gesture_motion_;
}

