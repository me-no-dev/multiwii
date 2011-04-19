// *********************
// I2C general functions
// *********************

// Mask prescaler bits : only 5 bits of TWSR defines the status of each I2C request
#define TW_STATUS_MASK	(1<<TWS7) | (1<<TWS6) | (1<<TWS5) | (1<<TWS4) | (1<<TWS3)
#define TW_STATUS       (TWSR & TW_STATUS_MASK)

void i2c_init(void) {
  #if defined(INTERNAL_I2C_PULLUPS)
    I2C_PULLUPS_ENABLE
  #else
    I2C_PULLUPS_DISABLE
  #endif
  TWSR = 0;        // no prescaler => prescaler = 1
  TWBR = ((16000000L / I2C_SPEED) - 16) / 2; // change the I2C clock rate
  TWCR = 1<<TWEN;  // enable twi module, no interrupt
}

void i2c_rep_start(uint8_t address) {
  TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN) | (1<<TWSTO); // send REAPEAT START condition
  waitTransmissionI2C(); // wait until transmission completed
  checkStatusI2C(); // check value of TWI Status Register
  TWDR = address; // send device address
  TWCR = (1<<TWINT) | (1<<TWEN);
  waitTransmissionI2C(); // wail until transmission completed
  checkStatusI2C(); // check value of TWI Status Register
}

void i2c_write(uint8_t data ) {	
  TWDR = data; // send data to the previously addressed device
  TWCR = (1<<TWINT) | (1<<TWEN);
  waitTransmissionI2C(); // wait until transmission completed
  checkStatusI2C(); // check value of TWI Status Register
}

uint8_t i2c_readAck() {
  TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
  waitTransmissionI2C();
  return TWDR;
}

uint8_t i2c_readNak(void) {
  TWCR = (1<<TWINT) | (1<<TWEN);
  waitTransmissionI2C();
  return TWDR;
}

void waitTransmissionI2C() {
  uint8_t count = 255;
  while (count-->0 && !(TWCR & (1<<TWINT)) );
  if (count<2) { //we are in a blocking state => we don't insist
    TWCR = 0;  //and we force a reset on TWINT register
    neutralizeTime = micros(); //we take a timestamp here to neutralize the value during a short delay after the hard reset
  }
}

void checkStatusI2C() {
  if ( TW_STATUS  == 0xF8) { //TW_NO_INFO : this I2C error status indicates a wrong I2C communication.
    // WMP does not respond anymore => we do a hard reset. I did not find another way to solve it. It takes only 13ms to reset and init to WMP or WMP+NK
    TWCR = 0;
    POWERPIN_OFF //switch OFF WMP
    delay(1);  
    POWERPIN_ON  //switch ON WMP
    delay(10);
    #if defined(ITG3200) || defined(L3G4200D)
    #else
      i2c_WMP_init(0);
    #endif
    neutralizeTime = micros(); //we take a timestamp here to neutralize the WMP or WMP+NK values during a short delay after the hard reset
  }
}


// **************************
// I2C Barometer BOSCH BMP085
// **************************
// I2C adress: 0xEE (8bit)   0x77 (7bit)
// principle:
//  1) read the calibration register (only once at the initialization)
//  2) read uncompensated temperature (not mandatory at every cycle)
//  3) read uncompensated pressure
//  4) raw temp + raw pressure => calculation of the adjusted pressure
//  the following code uses the maximum precision setting (oversampling setting 3)

// sensor registers from the BOSCH BMP085 datasheet
#if defined(BMP085)
static int16_t ac1,ac2,ac3,b1,b2,mb,mc,md;
static uint16_t  ac4,ac5,ac6;

static uint16_t ut; //uncompensated T
static uint32_t up; //uncompensated P
int32_t temperature = 0;
int32_t pressure = 0;
int16_t altitude = 0;
static int16_t altitudeZero;
static int16_t altitudeHold;

void  i2c_Baro_init() {
  delay(10);
  ac1 = i2c_BMP085_readIntRegister(0xAA);
  ac2 = i2c_BMP085_readIntRegister(0xAC);
  ac3 = i2c_BMP085_readIntRegister(0xAE);
  ac4 = i2c_BMP085_readIntRegister(0xB0);
  ac5 = i2c_BMP085_readIntRegister(0xB2);
  ac6 = i2c_BMP085_readIntRegister(0xB4);
  b1  = i2c_BMP085_readIntRegister(0xB6);
  b2  = i2c_BMP085_readIntRegister(0xB8);
  mb  = i2c_BMP085_readIntRegister(0xBA);
  mc  = i2c_BMP085_readIntRegister(0xBC);
  md  = i2c_BMP085_readIntRegister(0xBE);
  
  baroPresent = 1;
  i2c_BMP085_calibrate();
}

// read a 16 bit register
int16_t i2c_BMP085_readIntRegister(unsigned char r) {
  uint8_t msb, lsb;
  
  i2c_rep_start(0xEE + 0);
  i2c_write(r);
  i2c_rep_start(0xEE + 1);//I2C read direction => 1
  msb=i2c_readAck();
  lsb=i2c_readNak();
  return (((int16_t)msb<<8) | ((int16_t)lsb));
}

// read uncompensated temperature value: send command first
void i2c_BMP085_readUT_Command() {
  i2c_rep_start(0xEE + 0);
  i2c_write(0xf4);
  i2c_write(0x2e);
  i2c_rep_start(0xEE + 0);
  i2c_write(0xF6);
}

// read uncompensated temperature value: read result bytes
// the datasheet suggests a delay of 4.5 ms after the send command
uint16_t i2c_BMP085_readUT_Result() {
  uint8_t msb, lsb;
  i2c_rep_start(0xEE + 1);//I2C read direction => 1
  msb=i2c_readAck();
  lsb=i2c_readNak();
  return (((uint16_t)msb<<8) | ((uint16_t)lsb));
}

// read uncompensated pressure value: send command first
void i2c_BMP085_readUP_Command () {
  i2c_rep_start(0xEE + 0);
  i2c_write(0xf4);
  i2c_write(0xf4); //control register value for oversampling setting 3
  i2c_rep_start(0xEE + 0); //I2C write direction => 0
  i2c_write(0xf6);
}

// read uncompensated pressure value: read result bytes
// the datasheet suggests a delay of 25.5 ms (oversampling settings 3) after the send command
uint32_t i2c_BMP085_readUP_Result () {
  uint8_t msb, lsb, xlsb;
  i2c_rep_start(0xEE + 1);//I2C read direction => 1
  msb = i2c_readAck();
  lsb = i2c_readAck();
  xlsb = i2c_readNak();
  return (((uint32_t)msb<<16) | ((uint32_t)lsb<<8) | ((uint32_t)xlsb)) >>5;
}

// deduction of true temperature and pressure from sensor, code is described in the BMP085 specs
void i2c_BMP085_CompensatedSensor() {
  int32_t x1, x2, x3, b3, b5, b6, p;
  uint32_t b4, b7;
  uint16_t a,b;

  //calculate true temperature
  x1 = (int32_t)(ut - ac6) * ac5 >> 15;
  x2 = ((int32_t) mc << 11) / (x1 + md);
  b5 = x1 + x2;
  temperature = (b5 + 8) >> 4;
  //calculate true pressure
  b6 = b5 - 4000;
  x1 = (b2 * (b6 * b6 >> 12)) >> 11; 
  x2 = ac2 * b6 >> 11;
  x3 = x1 + x2;
  b3 = ( ( ((int32_t) ac1 * 4 + x3)<<3 ) + 2) >> 2;
  x1 = ac3 * b6 >> 13;
  x2 = (b1 * (b6 * b6 >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = (ac4 * (uint32_t) (x3 + 32768)) >> 15;
  b7 = (up - b3) * (50000 >> 3);
  p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;
  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  pressure = p + ((x1 + x2 + 3791) >> 4);
}

//in a whole cycle: we read temperature one time and pressure 5 times
void i2c_Baro_update() {
  static uint32_t t;
  static uint8_t state1 =0,state2 = 0;

  if ( (micros()-t )  < 30000 ) return; //each read is spaced by 30ms
  t = micros();
  
  TWBR = ((16000000L / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz, BMP085 is ok with this speed
  if (state1 == 0) { 
    if (state2 == 0) {
      i2c_BMP085_readUT_Command();
      state2=1;
    } else {
      ut = i2c_BMP085_readUT_Result();
      state2=0;
      state1=5;
    }
  } else {
    if (state2 == 0) {
      i2c_BMP085_readUP_Command();
      state2=1;
    } else {
      up = i2c_BMP085_readUP_Result();
      state2=0;
      state1--;
      i2c_BMP085_CompensatedSensor();
      altitude = (1.0 - pow(float(pressure)/101325.0, 0.190295)) * 443300 - altitudeZero; // altitude in decimeter from starting point
      if ( abs(altitude-altitudeSmooth) < 100 ) //avoid altitude spike
        altitudeSmooth = (altitudeSmooth*7+altitude+4)/8;
    }
  }
}

void i2c_BMP085_calibrate() {
  altitudeZero = 0;
  for (uint8_t i=0;i<7;i++) {
    i2c_Baro_update();
    delay(35);
  }
  altitudeZero = altitude;
}
#endif


// **************************
// I2C Accelerometer ADXL345 
// **************************
// I2C adress: 0x3A (8bit)    0x1D (7bit)
// principle:
//  1) CS PIN must be linked to VCC to select the I2C mode
//  2) SD0 PIN must be linked to VCC to select the right I2C adress
//  3) bit  b00000100 must be set on register 0x2D to read data (only once at the initialization)
//  4) bits b00001011 must be set on register 0x31 to select the data format (only once at the initialization)
#if defined(ADXL345)
static uint8_t rawADC_ADXL345[6];

void i2c_ACC_init () {
  delay(10);
  i2c_rep_start(0x3A+0);      // I2C write direction
  i2c_write(0x2D);            // register 2D Power CTRL
  i2c_write(1<<3);            // Set measure bit 3 on
  i2c_rep_start(0x3A+0);      // I2C write direction 
  i2c_write(0x31);            // DATA_FORMAT register
  i2c_write(0x0B);            // Set bits 3(full range) and 1 0 on (+/- 16g-range)
  i2c_rep_start(0x3A+0);      // I2C write direction 
  i2c_write(0x2C);            // BW_RATE
  i2c_write(8+2+1);           // 200Hz sampling (see table 5 of the spec)

  acc_1G = 250;
  accPresent = 1;
}

void i2c_ACC_getADC () {
  TWBR = ((16000000L / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz, ADXL435 is ok with this speed
  i2c_rep_start(0x3A);     // I2C write direction
  i2c_write(0x32);         // Start multiple read at reg 0x32 ADX
  i2c_rep_start(0x3A +1);  // I2C read direction => 1
  for(uint8_t i = 0; i < 5; i++) {
    rawADC_ADXL345[i]=i2c_readAck();}
  rawADC_ADXL345[5]= i2c_readNak();

  accADC[ROLL]  = - ((rawADC_ADXL345[3]<<8) | rawADC_ADXL345[2]);
  accADC[PITCH] =   ((rawADC_ADXL345[1]<<8) | rawADC_ADXL345[0]);
  accADC[YAW]   = - ((rawADC_ADXL345[5]<<8) | rawADC_ADXL345[4]);
}
#endif


// **************************
// contribution from opie11 (rc-grooups)
// I2C Accelerometer BMA180
// **************************
// I2C adress: 0x80 (8bit)    0x40 (7bit)
#if defined(BMA180)
static uint8_t rawADC_BMA180[6];

void i2c_ACC_init () {
  delay(10);
  i2c_rep_start(0x80+0);      // I2C write direction 
  i2c_write(0x0D);            // ctrl_reg0
  i2c_write(1<<4);            // Set bit 4 to 1 to enable writing
  i2c_rep_start(0x80+0);       
  i2c_write(0x35);            // 
  i2c_write(3<<1);            // range set to 3.  2730 1G raw data.  With /10 divisor on acc_ADC, more in line with other sensors and works with the GUI
  i2c_rep_start(0x80+0);
  i2c_write(0x20);            // bw_tcs reg: bits 4-7 to set bw
  i2c_write(0<<4);            // bw to 10Hz (low pass filter)

  acc_1G = 273;
  accPresent = 1;
}

void i2c_ACC_getADC () {
  TWBR = ((16000000L / 400000L) - 16) / 2;  // Optional line.  Sensor is good for it in the spec.
  i2c_rep_start(0x80);     // I2C write direction
  i2c_write(0x02);         // Start multiple read at reg 0x02 acc_x_lsb
  i2c_rep_start(0x80 +1);  // I2C read direction => 1
  for(uint8_t i = 0; i < 5; i++) {
    rawADC_BMA180[i]=i2c_readAck();}
  rawADC_BMA180[5]= i2c_readNak();

  accADC[ROLL]  = - (((rawADC_BMA180[1]<<8) | (rawADC_BMA180[0]))>>2)/10; // opie settings: + ; FFIMU: -
  accADC[PITCH] = - (((rawADC_BMA180[3]<<8) | (rawADC_BMA180[2]))>>2)/10;
  accADC[YAW]   = - (((rawADC_BMA180[5]<<8) | (rawADC_BMA180[4]))>>2)/10;
}
#endif

// **************
// contribution from Point65 and mgros (rc-grooups)
// BMA020 I2C
// **************
// I2C adress: 0x70 (8bit)
#if defined(BMA020)
static uint8_t rawADC_BMA020[6];

void i2c_ACC_init(){
  byte control;
  
  i2c_rep_start(0x70);     // I2C write direction
  i2c_write(0x15);         // 
  i2c_write(0x80);         // Write B10000000 at 0x15 init BMA020

  i2c_rep_start(0x70);     // 
  i2c_write(0x14);         //  
  i2c_write(0x71);         // 
  i2c_rep_start(0x71);     //
  control = i2c_readNak();
 
  control = control >> 5;  //ensure the value of three fist bits of reg 0x14 see BMA020 documentation page 9
  control = control << 2;
  control = control | 0x00; //Range 2G 00
  control = control << 3;
  control = control | 0x00; //Bandwidth 25 Hz 000
 
  i2c_rep_start(0x70);     // I2C write direction
  i2c_write(0x14);         // Start multiple read at reg 0x32 ADX
  i2c_write(control);

  acc_1G = 240;
  accPresent = 1;
}

void i2c_ACC_getADC(){
  TWBR = ((16000000L / 400000L) - 16) / 2;
  i2c_rep_start(0x70);     // I2C write direction
  i2c_write(0x02);         // Start multiple read at reg 0x32 ADX
  i2c_write(0x71);  
  i2c_rep_start(0x71);  //I2C read direction => 1
  for(uint8_t i = 0; i < 5; i++) {
    rawADC_BMA020[i]=i2c_readAck();}
  rawADC_BMA020[5]= i2c_readNak();

  accADC[ROLL]  =  (((rawADC_BMA020[1])<<8) | ((rawADC_BMA020[0]>>1)<<1))/64;
  accADC[PITCH] =  (((rawADC_BMA020[3])<<8) | ((rawADC_BMA020[2]>>1)<<1))/64;
  accADC[YAW]   = -(((rawADC_BMA020[5])<<8) | ((rawADC_BMA020[4]>>1)<<1))/64;
}
#endif


// **************************
// contribution from Ciskje
// I2C Gyroscope L3G4200D 
// **************************
#if defined(L3G4200D)
static uint8_t rawADC_L3G4200D[6];

void i2c_Gyro_init() {
  delay(100);
  i2c_rep_start(0XD2+0);      // CTRL_REG1
  i2c_write(0x20);            // 400Hz ODR, 20hz filter, run!
  i2c_write(0x8F); 
  i2c_rep_start(0XD2+0);      // CTRL_REG5
  i2c_write(0x24);            // low pass filter enable
  i2c_write(0x02);
  
  gyroPresent = 1;  
}

void i2c_Gyro_getADC () {
  TWBR = ((16000000L / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz
  i2c_rep_start(0XD2);     // I2C write direction
  i2c_write(0x80 | 0x28);  // Start multiple read
  i2c_rep_start(0XD2 +1);  // I2C read direction => 1
  for(uint8_t i = 0; i < 5; i++) {
    rawADC_L3G4200D[i]=i2c_readAck();}
  rawADC_L3G4200D[5]= i2c_readNak();

  gyroADC[ROLL]  =  ((rawADC_L3G4200D[1]<<8) | rawADC_L3G4200D[0])/20 ;
  gyroADC[PITCH] =  ((rawADC_L3G4200D[3]<<8) | rawADC_L3G4200D[2])/20 ;
  gyroADC[YAW]   =  -((rawADC_L3G4200D[5]<<8) | rawADC_L3G4200D[4])/20 ;
}
#endif



// **************************
// I2C Gyroscope ITG3200 
// **************************
// I2C adress: 0xD2 (8bit)   0x69 (7bit)  // for sparkfun breakout board default jumper
// I2C adress: 0xD0 (8bit)   0x68 (7bit)  // for FreeFlight IMU board default jumper <- this one in the code
// principle:
// 1) VIO is connected to VDD
// 2) I2C adress is set to 0x69 (AD0 PIN connected to VDD)
// or 2) I2C adress is set to 0x68 (AD0 PIN connected to GND) <- this is the case for the code here
// 3) sample rate = 1000Hz ( 1kHz/(div+1) )
#if defined(ITG3200)
static uint8_t rawADC_ITG3200[6];

void i2c_Gyro_init() {
  delay(100);
  i2c_rep_start(0XD0+0);      // I2C write direction 
  i2c_write(0x3E);            // Power Management register
  i2c_write(0x80);            //   reset device

  delay(5);
  i2c_rep_start(0XD0+0);      // I2C write direction 
  i2c_write(0x15);            // register Sample Rate Divider
  i2c_write(0x7);             //   7: 8000Hz/(7+1) = 1000Hz . more than twice the need
  delay(5);
  i2c_rep_start(0XD0+0);      // I2C write direction 
  i2c_write(0x16);            // register DLPF_CFG - low pass filter configuration & sample rate
  i2c_write(0x18);            //   256Hz Low Pass Filter Bandwidth - Internal Sample Rate 8kHz
  delay(5);
  i2c_rep_start(0XD0+0);      // I2C write direction 
  i2c_write(0x3E);            // Power Management register
  i2c_write(0x03);            //   PLL with Z Gyro reference
  delay(100);
  gyroPresent = 1;  
}


void i2c_Gyro_getADC () {
  TWBR = ((16000000L / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz
  i2c_rep_start(0XD0);     // I2C write direction
  i2c_write(0X1D);         // Start multiple read
  i2c_rep_start(0XD0 +1);  // I2C read direction => 1
  for(uint8_t i = 0; i < 5; i++) {
    rawADC_ITG3200[i]=i2c_readAck();}
  rawADC_ITG3200[5]= i2c_readNak();

  gyroADC[ROLL]  = + ( ((rawADC_ITG3200[2]<<8) | rawADC_ITG3200[3])/3 );
  gyroADC[PITCH] = - ( ((rawADC_ITG3200[0]<<8) | rawADC_ITG3200[1])/3 );
  gyroADC[YAW]   = - ( ((rawADC_ITG3200[4]<<8) | rawADC_ITG3200[5])/3 );
}
#endif

// **************************
// I2C Compass HMC5843 
// **************************
// I2C adress: 0x3C (8bit)   0x1E (7bit)

#if defined(HMC5843)
static uint8_t rawADC_HMC5843[6];

void i2c_Mag_init() { 
  delay(100);
  i2c_rep_start(0X3C+0);      // I2C write direction 
  i2c_write(0x02);            // Write to Mode register
  i2c_write(0x00);            //   Continuous-Conversion Mode
  magPresent = 1;  
}

void i2c_Mag_getADC() {
  static uint32_t t;
  if ( (micros()-t )  < 100000 ) return; //each read is spaced by 100ms
  t = micros();
  TWBR = ((16000000L / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz
  i2c_rep_start(0X3C);     // I2C write direction
  i2c_write(0X03);         // Start multiple read
  i2c_rep_start(0X3C +1);  // I2C read direction => 1
  for(uint8_t i = 0; i < 5; i++) {
    rawADC_HMC5843[i]=i2c_readAck();}
  rawADC_HMC5843[5]= i2c_readNak();

  magADC[ROLL]  =   ((rawADC_HMC5843[0]<<8) | rawADC_HMC5843[1]);
  magADC[PITCH] =   ((rawADC_HMC5843[2]<<8) | rawADC_HMC5843[3]);
  magADC[YAW]   = - ((rawADC_HMC5843[4]<<8) | rawADC_HMC5843[5]);
}
#endif

// **************************
// I2C Compass HMC5883 
// **************************
// I2C adress: 0x3C (8bit)   0x1E (7bit)

#if defined(HMC5883)
static uint8_t rawADC_HMC5883[6];

void i2c_Mag_init() { 
  delay(100);
  i2c_rep_start(0X3C+0);      // I2C write direction 
  i2c_write(0x02);            // Write to Mode register
  i2c_write(0x00);            //   Continuous-Conversion Mode
  magPresent = 1;  
}

void i2c_Mag_getADC () {
  static uint32_t t;
  if ( (micros()-t )  < 100000 ) return; //each read is spaced by 100ms
  t = micros();
  TWBR = ((16000000L / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz
  i2c_rep_start(0X3C);     // I2C write direction
  i2c_write(0X03);         // Start multiple read
  i2c_rep_start(0X3C +1);  // I2C read direction => 1
  for(uint8_t i = 0; i < 5; i++) {
    rawADC_HMC5883[i]=i2c_readAck();}
  rawADC_HMC5883[5]= i2c_readNak();

  magADC[ROLL]  =   ((rawADC_HMC5883[4]<<8) | rawADC_HMC5883[5]) +300;
  magADC[PITCH] =  -((rawADC_HMC5883[0]<<8) | rawADC_HMC5883[1]) +200;
  magADC[YAW]   =  -((rawADC_HMC5883[2]<<8) | rawADC_HMC5883[3]) +50;
}
#endif

// **************************
// standalone I2C NUNCHUK
// **************************
#if defined(NUNCHACK)
static uint8_t rawADC_NUN[6];

void i2c_ACC_init() {
  i2c_rep_start(0xA4 + 0);//I2C write direction => 0
  i2c_write(0xF0); 
  i2c_write(0x55); 
  i2c_rep_start(0xA4 + 0);//I2C write direction => 0
  i2c_write(0xFB); 
  i2c_write(0x00); 
  delay(250);
  accPresent = 1;
}

void i2c_ACC_getADC() {
  TWBR = ((16000000L / 400000L) - 16) / 2; // change the I2C clock rate. !! you must check if the nunchuk is ok with this freq
  i2c_rep_start(0xA4 + 0);//I2C write direction => 0
  i2c_write(0x00);
  i2c_rep_start(0xA4 + 1);//I2C read direction => 1
  for(uint8_t i = 0; i < 5; i++)
    rawADC_NUN[i]=i2c_readAck();
  rawADC_NUN[5]= i2c_readNak();

  accADC[ROLL]  =   ( (rawADC_NUN[3]<<2)        + ((rawADC_NUN[5]>>4)&0x2) );
  accADC[PITCH] = - ( (rawADC_NUN[2]<<2)        + ((rawADC_NUN[5]>>3)&0x2) );
  accADC[YAW]   = - ( ((rawADC_NUN[4]&0xFE)<<2) + ((rawADC_NUN[5]>>5)&0x6) );
}
#endif


// **************************
// I2C Wii Motion Plus 
// **************************
// I2C adress 1: 0xA6 (8bit)    0x53 (7bit)
// I2C adress 2: 0xA4 (8bit)    0x52 (7bit)

static uint8_t rawADC_WMP[6];

void i2c_WMP_init(uint8_t d) {
  delay(d);
  i2c_rep_start(0xA6 + 0);//I2C write direction => 0
  i2c_write(0xF0); 
  i2c_write(0x55); 
  delay(d);
  i2c_rep_start(0xA6 + 0);//I2C write direction => 0
  i2c_write(0xFE); 
  i2c_write(0x05); 
  delay(d);
  if (d>0) {
    uint8_t numberAccRead = 0;
    for(uint8_t i=0;i<100;i++) {
      delay(3);
      if (rawIMU(0) == 0) numberAccRead++; // we detect here is nunchuk extension is available
    }
    if (numberAccRead>25)
      nunchukPresent = 1;
    delay(10);
  }
}

void i2c_WMP_getRawADC() {
  TWBR = ((16000000L / I2C_SPEED) - 16) / 2; // change the I2C clock rate
  i2c_rep_start(0xA4 + 0);//I2C write direction => 0
  i2c_write(0x00);
  i2c_rep_start(0xA4 + 1);//I2C read direction => 1
  for(uint8_t i = 0; i < 5; i++)
    rawADC_WMP[i]=i2c_readAck();
  rawADC_WMP[5]= i2c_readNak();
}

// **************************
// ADC ACC
// **************************
#if defined(ADCACC)
void adc_ACC_init(){
  pinMode(A1,INPUT);
  pinMode(A2,INPUT);
  pinMode(A3,INPUT);
}

void adc_ACC_getRawADC() {
  accADC[ROLL]  =  -analogRead(A1);
  accADC[PITCH] =  -analogRead(A2);
  accADC[YAW]   =  -analogRead(A3);

  acc_1G = 75;
  accPresent = 1;  
}
#endif

