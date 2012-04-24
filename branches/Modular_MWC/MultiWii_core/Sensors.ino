
// ************************************************************************************************************
// I2C general functions
// ************************************************************************************************************

void i2c_init(void) {
  #if defined(INTERNAL_I2C_PULLUPS)
    I2C_PULLUPS_ENABLE
  #else
    I2C_PULLUPS_DISABLE
  #endif
  TWSR = 0;                                    // no prescaler => prescaler = 1
  TWBR = ((16000000L / I2C_SPEED) - 16) / 2;   // change the I2C clock rate
  TWCR = 1<<TWEN;                              // enable twi module, no interrupt
}

void i2c_rep_start(uint8_t address) {
  TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN) ; // send REPEAT START condition
  waitTransmissionI2C();                       // wait until transmission completed
  TWDR = address;                              // send device address
  TWCR = (1<<TWINT) | (1<<TWEN);
  waitTransmissionI2C();                       // wail until transmission completed
}

void i2c_stop(void) {
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
  //  while(TWCR & (1<<TWSTO));                // <- can produce a blocking state with some WMP clones
}

void i2c_write(uint8_t data ) {	
  TWDR = data;                                 // send data to the previously addressed device
  TWCR = (1<<TWINT) | (1<<TWEN);
  waitTransmissionI2C();
}

uint8_t i2c_readAck() {
  TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
  waitTransmissionI2C();
  return TWDR;
}

uint8_t i2c_readNak(void) {
  TWCR = (1<<TWINT) | (1<<TWEN);
  waitTransmissionI2C();
  uint8_t r = TWDR;
  i2c_stop();
  return r;
}

void waitTransmissionI2C() {
  uint16_t count = 255;
  while (!(TWCR & (1<<TWINT))) {
    count--;
    if (count==0) {              //we are in a blocking state => we don't insist
      TWCR = 0;                  //and we force a reset on TWINT register
      neutralizeTime = micros(); //we take a timestamp here to neutralize the value during a short delay
      i2c_errors_count++;
      break;
    }
  }
}

void i2c_getSixRawADC(uint8_t add, uint8_t reg) {
  i2c_rep_start(add);
  i2c_write(reg);         // Start multiple read at the reg register
  i2c_rep_start(add +1);  // I2C read direction => I2C address + 1
  for(uint8_t i = 0; i < 5; i++)
    rawADC[i]=i2c_readAck();
  rawADC[5]= i2c_readNak();
}

void i2c_writeReg(uint8_t add, uint8_t reg, uint8_t val) {
  i2c_rep_start(add+0);  // I2C write direction
  i2c_write(reg);        // register selection
  i2c_write(val);        // value to write in register
  i2c_stop();
}

uint8_t i2c_readReg(uint8_t add, uint8_t reg) {
  i2c_rep_start(add+0);  // I2C write direction
  i2c_write(reg);        // register selection
  i2c_rep_start(add+1);  // I2C read direction
  return i2c_readNak();  // Read single register and return value
}

// ****************
// GYRO common part
// ****************
void GYRO_Common() {
  static int16_t previousGyroADC[3] = {0,0,0};
  static int32_t g[3];
  uint8_t axis;
  if (calibratingG>0) {
    for (axis = 0; axis < 3; axis++) {
      // Reset g[axis] at start of calibration
      if (calibratingG == 400) g[axis]=0;
      // Sum up 400 readings
      g[axis] +=gyroADC[axis];
      // Clear global variables for next reading
      gyroADC[axis]=0;
      gyroZero[axis]=0;
      if (calibratingG == 1) {
        gyroZero[axis]=g[axis]/400;
        blinkLED(10,15,1+3*nunchuk);
      }
    }
    calibratingG--;
  }
  for (axis = 0; axis < 3; axis++) {
    gyroADC[axis]  -= gyroZero[axis];
    //anti gyro glitch, limit the variation between two consecutive readings
    gyroADC[axis] = constrain(gyroADC[axis],previousGyroADC[axis]-800,previousGyroADC[axis]+800);   
    previousGyroADC[axis] = gyroADC[axis];
  }
}

// ****************
// ACC common part
// ****************
void ACC_Common() {
  static int32_t a[3];
  if (calibratingA>0) {
    for (uint8_t axis = 0; axis < 3; axis++) {
      // Reset a[axis] at start of calibration
      if (calibratingA == 400) a[axis]=0;
      // Sum up 400 readings
      a[axis] +=accADC[axis];
      // Clear global variables for next reading
      accADC[axis]=0;
      accZero[axis]=0;
    }
    // Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
    if (calibratingA == 1) {
      accZero[ROLL]  = a[ROLL]/400;
      accZero[PITCH] = a[PITCH]/400;
      accZero[YAW]   = a[YAW]/400-acc_1G; // for nunchuk 200=1G
      accTrim[ROLL]   = 0;
      accTrim[PITCH]  = 0;
      writeParams(1); // write accZero in EEPROM
    }
    calibratingA--;
  }
  accADC[ROLL]  -=  accZero[ROLL];
  accADC[PITCH] -=  accZero[PITCH];
  accADC[YAW]   -=  accZero[YAW];
}




void initSensors() {
  delay(200);
  POWERPIN_ON;
  delay(100);
  i2c_init();
  delay(100);
  #if defined(GYRO) && GYRO
    Gyro_init();
  #else
    WMP_init();
  #endif
  #if defined(BARO) && BARO
    Baro_init();
  #endif
  #if defined(MAG) && MAG
    Mag_init();
  #endif
  #if defined(ACC) && ACC
   ACC_init();acc_25deg = acc_1G * 0.423;
  #endif
  #if defined(SONAR) && SONAR
    Sonar_init();
  #endif
}


// i think this must stay here :P
// ___ felix ___

#if !GYRO 
  // ************************************************************************************************************
  // I2C Wii Motion Plus + optional Nunchuk
  // ************************************************************************************************************
  // I2C adress 1: 0xA6 (8bit)    0x53 (7bit)
  // I2C adress 2: 0xA4 (8bit)    0x52 (7bit)
  // ************************************************************************************************************
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] = X; gyroADC[PITCH] = Y; gyroADC[YAW] = Z;}
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = X; accADC[PITCH]  = Y; accADC[YAW]  = Z;}

  void WMP_init() {
    delay(250);
    i2c_writeReg(0xA6, 0xF0, 0x55); // Initialize Extension
    delay(250);
    i2c_writeReg(0xA6, 0xFE, 0x05); // Activate Nunchuck pass-through mode
    delay(250);
  
    // We need to set acc_1G for the Nunchuk beforehand; It's used in WMP_getRawADC() and ACC_Common()
    // If a different accelerometer is used, it will be overwritten by its ACC_init() later.
    acc_1G = 200;
    acc_25deg = acc_1G * 0.423;
    uint8_t numberAccRead = 0;
    // Read from WMP 100 times, this should return alternating WMP and Nunchuk data
    for(uint8_t i=0;i<100;i++) {
      delay(4);
      if (WMP_getRawADC() == 0) numberAccRead++; // Count number of times we read from the Nunchuk extension
    }
    // If we got at least 25 Nunchuck reads, we assume the Nunchuk is present
    if (numberAccRead>25)
      nunchuk = 1;
    delay(10);
  }
  
  uint8_t WMP_getRawADC() {
    uint8_t axis;
    TWBR = ((16000000L / I2C_SPEED) - 16) / 2; // change the I2C clock rate
    i2c_getSixRawADC(0xA4,0x00);
  
    if (micros() < (neutralizeTime + NEUTRALIZE_DELAY)) {//we neutralize data in case of blocking+hard reset state
      for (axis = 0; axis < 3; axis++) {gyroADC[axis]=0;accADC[axis]=0;}
      accADC[YAW] = acc_1G;
      return 1;
    } 
  
    // Wii Motion Plus Data
    if ( (rawADC[5]&0x03) == 0x02 ) {
      // Assemble 14bit data 
      gyroADC[ROLL]  = - ( ((rawADC[5]>>2)<<8) | rawADC[2] ); //range: +/- 8192
      gyroADC[PITCH] = - ( ((rawADC[4]>>2)<<8) | rawADC[1] );
      gyroADC[YAW]  =  - ( ((rawADC[3]>>2)<<8) | rawADC[0] );
      GYRO_Common();
      // Check if slow bit is set and normalize to fast mode range
      gyroADC[ROLL]  = (rawADC[3]&0x01)     ? gyroADC[ROLL]/5  : gyroADC[ROLL];  //the ratio 1/5 is not exactly the IDG600 or ISZ650 specification 
      gyroADC[PITCH] = (rawADC[4]&0x02)>>1  ? gyroADC[PITCH]/5 : gyroADC[PITCH]; //we detect here the slow of fast mode WMP gyros values (see wiibrew for more details)
      gyroADC[YAW]   = (rawADC[3]&0x02)>>1  ? gyroADC[YAW]/5   : gyroADC[YAW];   // this step must be done after zero compensation    
      return 1;
    } else if ( (rawADC[5]&0x03) == 0x00 ) { // Nunchuk Data
      ACC_ORIENTATION(  ( (rawADC[3]<<2)      | ((rawADC[5]>>4)&0x02) ) ,
                      - ( (rawADC[2]<<2)      | ((rawADC[5]>>3)&0x02) ) ,
                        ( ((rawADC[4]>>1)<<3) | ((rawADC[5]>>5)&0x06) ) );
      ACC_Common();
      return 0;
    } else
      return 2;
  }
#endif



