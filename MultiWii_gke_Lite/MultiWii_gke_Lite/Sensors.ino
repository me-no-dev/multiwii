
// Default MPU6050 orientation

#if !defined(ACC_ORIENTATION) 
#define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = X; accADC[PITCH]  = Y; accADC[YAW]  = Z;}
#endif
#if !defined(GYRO_ORIENTATION) 
#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] = X; gyroADC[PITCH] = Y; gyroADC[YAW] = Z;}
#endif


#if !defined(MPU6050_ADDRESS)
#define MPU6050_ADDRESS     0x68 // address pin AD0 low (GND), default for FreeIMU v0.4 and InvenSense evaluation board
//#define MPU6050_ADDRESS     0x69 // address pin AD0 high (VCC)
#endif

// MPU6050 Gyro LPF setting
#if defined(MPU6050_LPF_256HZ) || defined(MPU6050_LPF_188HZ) || defined(MPU6050_LPF_98HZ) || defined(MPU6050_LPF_42HZ) || defined(MPU6050_LPF_20HZ) || defined(MPU6050_LPF_10HZ) || defined(MPU6050_LPF_5HZ)
#if defined(MPU6050_LPF_256HZ)
#define MPU6050_DLPF_CFG   0
#endif
#if defined(MPU6050_LPF_188HZ)
#define MPU6050_DLPF_CFG   1
#endif
#if defined(MPU6050_LPF_98HZ)
#define MPU6050_DLPF_CFG   2
#endif
#if defined(MPU6050_LPF_42HZ)
#define MPU6050_DLPF_CFG   3
#endif
#if defined(MPU6050_LPF_20HZ)
#define MPU6050_DLPF_CFG   4
#endif
#if defined(MPU6050_LPF_10HZ)
#define MPU6050_DLPF_CFG   5
#endif
#if defined(MPU6050_LPF_5HZ)
#define MPU6050_DLPF_CFG   6
#endif
#else
//Default settings LPF 98Hz/8000Hz sample
#define MPU6050_DLPF_CFG   2
#endif

void conditionGyro(void) {
  static int16_t gyroADCp[3] = {
    0,0,0
  };
  static int32_t g[3];
  uint8_t axis, tilt = 0;

  if (calibratingG > 0) {
    if (calibratingG == 512) {
      for (axis = 0; axis < 3; axis++)
        gyroZero[axis] = g[axis] = 0;
    } 

    for (axis = 0; axis < 3; axis++) {
      g[axis] += gyroADC[axis];

      if (calibratingG == 1) {
        gyroZero[axis]=g[axis] >> 9;
        blinkLED(10,15,1); //the delay causes to beep the buzzer really long 
      }

    }
    calibratingG--;
  }

  for (axis = 0; axis < 3; axis++) {
    gyroADC[axis]  -= gyroZero[axis];
    gyroADC[axis] = Limit1(gyroADC[axis], gyroADCp[axis] + 800); // slew limit    
    gyroADC[axis] =  gyroADCp[axis] = (gyroADC[axis] + gyroADCp[axis]) >> 1;   
    gyroData[axis] = gyroADC[axis]; // used in PID
  }
} // conditionGyro


void conditionAcc(void) {
  static int32_t a[3];
  uint8_t axis;

  if (calibratingA > 0) {

    if (calibratingA == 512) {
      for (axis = 0; axis < 3; axis++)
        conf.accTrim[axis] = a[axis] = 0;
      f.ACC_CALIBRATED = false;
    }  

    for (axis = 0; axis < 3; axis++) {
      a[axis] += accADC[axis];

      if (calibratingA == 1) {
        for (axis = 0; axis < 3; axis++) 
          global_conf.accZero[axis] = a[axis] >> 9;

        global_conf.accZero[YAW] -= acc_1G; 
        global_conf.accCalibrated = f.ACC_CALIBRATED = true; 
        writeGlobalSet(1);        
        writeParams(1);        
      }
    }
    calibratingA--;
  }

  accADC[ROLL]  -=  global_conf.accZero[ROLL] + conf.accTrim[ROLL];
  accADC[PITCH] -=  global_conf.accZero[PITCH] + conf.accTrim[PITCH];
  accADC[YAW]   -=  global_conf.accZero[YAW];

} // conditionAcc


void initMPU6050(void) {

  i2c_writeReg(MPU6050_ADDRESS, 0x6B, 0x80);             //PWR_MGMT_1    -- DEVICE_RESET 1
  delay(5);
  i2c_writeReg(MPU6050_ADDRESS, 0x6B, 0x03);             //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
  i2c_writeReg(MPU6050_ADDRESS, 0x1A, MPU6050_DLPF_CFG); //CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
  i2c_writeReg(MPU6050_ADDRESS, 0x1B, 0x18);             //GYRO_CONFIG   -- FS_SEL = 3: Full scale set to 2000 deg/sec

  i2c_writeReg(MPU6050_ADDRESS, 0x1C, 0x10); //ACCEL_CONFIG  -- AFS_SEL=2 (Full Scale = +/-8G)  ; ACCELL_HPF=0   //note something is wrong in the spec.
  //note: something seems to be wrong in the spec here. With AFS=2 1G = 4096 but according to my measurement: 1G=2048 (and 2048/8 = 256)
  //confirmed here: http://www.multiwii.com/forum/viewtopic.php?f=8&t=1080&start=10#p7480

    acc_1G = 512;
} // initMPU6050

void getRatesAndAccelerations(void) {
  uint8_t rawADC[14];

  i2c_read_reg_to_buf(MPU6050_ADDRESS, 0x3B, &rawADC, 14);

  ACC_ORIENTATION( ((rawADC[0]<<8) | rawADC[1])>>3 ,
  ((rawADC[2]<<8) | rawADC[3])>>3 ,
  ((rawADC[4]<<8) | rawADC[5])>>3 );
  conditionAcc();

  // 6 & 7 is temperature

  GYRO_ORIENTATION( ((rawADC[8]<<8) | rawADC[9])>>2 , // range: +/- 8192; +/- 2000 deg/sec
  ((rawADC[10]<<8) | rawADC[11])>>2 ,
  ((rawADC[12]<<8) | rawADC[13])>>2 );
  conditionGyro();

} // getRatesAndAccelerations

void initSensors(void) {
  delay(200);
  POWERPIN_ON;
  delay(100);
  i2c_init();
  delay(100);
  initMPU6050();
  acc_25deg = (float)acc_1G * 0.423;

  f.I2C_INIT_DONE = true;
} // initSensors









