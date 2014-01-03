  // ************************************************************************************************************
  // I2C Gyroscope and Accelerometer MPU6050
  // ************************************************************************************************************
  
  void Gyro_init() {
    TWBR = ((16000000L / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz
    i2c_writeReg(MPU6050_ADDRESS, 0x6B, 0x80);             //PWR_MGMT_1    -- DEVICE_RESET 1
    delay(5);
    i2c_writeReg(MPU6050_ADDRESS, 0x19, 0x00);             //SMPLRT_DIV    -- SMPLRT_DIV = 0  Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
    
    i2c_writeReg(MPU6050_ADDRESS, 0x6B, 0x03);             //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
    i2c_writeReg(MPU6050_ADDRESS, 0x1B, 0x18);             //GYRO_CONFIG   -- FS_SEL = 3: Full scale set to 2000 deg/sec
    // enable I2C bypass for AUX I2C
    #if defined(MAG)
      i2c_writeReg(MPU6050_ADDRESS, 0x6A, 0x00);             //USER_CTRL     -- DMP_EN=0 ; FIFO_EN=0 ; I2C_MST_EN=0 (I2C bypass mode) ; I2C_IF_DIS=0 ; FIFO_RESET=0 ; I2C_MST_RESET=0 ; SIG_COND_RESET=0
      i2c_writeReg(MPU6050_ADDRESS, 0x37, 0x02);             //INT_PIN_CFG   -- INT_LEVEL=0 ; INT_OPEN=0 ; LATCH_INT_EN=0 ; INT_RD_CLEAR=0 ; FSYNC_INT_LEVEL=0 ; FSYNC_INT_EN=0 ; I2C_BYPASS_EN=1 ; CLKOUT_EN=0
    #endif
    
    i2c_writeReg(MPU6050_ADDRESS, 0x1A, MPU6050_DLPF_CFG); //CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
    //debug2 = i2c_readReg(MPU6050_ADDRESS, 0x1A);
    
    //debug2 = MPU6050_DLPF_CFG;
  }
  
  void Gyro_getADC () {
    i2c_getSixRawADC(MPU6050_ADDRESS, 0x43);
    GYRO_ORIENTATION( ((rawADC[0]<<8) | rawADC[1])/4 , // range: +/- 8192; +/- 2000 deg/sec
  	            ((rawADC[2]<<8) | rawADC[3])/4 ,
  	            ((rawADC[4]<<8) | rawADC[5])/4 );
    GYRO_Common();
  }
  
  void ACC_init () {
    i2c_writeReg(MPU6050_ADDRESS, 0x1C, 0x10);             //ACCEL_CONFIG  -- AFS_SEL=2 (Full Scale = +/-8G)  ; ACCELL_HPF=0   //note something is wrong in the spec.
    //note: something seems to be wrong in the spec here. With AFS=2 1G = 4096 but according to my measurement: 1G=2048 (and 2048/8 = 256)
    //confirmed here: http://www.multiwii.com/forum/viewtopic.php?f=8&t=1080&start=10#p7480
    #if defined(MPU6050ES)
      acc_1G = 255;
    #else
      acc_1G = 512;
    #endif
  
    #if defined(MPU6050_EN_I2C_BYPASS)
      //at this stage, the MAG is configured via the original MAG init function in I2C bypass mode
      //now we configure MPU as a I2C Master device to handle the MAG via the I2C AUX port (done here for HMC5883)
      i2c_writeReg(MPU6050_ADDRESS, 0x6A, 0b00100000);       //USER_CTRL     -- DMP_EN=0 ; FIFO_EN=0 ; I2C_MST_EN=1 (I2C master mode) ; I2C_IF_DIS=0 ; FIFO_RESET=0 ; I2C_MST_RESET=0 ; SIG_COND_RESET=0
      i2c_writeReg(MPU6050_ADDRESS, 0x37, 0x00);             //INT_PIN_CFG   -- INT_LEVEL=0 ; INT_OPEN=0 ; LATCH_INT_EN=0 ; INT_RD_CLEAR=0 ; FSYNC_INT_LEVEL=0 ; FSYNC_INT_EN=0 ; I2C_BYPASS_EN=0 ; CLKOUT_EN=0
      i2c_writeReg(MPU6050_ADDRESS, 0x24, 0x0D);             //I2C_MST_CTRL  -- MULT_MST_EN=0 ; WAIT_FOR_ES=0 ; SLV_3_FIFO_EN=0 ; I2C_MST_P_NSR=0 ; I2C_MST_CLK=13 (I2C slave speed bus = 400kHz)
      i2c_writeReg(MPU6050_ADDRESS, 0x25, 0x80|(MAG_ADDRESS>>1));//I2C_SLV0_ADDR -- I2C_SLV4_RW=1 (read operation) ; I2C_SLV4_ADDR=MAG_ADDRESS
      i2c_writeReg(MPU6050_ADDRESS, 0x26, MAG_DATA_REGISTER);//I2C_SLV0_REG  -- 6 data bytes of MAG are stored in 6 registers. First register address is MAG_DATA_REGISTER
      i2c_writeReg(MPU6050_ADDRESS, 0x27, 0x86);             //I2C_SLV0_CTRL -- I2C_SLV0_EN=1 ; I2C_SLV0_BYTE_SW=0 ; I2C_SLV0_REG_DIS=0 ; I2C_SLV0_GRP=0 ; I2C_SLV0_LEN=3 (3x2 bytes)
    #endif
  }
  
  void ACC_getADC () {
    i2c_getSixRawADC(MPU6050_ADDRESS, 0x3B);
    ACC_ORIENTATION( ((rawADC[0]<<8) | rawADC[1])/8 ,
                     ((rawADC[2]<<8) | rawADC[3])/8 ,
                     ((rawADC[4]<<8) | rawADC[5])/8 );
    ACC_Common();
  }
  
  //The MAG acquisition function must be replaced because we now talk to the MPU device
    #if defined(MPU6050_EN_I2C_BYPASS)
      void Device_Mag_getADC() {
        i2c_getSixRawADC(MPU6050_ADDRESS, 0x49);               //0x49 is the first memory room for EXT_SENS_DATA
        #if defined(HMC5843)
          MAG_ORIENTATION( ((rawADC[0]<<8) | rawADC[1]) ,
                           ((rawADC[2]<<8) | rawADC[3]) ,
                           ((rawADC[4]<<8) | rawADC[5]) );
        #endif
        #if defined (HMC5883)  
          MAG_ORIENTATION( ((rawADC[0]<<8) | rawADC[1]) ,
                           ((rawADC[4]<<8) | rawADC[5]) ,
                           ((rawADC[2]<<8) | rawADC[3]) );
        #endif
        #if defined (AK8975)
          MAG_ORIENTATION( ((rawADC[1]<<8) | rawADC[0]) ,          
                           ((rawADC[3]<<8) | rawADC[2]) ,     
                           ((rawADC[5]<<8) | rawADC[4]) );
        #endif
      }
    #endif
