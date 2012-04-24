// ************************************************************************************************************
// I2C Accelerometer BMA180
// ************************************************************************************************************
// I2C adress: 0x80 (8bit)    0x40 (7bit) (SDO connection to VCC) 
// I2C adress: 0x82 (8bit)    0x41 (7bit) (SDO connection to VDDIO)
// Resolution: 14bit
//
// Control registers:
//
// 0x20    bw_tcs:      |                                           bw<3:0> |                        tcs<3:0> |
//                      |                                             150Hz |                        xxxxxxxx |
// 0x30    tco_z:       |                                                tco_z<5:0>    |     mode_config<1:0> |
//                      |                                                xxxxxxxxxx    |                   00 |
// 0x35    offset_lsb1: |          offset_x<3:0>              |                   range<2:0>       | smp_skip |
//                      |          xxxxxxxxxxxxx              |                    8G:   101       | xxxxxxxx |
// ************************************************************************************************************
void ACC_init () {
  delay(10);
  //default range 2G: 1G = 4096 unit.
  i2c_writeReg(BMA180_ADDRESS,0x0D,1<<4); // register: ctrl_reg0  -- value: set bit ee_w to 1 to enable writing
  delay(5);
  uint8_t control = i2c_readReg(BMA180_ADDRESS, 0x20);
  control = control & 0x0F;        // save tcs register
  control = control | (0x01 << 4); // register: bw_tcs reg: bits 4-7 to set bw -- value: set low pass filter to 20Hz
  i2c_writeReg(BMA180_ADDRESS, 0x20, control);
  delay(5);
  control = i2c_readReg(BMA180_ADDRESS, 0x30);
  control = control & 0xFC;        // save tco_z register
  control = control | 0x00;        // set mode_config to 0
  i2c_writeReg(BMA180_ADDRESS, 0x30, control);
  delay(5); 
  control = i2c_readReg(BMA180_ADDRESS, 0x35);
  control = control & 0xF1;        // save offset_x and smp_skip register
  control = control | (0x05 << 1); // set range to 8G
  i2c_writeReg(BMA180_ADDRESS, 0x35, control);
  delay(5); 
  acc_1G = 255;
}

void ACC_getADC () {
  TWBR = ((16000000L / 400000L) - 16) / 2;  // Optional line.  Sensor is good for it in the spec.
  i2c_getSixRawADC(BMA180_ADDRESS,0x02);
  //usefull info is on the 14 bits  [2-15] bits  /4 => [0-13] bits  /4 => 12 bit resolution
  ACC_ORIENTATION( ((rawADC[1]<<8) | rawADC[0])/16 ,
                   ((rawADC[3]<<8) | rawADC[2])/16 ,
                   ((rawADC[5]<<8) | rawADC[4])/16 );
  ACC_Common();
}
