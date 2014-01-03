// ************************************************************************************************************
// I2C Accelerometer BMA020
// ************************************************************************************************************
// I2C adress: 0x70 (8bit)
// Resolution: 10bit
// Control registers:
//
// Datasheet: After power on reset or soft reset it is recommended to set the SPI4-bit to the correct value.
//            0x80 = SPI four-wire = Default setting
// | 0x15: | SPI4 | enable_adv_INT | new_data_INT | latch_INT | shadow_dis | wake_up_pause<1:0> | wake_up |
// |       |    1 |              0 |            0 |         0 |          0 |                 00 |       0 |
//
// | 0x14: |                       reserved <2:0> |            range <1:0> |               bandwith <2:0> |
// |       |                      !!Calibration!! |                     2g |                         25Hz |
//
// ************************************************************************************************************
void ACC_init(){
  i2c_writeReg(0x70,0x15,0x80);    // set SPI4 bit
  uint8_t control = i2c_readReg(0x70, 0x14);
  control = control & 0xE0;        // save bits 7,6,5
  control = control | (0x02 << 3); // Range 8G (10)
  control = control | 0x00;        // Bandwidth 25 Hz 000
  i2c_writeReg(0x70,0x14,control); 
  acc_1G = 63;
}

void ACC_getADC(){
  TWBR = ((16000000L / 400000L) - 16) / 2;
  i2c_getSixRawADC(0x70,0x02);
  ACC_ORIENTATION( ((rawADC[1]<<8) | rawADC[0])/64 ,
                   ((rawADC[3]<<8) | rawADC[2])/64 ,
                   ((rawADC[5]<<8) | rawADC[4])/64 );
  ACC_Common();
}
