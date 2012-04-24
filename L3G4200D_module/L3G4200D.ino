/********************************************************************/
/****                     L3G4200D config                        ****/
/********************************************************************/
void Gyro_init() {
  delay(100);
  i2c_writeReg(0XD2+0 ,0x20 ,0x8F ); // CTRL_REG1   400Hz ODR, 20hz filter, run!
  delay(5);
  i2c_writeReg(0XD2+0 ,0x24 ,0x02 ); // CTRL_REG5   low pass filter enable
}

void Gyro_getADC () {
  TWBR = ((16000000L / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz
  i2c_getSixRawADC(0XD2,0x80|0x28);

  GYRO_ORIENTATION( ((rawADC[1]<<8) | rawADC[0])/20  ,
                    ((rawADC[3]<<8) | rawADC[2])/20  ,
                    ((rawADC[5]<<8) | rawADC[4])/20  );
  GYRO_Common();
}
