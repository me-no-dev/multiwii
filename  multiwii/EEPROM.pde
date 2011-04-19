void readEEPROM() {
  uint8_t i,p=1;
  for(i=0;i<3;i++) {P8[i] = EEPROM.read(p++);I8[i] = EEPROM.read(p++);D8[i] = EEPROM.read(p++);}
  PLEVEL8 = EEPROM.read(p++);ILEVEL8 = EEPROM.read(p++);
  rcRate8 = EEPROM.read(p++);rcExpo8 = EEPROM.read(p++);
  rollPitchRate = EEPROM.read(p++);
  yawRate = EEPROM.read(p++);
  dynThrPID = EEPROM.read(p++);
  activateAcc8 = EEPROM.read(p++);activateBaro8 = EEPROM.read(p++);activateMag8 = EEPROM.read(p++);
  activateCamStab8 = EEPROM.read(p++);activateCamTrig8 = EEPROM.read(p++);
  for(i=0;i<3;i++) accZero[i] = (EEPROM.read(p++)&0xff) + (EEPROM.read(p++)<<8);
  //note on the following lines: we do this calcul here because it's a static and redundant result and we don't want to load the critical loop whith it
  rcFactor1 = rcRate8/50.0*rcExpo8/100.0/250000.0;
  rcFactor2 = (100-rcExpo8)*rcRate8/5000.0;
}

void writeParams() {
  uint8_t i,p=1;
  EEPROM.write(0, checkNewConf);
  for(i=0;i<3;i++) {EEPROM.write(p++,P8[i]);  EEPROM.write(p++,I8[i]);  EEPROM.write(p++,D8[i]);}
  EEPROM.write(p++,PLEVEL8);EEPROM.write(p++,ILEVEL8);
  EEPROM.write(p++,rcRate8);EEPROM.write(p++,rcExpo8);
  EEPROM.write(p++,rollPitchRate);
  EEPROM.write(p++,yawRate);
  EEPROM.write(p++,dynThrPID);
  EEPROM.write(p++,activateAcc8);EEPROM.write(p++,activateBaro8);EEPROM.write(p++,activateMag8);
  EEPROM.write(p++,activateCamStab8);EEPROM.write(p++,activateCamTrig8);
  for(i=0;i<3;i++) {EEPROM.write(p++,accZero[i]);EEPROM.write(p++,accZero[i]>>8&0xff);}
  readEEPROM();
  blinkLED(15,20,1);
}

void checkFirstTime() {
  if ( EEPROM.read(0) != checkNewConf ) {
    P8[ROLL] = 40; I8[ROLL] = 30; D8[ROLL] = 15;
    P8[PITCH] = 40; I8[PITCH] = 30; D8[PITCH] = 15;
    P8[YAW]  = 80; I8[YAW]  = 0;  D8[YAW]  = 0;
    PLEVEL8 = 140; ILEVEL8 = 45;
    rcRate8 = 45;
    rcExpo8 = 65;
    rollPitchRate = 0;
    yawRate = 0;
    dynThrPID = 0;
    activateAcc8 = 0;activateBaro8 = 0;activateMag8 = 0;
    activateCamStab8 = 0;activateCamTrig8 = 0;
    writeParams();
  }
}

