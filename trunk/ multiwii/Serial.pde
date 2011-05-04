static uint8_t point;
static uint8_t s[128];
void serialize16(int16_t a) {s[point++]  = a; s[point++]  = a>>8&0xff;}
void serialize8(uint8_t a)  {s[point++]  = a;}

// ***********************************
// Interrupt driven UART transmitter for MIS_OSD
// ***********************************
static uint8_t tx_ptr;

ISR_UART {
  UDR0 = s[tx_ptr++];           /* Transmit next byte */
  if ( tx_ptr == point )        /* Check if all data is transmitted */
    UCSR0B &= ~(1<<UDRIE0);     /* Disable transmitter UDRE interrupt */
}

void UartSendData() {          // start of the data block transmission
  tx_ptr = 0;
  UCSR0A |= (1<<UDRE0);        /* Clear UDRE interrupt flag */
  UCSR0B |= (1<<UDRIE0);       /* Enable transmitter UDRE interrupt */
  UDR0 = s[tx_ptr++];          /* Start transmission */
}

void serialCom() {
  int16_t a;
  uint8_t i;
  if (Serial.available()) {
    switch (Serial.read()) {
    case 'A': //arduino to GUI all data
      point=0;
      serialize8('A');
      for(i=0;i<3;i++) serialize16(accSmooth[i]);
      for(i=0;i<3;i++) serialize16(gyroData[i]); //12
      serialize16(altitudeSmooth);
      serialize16(heading); // compass
      for(i=0;i<4;i++) serialize16(servo[i]); //24
      for(i=0;i<6;i++) serialize16(motor[i]); //36
      for(i=0;i<8;i++) serialize16(rcData[i]); //52
      serialize8(nunchukPresent|accPresent<<1|baroPresent<<2|magPresent<<3);
      serialize8(accMode|baroMode<<1|magMode<<2);
      serialize16(cycleTime);
      for(i=0;i<2;i++) serialize16(angle[i]/10); //60
    #if defined(TRI)
      serialize8(1);
    #elif defined(QUADP)
      serialize8(2);
    #elif defined(QUADX)
      serialize8(3);
    #elif defined(BI)
      serialize8(4);
    #elif defined(GIMBAL)
      serialize8(5);
    #elif defined(Y6)
      serialize8(6);
    #elif defined(HEX6)
      serialize8(7);
    #elif defined(FLYING_WING)
      serialize8(8);
    #elif defined(Y4)
      serialize8(9);
    #elif defined(HEX6X)
      serialize8(10);
    #elif defined(OCTOX8)
      serialize8(11);
    #endif
      for(i=0;i<3;i++) {serialize8(P8[i]);serialize8(I8[i]);serialize8(D8[i]);}//70
      serialize8(PLEVEL8);serialize8(ILEVEL8);
      serialize8(rcRate8); serialize8(rcExpo8);
      serialize8(rollPitchRate); serialize8(yawRate);
      serialize8(dynThrPID);
      serialize8(activateAcc8);serialize8(activateBaro8);serialize8(activateMag8);//80
      serialize8(activateCamStab8);serialize8(activateCamTrig8);//82
      serialize8(activateArm8);//83
      serialize8('A');
      UartSendData(); // Serial.write(s,point);
      break;
    case 'O':  // arduino to OSD data - contribution from MIS
      point=0;
      serialize8('O');
      for(i=0;i<3;i++) serialize16(accSmooth[i]);
      for(i=0;i<3;i++) serialize16(gyroData[i]);
      serialize16(altitudeSmooth);
      serialize16(heading); // compass - 16 bytes
      for(i=0;i<2;i++) serialize16(angle[i]); //20
      for(i=0;i<6;i++) serialize16(motor[i]); //32
      for(i=0;i<6;i++) {serialize16(rcData[i]);} //44
      serialize8(nunchukPresent|accPresent<<1|baroPresent<<2|magPresent<<3);
      serialize8(accMode|baroMode<<1|magMode<<2);
      serialize8(vbat);     // Vbatt 47
      serialize8(17);  // MultiWii Firmware version
      serialize8('O'); //49
      UartSendData();
      break;
    case 'C': //GUI to arduino param
      while (Serial.available()<22) {}
      for(i=0;i<3;i++) {P8[i]= Serial.read(); I8[i]= Serial.read(); D8[i]= Serial.read();}
      PLEVEL8 = Serial.read(); ILEVEL8 = Serial.read();
      rcRate8 = Serial.read(); rcExpo8 = Serial.read();
      rollPitchRate = Serial.read(); yawRate = Serial.read();
      dynThrPID = Serial.read();
      activateAcc8 = Serial.read();activateBaro8 = Serial.read();activateMag8 = Serial.read();
      activateCamStab8 = Serial.read();activateCamTrig8 = Serial.read();activateArm8 = Serial.read();//22
      writeParams();
      break;
    case 'D': //GUI to arduino ACC calibration request
      calibratingA=400;
      break;
    case 'E': //GUI to arduino MAG calibration request
      calibratingM=1;
      break;
    }
  }
}
