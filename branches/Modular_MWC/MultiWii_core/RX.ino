#if !defined(SPECIAL_RX)
  static uint8_t PCInt_RX_Pins[PCINT_PIN_COUNT] = {PCINT_RX_BITS};
#endif
#if defined(SPECIAL_RX_PINLAYOUT)
  static uint8_t rcChannel[8]  = {SPECIAL_RX_PINLAYOUT};
#else
  static uint8_t rcChannel[8]  = {ROLLPIN, PITCHPIN, YAWPIN, THROTTLEPIN, AUX1PIN,AUX2PIN,AUX3PIN,AUX4PIN};
#endif

// Configure each rc pin for PCINT
void configureReceiver() {
  #if !defined(SPECIAL_RX)
    // PCINT activation
    for(uint8_t i = 0; i < PCINT_PIN_COUNT; i++){
      PCINT_RX_PORT |= PCInt_RX_Pins[i];
      PCINT_RX_MASK |= PCInt_RX_Pins[i];
    }
    PCICR = PCIR_PORT_BIT;
    #if defined(SPECIAL_PINS)
      init_special_pins();
    #endif
  #else
    init_special_RX();
  #endif
}

#if !defined(SPECIAL_RX)
  ISR(RX_PC_INTERRUPT){
    uint8_t mask;
    uint8_t pin;
    uint16_t cTime,dTime;
    static uint16_t edgeTime[8];
    static uint8_t PCintLast;
  
    pin = RX_PCINT_PIN_PORT;            

    mask = pin ^ PCintLast;   // doing a ^ between the current interruption and the last one indicates wich pin changed
    sei();                    // re enable other interrupts at this point, the rest of this interrupt is not so time critical and can be interrupted safely
    PCintLast = pin;          // we memorize the current state of all PINs [D0-D7]
  
    cTime = micros();         // micros() return a uint32_t, but it is not usefull to keep the whole bits => we keep only 16 bits
    #if (PCINT_PIN_COUNT > 0)
      if (mask & PCInt_RX_Pins[0]) 
        if (!(pin & PCInt_RX_Pins[0])) {
          dTime = cTime-edgeTime[0]; if (900<dTime && dTime<2200) rcValue[2] = dTime;
        } else edgeTime[0] = cTime;
    #endif
    #if (PCINT_PIN_COUNT > 1)
      if (mask & PCInt_RX_Pins[1]) 
        if (!(pin & PCInt_RX_Pins[1])) {
          dTime = cTime-edgeTime[1]; if (900<dTime && dTime<2200) rcValue[4] = dTime;
        } else edgeTime[1] = cTime; 
    #endif   
    #if (PCINT_PIN_COUNT > 2)
      if (mask & PCInt_RX_Pins[2]) 
        if (!(pin & PCInt_RX_Pins[2])) {
          dTime = cTime-edgeTime[2]; if (900<dTime && dTime<2200) rcValue[5] = dTime;
        } else edgeTime[2] = cTime; 
    #endif   
    #if (PCINT_PIN_COUNT > 3)
      if (mask & PCInt_RX_Pins[3]) 
        if (!(pin & PCInt_RX_Pins[3])) {
          dTime = cTime-edgeTime[3]; if (900<dTime && dTime<2200) rcValue[6] = dTime;
        } else edgeTime[3] = cTime; 
    #endif   
    #if (PCINT_PIN_COUNT > 4)
      if (mask & PCInt_RX_Pins[4]) 
        if (!(pin & PCInt_RX_Pins[4])) {
          dTime = cTime-edgeTime[4]; if (900<dTime && dTime<2200) rcValue[7] = dTime;
        } else edgeTime[4] = cTime; 
    #endif   
    #if (PCINT_PIN_COUNT > 5)
      if (mask & PCInt_RX_Pins[5]) 
        if (!(pin & PCInt_RX_Pins[5])) {
          dTime = cTime-edgeTime[5]; if (900<dTime && dTime<2200) rcValue[0] = dTime;
        } else edgeTime[5] = cTime; 
    #endif   
    #if (PCINT_PIN_COUNT > 6)
      if (mask & PCInt_RX_Pins[6]) 
        if (!(pin & PCInt_RX_Pins[6])) {
          dTime = cTime-edgeTime[6]; if (900<dTime && dTime<2200) rcValue[1] = dTime;
        } else edgeTime[6] = cTime; 
    #endif   
    #if (PCINT_PIN_COUNT > 7)
      if (mask & PCInt_RX_Pins[7]) 
        if (!(pin & PCInt_RX_Pins[7])) {
          dTime = cTime-edgeTime[7]; if (900<dTime && dTime<2200) rcValue[3] = dTime;
        } else edgeTime[7] = cTime; 
    #endif   
  }
#endif


#if defined(SERIAL_SUM_PPM)
  void init_special_RX(){
    PPM_PIN_INTERRUPT;  
  }
  
  void rxInt(){
    uint16_t now,diff;
    static uint16_t last = 0;
    static uint8_t chan = 0;
    now = micros();
    diff = now - last;
    last = now;
    if(diff>3000) chan = 0;
    else {
      if(900<diff && diff<2200 && chan<8 ) {   //Only if the signal is between these values it is valid, otherwise the failsafe counter should move up
        rcValue[chan] = diff;
        /*
        #if defined(FAILSAFE)
          if(failsafeCnt > 20) failsafeCnt -= 20; else failsafeCnt = 0;   // clear FailSafe counter - added by MIS  //incompatible to quadroppm
        #endif
        */
      }
    chan++;
    }
  }
#endif


uint16_t readRawRC(uint8_t chan) {
  uint16_t data;
  uint8_t oldSREG;
  oldSREG = SREG; cli(); // Let's disable interrupts
  data = rcValue[rcChannel[chan]]; // Let's copy the data Atomically
  #if defined(READ_SPECIAL_RX_WO_INT)
    data = read_Sp_RX_WO_int(data,chan);
  #endif
  SREG = oldSREG; sei();// Let's enable the interrupts
  #if defined(READ_SPECIAL_RX_W_INT)
    data = read_Sp_RX_W_int(data,chan);
  #endif
  return data; // We return the value correctly copied when the IRQ's where disabled
}


void computeRC() {
  static int16_t rcData4Values[8][4], rcDataMean[8];
  static uint8_t rc4ValuesIndex = 0;
  uint8_t chan,a;
    rc4ValuesIndex++;
    for (chan = 0; chan < 8; chan++) {
      rcData4Values[chan][rc4ValuesIndex%4] = readRawRC(chan);
      rcDataMean[chan] = 0;
      for (a=0;a<4;a++) rcDataMean[chan] += rcData4Values[chan][a];
      rcDataMean[chan]= (rcDataMean[chan]+2)/4;
      if ( rcDataMean[chan] < rcData[chan] -3)  rcData[chan] = rcDataMean[chan]+2;
      if ( rcDataMean[chan] > rcData[chan] +3)  rcData[chan] = rcDataMean[chan]-2;
    }
}
