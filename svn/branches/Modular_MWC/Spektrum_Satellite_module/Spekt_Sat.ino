
/********************************************************************/
/****                    spektrum satellite RX                   ****/
/********************************************************************/


volatile uint8_t spekFrame[SPEK_FRAME_SIZE];
static uint32_t spekChannelData[SPEK_MAX_CHANNEL];

void init_special_RX(){
  SerialOpen(1,115200);
}

void Pre_computeRC(){
  if (rcFrameComplete) computeRC();
}

ISR(SPEK_SERIAL_VECT) {
  uint32_t spekTime;
  static uint32_t spekTimeLast, spekTimeInterval;
  static uint8_t  spekFramePosition;
  spekTime=micros();
  spekTimeInterval = spekTime - spekTimeLast;
  spekTimeLast = spekTime;
  if (spekTimeInterval > 5000) spekFramePosition = 0;
  spekFrame[spekFramePosition] = SPEK_DATA_REG;
  if (spekFramePosition == SPEK_FRAME_SIZE - 1) {
    rcFrameComplete = 1;
    #if defined(FAILSAFE)
      if(failsafeCnt > 20) failsafeCnt -= 20; else failsafeCnt = 0;   // clear FailSafe counter
    #endif
  } else {
    spekFramePosition++;
  }
}

uint16_t read_Sp_RX_WO_int(uint16_t data, uint8_t chan){
  if (rcFrameComplete) {
    for (uint8_t b = 3; b < SPEK_FRAME_SIZE; b += 2) {
      uint8_t spekChannel = 0x0F & (spekFrame[b - 1] >> SPEK_CHAN_SHIFT);
      if (spekChannel < SPEK_MAX_CHANNEL) spekChannelData[spekChannel] = ((uint32_t)(spekFrame[b - 1] & SPEK_CHAN_MASK) << 8) + spekFrame[b];
    }
    rcFrameComplete = 0;
  }  
  return 0;
}

uint16_t read_Sp_RX_W_int(uint16_t data, uint8_t chan){
  static uint8_t spekRcChannelMap[SPEK_MAX_CHANNEL] = {1,2,3,0,4,5,6};
  if (chan >= SPEK_MAX_CHANNEL) {
    data = 1500;
  } else {
    #if (SPEKTRUM == 1024)
      data = 988 + spekChannelData[spekRcChannelMap[chan]];          // 1024 mode
    #endif
    #if (SPEKTRUM == 2048)
      data = 988 + (spekChannelData[spekRcChannelMap[chan]] >> 1);   // 2048 mode
    #endif
  } 
  return data;
}
