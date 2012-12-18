
#include "def.h"

uint8_t  rcOptions[CHECKBOXITEMS];
int16_t rcData[8];          // interval [1000;2000]
int16_t rcCommand[4];       // interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW
int16_t lookupPitchRollRC[6];// lookup table for expo & RC rate PITCH+ROLL
int16_t lookupThrottleRC[11];// lookup table for expo & mid THROTTLE
volatile uint8_t rcFrameComplete; // for serial rc receiver Spektrum

/**************************************************************************************/
/***************             Global RX related variables           ********************/
/**************************************************************************************/

//RAW RC values will be store here
#if defined(SBUS)
  volatile uint16_t rcValue[18] = {1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502}; // interval [1000;2000]
#else
  volatile uint16_t rcValue[8] = {1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502}; // interval [1000;2000]
#endif

#if defined(SERIAL_SUM_PPM) //Channel order for PPM SUM RX Configs
  static uint8_t rcChannel[8] = {SERIAL_SUM_PPM};
#elif defined(SBUS) //Channel order for SBUS RX Configs
  // for 16 + 2 Channels SBUS. The 10 extra channels 8->17 are not used by MultiWii, but it should be easy to integrate them.
  static uint8_t rcChannel[18] = {PITCH,YAW,THROTTLE,ROLL,AUX1,AUX2,AUX3,AUX4,8,9,10,11,12,13,14,15,16,17};
  static uint16_t sbusIndex=0;
#else // Standard Channel order
  static uint8_t rcChannel[8]  = {ROLLPIN, PITCHPIN, YAWPIN, THROTTLEPIN, AUX1PIN,AUX2PIN,AUX3PIN,AUX4PIN};
static struct {
	uint32_t RisingEdge, FallingEdge;
	uint8_t State;
	} rcInp[8] = {{0,0,true},{0,0,true},{0,0,true},{0,0,true},{0,0,true},{0,0,true},{0,0,true},{0,0,true}};
#endif
#if defined(SPEKTRUM) // Spektrum Satellite Frame size
  //Note: Defines are moved to def.h 
  volatile uint8_t spekFrame[SPEK_FRAME_SIZE];
#endif


/**************************************************************************************/
/***************                   RX Pin Setup                    ********************/
/**************************************************************************************/
void configureReceiver() {

  #if defined (SPEKTRUM)
    SerialOpen(1,SERIAL_COM_BAUD);
  #endif
  // Init SBUS RX
  #if defined(SBUS)
    SerialOpen(1,100000);
  #endif
}

/**************************************************************************************/
/***************               Standard RX Pins reading            ********************/
/**************************************************************************************/
#if defined(STANDARD_RX)

void RCParallelISR(TIM_TypeDef *tim) {

	uint8 chan;
	uint16 TimerVal = 0;
	int32 diff;
	TIMChannelDef * u;

	for (chan = 0; chan < 8; chan++) {
		u = &RCPins[chan].Timer;

		if (u->Tim == tim && (TIM_GetITStatus(tim, u->CC) == SET)) {
			TIM_ClearITPendingBit(u->Tim, u->CC);

			switch (u->Channel) {
				case TIM_Channel_1:
				TimerVal = TIM_GetCapture1(u->Tim);
				break;
				case TIM_Channel_2:
				TimerVal = TIM_GetCapture2(u->Tim);
				break;
				case TIM_Channel_3:
				TimerVal = TIM_GetCapture3(u->Tim);
				break;
				case TIM_Channel_4:
				TimerVal = TIM_GetCapture4(u->Tim);
				break;
			} // switch

			if (rcInp[chan].State) {
				rcInp[chan].FallingEdge = TimerVal;
				rcInp[chan].State = false;

				if (rcInp[chan].FallingEdge > rcInp[chan].RisingEdge)
				diff = rcInp[chan].FallingEdge - rcInp[chan].RisingEdge;
				else
				diff = (0xffff - rcInp[chan].RisingEdge) + rcInp[chan].FallingEdge;

				if (900<diff && diff<2200) {
					rcValue[chan] = diff;
#if defined(FAILSAFE)
					// clear FailSafe counter - added by MIS
					//incompatible to quadroppm
					if(failsafeCnt > 20) failsafeCnt -= 20; else failsafeCnt = 0;
#endif
				}

				TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
			} else {
				rcInp[chan].RisingEdge = TimerVal;
				rcInp[chan].State = true;
				TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
			}
			TIM_ICInitStructure.TIM_Channel = u->Channel;
			TIM_ICInit(u->Tim, &TIM_ICInitStructure);
		}
	}
}

#endif


/**************************************************************************************/
/***************                PPM SUM RX Pin reading             ********************/
/**************************************************************************************/

// Read PPM SUM RX Data
#if defined(SERIAL_SUM_PPM)

  void RCSerialISR(uint32 TimerVal) {
	int32 diff;
	static uint32_t last = 0;
	static uint8_t chan = 0;

	if (TimerVal < last)
		last -= (int32) 0x0000ffff;
	diff = (TimerVal - last);
	last = TimerVal;

	if (diff > 3000)
		chan = 0;
	else {
		if (900 < diff && diff < 2200 && chan < 8) {
			rcValue[chan] = diff;
#if defined(FAILSAFE)
			if(failsafeCnt > 20) failsafeCnt -= 20; else failsafeCnt = 0; // clear FailSafe counter - added by MIS  //incompatible to quadroppm
#endif
		}
		chan++;
	}
}
#endif

/**************************************************************************************/
/***************            Spektrum Satellite RX Data             ********************/
/**************************************************************************************/
#if defined(SPEKTRUM)

  void SpektrumSBusISR(uint8 s) {
  }
    uint32_t spekTime;
    static uint32_t spekTimeLast, spekTimeInterval;
    static uint8_t  spekFramePosition;
    spekTime=micros();
    spekTimeInterval = spekTime - spekTimeLast;
    spekTimeLast = spekTime;
    if (spekTimeInterval > 5000) spekFramePosition = 0;
    spekFrame[spekFramePosition] = RxChar(RCSerial);
    if (spekFramePosition == SPEK_FRAME_SIZE - 1) {
      rcFrameComplete = 1;
      #if defined(FAILSAFE)
        if(failsafeCnt > 20) failsafeCnt -= 20; else failsafeCnt = 0;   // clear FailSafe counter
      #endif
    } else {
      spekFramePosition++;
    }
  }
#endif

/**************************************************************************************/
/***************                   SBUS RX Data                    ********************/
/**************************************************************************************/
#if defined(SBUS)
void  readSBus(){
  #define SBUS_SYNCBYTE 0x0F // Not 100% sure: at the beginning of coding it was 0xF0 !!!
  static uint16_t sbus[25]={0};
  while(SerialAvailable(RCSerial)){
    int val = SerialRead(RCSerial);
    if(sbusIndex==0 && val != SBUS_SYNCBYTE)
      continue;
    sbus[sbusIndex++] = val;
    if(sbusIndex==25){
      sbusIndex=0;
      rcValue[0]  = ((sbus[1]|sbus[2]<< 8) & 0x07FF)/2+976; // Perhaps you may change the term "/2+976" -> center will be 1486
      rcValue[1]  = ((sbus[2]>>3|sbus[3]<<5) & 0x07FF)/2+976; 
      rcValue[2]  = ((sbus[3]>>6|sbus[4]<<2|sbus[5]<<10) & 0x07FF)/2+976; 
      rcValue[3]  = ((sbus[5]>>1|sbus[6]<<7) & 0x07FF)/2+976; 
      rcValue[4]  = ((sbus[6]>>4|sbus[7]<<4) & 0x07FF)/2+976; 
      rcValue[5]  = ((sbus[7]>>7|sbus[8]<<1|sbus[9]<<9) & 0x07FF)/2+976;
      rcValue[6]  = ((sbus[9]>>2|sbus[10]<<6) & 0x07FF)/2+976; 
      rcValue[7]  = ((sbus[10]>>5|sbus[11]<<3) & 0x07FF)/2+976; // & the other 8 + 2 channels if you need them
      //The following lines: If you need more than 8 channels, max 16 analog + 2 digital. Must comment the not needed channels!
      rcValue[8]  = ((sbus[12]|sbus[13]<< 8) & 0x07FF)/2+976; 
      rcValue[9]  = ((sbus[13]>>3|sbus[14]<<5) & 0x07FF)/2+976; 
      rcValue[10] = ((sbus[14]>>6|sbus[15]<<2|sbus[16]<<10) & 0x07FF)/2+976; 
      rcValue[11] = ((sbus[16]>>1|sbus[17]<<7) & 0x07FF)/2+976; 
      rcValue[12] = ((sbus[17]>>4|sbus[18]<<4) & 0x07FF)/2+976; 
      rcValue[13] = ((sbus[18]>>7|sbus[19]<<1|sbus[20]<<9) & 0x07FF)/2+976; 
      rcValue[14] = ((sbus[20]>>2|sbus[21]<<6) & 0x07FF)/2+976; 
      rcValue[15] = ((sbus[21]>>5|sbus[22]<<3) & 0x07FF)/2+976; 
      // now the two Digital-Channels
      if ((sbus[23]) & 0x0001)       rcValue[16] = 2000; else rcValue[16] = 1000;
      if ((sbus[23] >> 1) & 0x0001)  rcValue[17] = 2000; else rcValue[17] = 1000;

      // Failsafe: there is one Bit in the SBUS-protocol (Byte 25, Bit 4) whitch is the failsafe-indicator-bit
      #if defined(FAILSAFE)
      if (!((sbus[23] >> 3) & 0x0001))
        {if(failsafeCnt > 20) failsafeCnt -= 20; else failsafeCnt = 0;}   // clear FailSafe counter
      #endif
    }
  }        
}
#endif


/**************************************************************************************/
/***************          combine and sort the RX Datas            ********************/
/**************************************************************************************/
uint16_t readRawRC(uint8_t chan) {
  uint16_t data;
  data = rcValue[rcChannel[chan]]; // Let's copy the data Atomically
  #if defined(SPEKTRUM)
    static uint32_t spekChannelData[SPEK_MAX_CHANNEL];
    if (rcFrameComplete) {
      for (uint8_t b = 3; b < SPEK_FRAME_SIZE; b += 2) {
        uint8_t spekChannel = 0x0F & (spekFrame[b - 1] >> SPEK_CHAN_SHIFT);
        if (spekChannel < SPEK_MAX_CHANNEL) spekChannelData[spekChannel] = ((uint32_t)(spekFrame[b - 1] & SPEK_CHAN_MASK) << 8) + spekFrame[b];
      }
      rcFrameComplete = 0;
    }
  #endif

  #if defined(SPEKTRUM)
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
  #endif
  return data; // We return the value correctly copied when the IRQ's where disabled
}

/**************************************************************************************/
/***************          compute and Filter the RX data           ********************/
/**************************************************************************************/
void computeRC() {
  static int16_t rcData4Values[8][4], rcDataMean[8];
  static uint8_t rc4ValuesIndex = 0;
  uint16_t Temp;
  uint8_t chan,a;
  #if !(defined(RCSERIAL)) // don't know if this is right here
    #if defined(SBUS)
      readSBus();
    #endif
    rc4ValuesIndex++;

    for (chan = 0; chan < 8; chan++) {
      Temp = readRawRC(chan);
      #if defined (HARMONISE)
      if (chan == PITCH)
          Temp = -Temp + 3000;
      #endif
      rcData4Values[chan][rc4ValuesIndex%4] = Temp;
      rcDataMean[chan] = 0;
      for (a=0;a<4;a++) rcDataMean[chan] += rcData4Values[chan][a];
      rcDataMean[chan]= (rcDataMean[chan]+2)/4;
      if ( rcDataMean[chan] < rcData[chan] -3)  rcData[chan] = rcDataMean[chan]+2;
      if ( rcDataMean[chan] > rcData[chan] +3)  rcData[chan] = rcDataMean[chan]-2;
    }
  #endif
}


