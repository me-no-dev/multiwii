// Rx


#define RC_GOOD_BUCKET_MAX 20
#define RC_GOOD_RATIO 4

#define WidthOK(w) ((w>=900) && (w<=2200))

#if defined(SPEKTRUM)
#include <wiring.c>  //Auto-included by the Arduino core... but we need it sooner. 
#endif

#if defined(SBUS)
volatile uint16_t rcValue[RC_CHANS] = {
  1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502}; // interval [1000;2000]
#elif defined(SPEKTRUM)
volatile uint16_t rcValue[RC_CHANS] = {
  1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502}; // interval [1000;2000]
#elif defined(SERIAL_SUM_PPM)
volatile uint16_t rcValue[RC_CHANS]= {
  1502, 1502, 1502, 1502, 1502, 1502, 1502}; // interval [1000;2000]
#else
volatile uint16_t rcValue[RC_CHANS] = {
  1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502}; // interval [1000;2000]
#endif

#if defined(SERIAL_SUM_PPM) //Channel order for PPM SUM RX Configs
static uint8_t rcChannel[RC_CHANS] = {
  SERIAL_SUM_PPM};
#elif defined(SBUS) //Channel order for SBUS RX Configs
// for 16 + 2 Channels SBUS. The 10 extra channels 8->17 are not used by MultiWii, but it should be easy to integrate them.
static uint8_t rcChannel[RC_CHANS] = {
  PITCH,YAW,THROTTLE,ROLL,AUX1,AUX2,AUX3,AUX4,8,9,10,11,12,13,14,15,16,17};
static uint16_t sbusIndex=0;
#elif defined(SPEKTRUM)
static uint8_t rcChannel[RC_CHANS] = {
  PITCH,YAW,THROTTLE,ROLL,AUX1,AUX2,AUX3,AUX4,8,9,10,11};
#else // Standard Channel order
static uint8_t rcChannel[RC_CHANS]  = {
  ROLLPIN, PITCHPIN, YAWPIN, THROTTLEPIN, AUX1PIN,AUX2PIN,AUX3PIN,AUX4PIN};
static uint8_t PCInt_RX_Pins[PCINT_PIN_COUNT] = {
  PCINT_RX_BITS}; // if this slowes the PCINT readings we can switch to a define for each pcint bit
#endif

//_____________________________________________________________________________________________

// Configure Pins

void configureReceiver(void) {

#if defined(STANDARD_RX)
  // PCINT activation
  for(uint8_t i = 0; i < PCINT_PIN_COUNT; i++){ // i think a for loop is ok for the init.
    PCINT_RX_PORT |= PCInt_RX_Pins[i];
    PCINT_RX_MASK |= PCInt_RX_Pins[i];
  }
  PCICR = PCIR_PORT_BIT;

  //atmega32u4's Specific RX Pin Setup
  //Throttle on pin 7
  DDRE &= ~(1 << 6); // pin 7 to input
  PORTE |= (1 << 6); // enable pullups
  EIMSK |= (1 << INT6); // enable interuppt
  EICRB |= (1 << ISC60);
  // Aux2 pin on PBO (D17/RXLED)
#if defined(RCAUX2PIND17)
  DDRB &= ~(1 << 0); // set D17 to input 
#endif
  // Aux2 pin on PD2 (RX0)
#if defined(RCAUX2PINRXO)
  DDRD &= ~(1 << 2); // RX to input
  PORTD |= (1 << 2); // enable pullups
  EIMSK |= (1 << INT2); // enable interuppt
  EICRA |= (1 << ISC20);
#endif

#endif // STANDARD_RX

#if defined(SERIAL_SUM_PPM)
  PPM_PIN_INTERRUPT; 
#endif

#if defined (SPEKTRUM)
  SerialOpen(SPEK_SERIAL_PORT,115200);
#elif defined(SBUS)
  SerialOpen(1,100000);
#endif

} // configureReceiver

void failsafeUpdate(void) {

  rcNewValues = rcFrameOK;

  if( rcFrameOK) 
    failsafeCnt++;
  else {
    rcGlitches++;
    debug[0] = rcGlitches;
    failsafeCnt -= RC_GOOD_RATIO;
  }
  failsafeCnt = Limit1(failsafeCnt, RC_GOOD_BUCKET_MAX);
  inFailsafe = failsafeCnt <= 0;
  rcFrameOK = true;
} // failsafeUpdate

//_____________________________________________________________________________________________

// Standard Parallel PPM

#if defined(STANDARD_RX)

// predefined PC pin block (thanks to lianj)  - Version without failsafe
void inline rxPinCheck(uint8_t pin_pos, uint8_t rc_value_pos) {

  if (mask & PCInt_RX_Pins[pin_pos])                           
    if (!(pin & PCInt_RX_Pins[pin_pos])) 
    {
      if (pin_pos == 0) 
        failsafeUpdate();

      dTime = cTime-edgeTime[pin_pos]; 
      if (WidthOK(dTime)) 
        rcValue[rc_value_pos] = dTime;
      else 
        rcFrameOK &= false; 
    } 
    else 
      edgeTime[pin_pos] = cTime; 
} // rxPinCheck

// port change Interrupt
ISR(RX_PC_INTERRUPT) { //this ISR is common to every receiver channel, it is call everytime a change state occurs on a RX input pin
  uint8_t mask;
  uint8_t pin;
  uint16_t cTime,dTime;
  static uint16_t edgeTime[8];
  static uint8_t PCintLast;

  pin = RX_PCINT_PIN_PORT; // RX_PCINT_PIN_PORT indicates the state of each PIN for the arduino port dealing with Ports digital pins

  mask = pin ^ PCintLast;   // doing a ^ between the current interruption and the last one indicates which pin changed
  cTime = micros();         // micros() return a uint32_t, but it is not usefull to keep the whole bits => we keep only 16 bits
  sei();                    // re enable other interrupts at this point, the rest of this interrupt is not so time critical and can be interr
  pted safely
    PCintLast = pin;          // we memorize the current state of all PINs [D0-D7]

#if (PCINT_PIN_COUNT > 0)
  rxPinCheck(0,2);
#endif
#if (PCINT_PIN_COUNT > 1)
  rxPinCheck(1,4);
#endif
#if (PCINT_PIN_COUNT > 2)
  rxPinCheck(2,5);
#endif
#if (PCINT_PIN_COUNT > 3)
  rxPinCheck(3,6);
#endif
#if (PCINT_PIN_COUNT > 4)
  rxPinCheck(4,7);
#endif
#if (PCINT_PIN_COUNT > 5)
  rxPinCheck(5,0);
#endif
#if (PCINT_PIN_COUNT > 6)
  rxPinCheck(6,1);
#endif
#if (PCINT_PIN_COUNT > 7)
  rxPinCheck(7,3);
#endif

} // RX_PC_INTERRUPT

// atmega32u4's Throttle & Aux2 Pin

// throttle
ISR(INT6_vect){ 
  static uint16_t now,diff;
  static uint16_t last = 0;
  now = micros();  
  if(!(PINE & (1<<6))){
    diff = now - last;
    if(WidthOK(diff))
      rcValue[3] = diff;
    else 
      rcFrameOK &= false;
  }
  else last = now; 
}

// Aux 2
#if defined(RCAUX2PINRXO)
ISR(INT2_vect){
  static uint16_t now,diff;
  static uint16_t last = 0; 
  now = micros();  
  if(!(PIND & (1<<2))){
    diff = now - last;
    if(WidthOK(diff)) 
      rcValue[7] = diff;
    else 
      rcFrameOK &= false;
  }
  else last = now;
}
#endif  
#endif // STANDARD_RX

//_____________________________________________________________________________________________

// Compound PPM (CPPM)

#if defined(SERIAL_SUM_PPM)
ISR(INT6_vect){
  rxInt();
} // INT6_vect
#endif

// Compound PPM

#if defined(SERIAL_SUM_PPM)
void rxInt(void) {
  uint16_t now,diff;
  static uint16_t last = 0;
  static uint8_t chan = 0;

  now = micros();
  sei();
  diff = now - last;
  last = now;
  if( diff > 3000) { 
    if (chan >= 4) 
      failsafeUpdate();
  }
  else
    if (chan < RC_CHANS) { 
      if (WidthOK(diff))
        rcValue[chan] = diff;
      else 
        rcFrameOK &= false;
      chan++;
    }
} // rxInt
#endif

//_____________________________________________________________________________________________

// SBus

#if defined(SBUS)
void  readSBus(void){
#define SBUS_SYNCBYTE 0x0F // Not 100% sure: at the beginning of coding it was 0xF0 !!!
  static uint16_t sbus[25]={
    0                                                                                                                                                        };
  while(SerialAvailable(1)){
    int val = SerialRead(1);
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
      if ((sbus[23]) & 0x0001)       rcValue[16] = 2000; 
      else rcValue[16] = 1000;
      if ((sbus[23] >> 1) & 0x0001)  rcValue[17] = 2000; 
      else rcValue[17] = 1000;

      // Failsafe: there is one Bit in the SBUS-protocol (Byte 25, Bit 4) whitch is the failsafe-indicator-bit
      rcFrameOK = !((sbus[23] >> 3) & 0x0001);
      failsafeUpdate();
    }
  }        
} // readSBus
#endif // SBUS

//_____________________________________________________________________________________________

// Spektrum

#if defined(SPEKTRUM)
void readSpektrum(void) {
  uint32_t spekInterval;
  int16_t Temp;
  uint8_t i, b, bh, bl, spekChannel;

  if ((!f.ARMED) && ( SerialPeek(SPEK_SERIAL_PORT) == '$')) {
    while (SerialAvailable(SPEK_SERIAL_PORT)) {
      serialCom();
      delay (10);
    }
    return;
  } //End of: Is it the GUI?

  while (SerialAvailable(SPEK_SERIAL_PORT) > SPEK_FRAME_SIZE) // More than a frame?  More bytes implies we weren't called for multiple frame times.  
    // We do not want to process 'old' frames in the buffer.
    for (i = 0; i < SPEK_FRAME_SIZE; i++) 
      SerialRead(SPEK_SERIAL_PORT); //Toss one full frame of bytes.

  if (spekFrameFlags == 0x01) //The interrupt handler saw at least one valid frame start since we were last here. 
    if (SerialAvailable(SPEK_SERIAL_PORT) == SPEK_FRAME_SIZE) {  //A complete frame? If not, we'll catch it next time we are called. 
      SerialRead(SPEK_SERIAL_PORT); 
      SerialRead(SPEK_SERIAL_PORT);        //Eat the header bytes 
      for (b = 2; b < SPEK_FRAME_SIZE; b += 2) {
        bh = SerialRead(SPEK_SERIAL_PORT);
        bl = SerialRead(SPEK_SERIAL_PORT);
        spekChannel = 0x0F & (bh >> SPEK_CHAN_SHIFT);
        if (spekChannel < RC_CHANS) {
          Temp =  ((((uint16_t)(bh & SPEK_CHAN_MASK) << 8) + bl) SPEK_DATA_SHIFT);
#if defined(USE_MW_SPEKTRUM_SCALING) 
          rcValue[spekChannel] = Temp + 988; // widely used but does not give same width or midpoint as measured Rx output?
#else
          rcValue[spekChannel] = (Temp * 1.18) + 913; // direct measurement	
#endif
        }  
      }
      failsafeUpdate();
      spekFrameFlags = 0x00;
    } 
    else { //Start flag is on, but not enough bytes means there is an incomplete frame in buffer.  
      // This could be OK, if we happened to be calle in the middle of a frame.  Or not, if it has 
      // been a while since the start flag was set.
      spekInterval = micros() - spekTimeLast;
      if (spekInterval > 2500) {
        rcGlitches++;
        spekFrameFlags = 0; //If it has been a while, make the interrupt handler start over.
      } 
    }
} // readSpektrum
#endif // SPEKTRUM

//_____________________________________________________________________________________________

// General 

uint16_t readRawRC(uint8_t chan) {
  uint16_t data;
#if defined(SPEKTRUM)
  readSpektrum();
  if (chan < RC_CHANS) 
    data = rcValue[rcChannel[chan]];
  else data = MIDRC;
#else
  uint8_t oldSREG;
  oldSREG = SREG; 
  cli(); // disable interrupts
  data = rcValue[rcChannel[chan]]; // Let's copy the data Atomically
  SREG = oldSREG; // restore interrupt state
#endif
  return data; // We return the value correctly copied when the IRQ's where disabled
} // readRawRC

#if defined(USE_MW_RC_FILTER)

void computeRC(void) { // overkill?
  static uint16_t rcData4Values[RC_CHANS][4], rcDataMean[RC_CHANS];
  static uint8_t rc4ValuesIndex = 0;
  uint8_t chan,a;

  rc4ValuesIndex++;
  if (rc4ValuesIndex == 4) 
    rc4ValuesIndex = 0;

  for (chan = 0; chan < RC_CHANS; chan++) {
    rcData4Values[chan][rc4ValuesIndex] = readRawRC(chan);
    rcDataMean[chan] = 0;

    for (a=0;a<4;a++) 
      rcDataMean[chan] += rcData4Values[chan][a];

    rcDataMean[chan]= (rcDataMean[chan] + 2) >> 2;
    if ( rcDataMean[chan] < (uint16_t)rcData[chan] - 3)
      rcData[chan] = rcDataMean[chan] + 2;

    if ( rcDataMean[chan] > (uint16_t)rcData[chan] + 3)  
      rcData[chan] = rcDataMean[chan] - 2;
  }
} // computeRC

#else

void computeRC(void) {
  uint8_t chan;

#if defined(USE_RC_FILTER)

  static uint16_t rcDatap[RC_CHANS];
  static bool firstRC = true;

  for (chan = 0; chan < RC_CHANS; chan++) { // simple average
    rcData[chan] = readRawRC(chan);
    if (firstRC)
      rcDatap[chan] = rcData[chan];
    rcData[chan] = rcDatap[chan] = (rcData[chan] + rcDatap[chan])>>1; 
  }
  firstRC = false;

#else

    for (chan = 0; chan < RC_CHANS; chan++) 
    rcData[chan] = readRawRC(chan);

#endif // USE_RC_FILTER

} // computeRC

#endif // USE_MW_RC_FILTER

bool inline rxReady(void) {
  static uint32_t rcLastUpdateuS = 0;
  static uint32_t rcIntervaluS = 0;
  bool r;

#if defined(SPEKTRUM)
  readSpektrum();   
#elif defined(SBUS)
  readSBus();
#endif

  rcIntervaluS = micros() - rcLastUpdateuS;

  if (rcNewValues) {

    debug[1] = rcIntervaluS / 1000;

    rcLastUpdateuS = micros();
    rcNewValues = inFailsafe = false;
    r = true;
  } 
  else
    r = false;

  if (f.ARMED && (inFailsafe || (rcIntervaluS > (FAILSAFE_DELAY * 100000))))  {

    inFailsafe = true;
    debug[2] = inFailsafe;

#ifdef FAILSAFE
    rcCommand[ROLL] = rcCommand[PITCH] = AngleIntE[ROLL] = AngleIntE[PITCH] = 0;
    f.ANGLE_MODE = true;  
    f.HORIZON_MODE = false;

    if (rcIntervaluS > (FAILSAFE_OFF_DELAY * 100000)) {
      rcCommand[THROTTLE] = MINCOMMAND;
      f.OK_TO_ARM = false;
      go_disarm();
    } 
    else
      rcCommand[THROTTLE] = FAILSAFE_THROTTLE;   
#endif
  }

  return (r);
} // rxReady























































