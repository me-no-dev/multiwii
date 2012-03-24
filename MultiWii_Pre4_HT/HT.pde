#ifdef HEADTRACKER 

//======================================================================== 
// Headtracker with Mwii
// PPM Encoder MultiWii to Trainer Port or Transmitter module
// For use with Arduino PRO mini Or Mega 
//
//========================================================================
// Patrik Emilsson 24/06/2011
// Version BETA 1.0
//========================================================================

//*********************************************************************
// TX Settings
//*********************************************************************
// PWM_pin  To Feed transmitter with 
#define PPM_Out   10   // Any PWM_pin

// PID settings
// Increase untill it feels natural.
int AXP[2]={5,4}; // Adjust effect of ACC  PAN,TILT to match headmovemoents
//int AXP[2]={P8[0],P8[1]}; // For use in GUI with P[ROLL] AND P[PITCH]



// TX settings
#define NumCannels  6  // Number of Channels To Emulate
#define PanChannel  1  // TX-Channel to injekt PAN
#define NickChannel 2  // TX-Channel to injekt TILT
//End of settings
//*********************************************************************
//*********************************************************************




//*********************************************************************
// Advanced settings for PPM-Frame
//********************************************************************* 
#define  pulseMin    750   // pulse minimum width minus start in uS
#define  pulseMax    1700  // pulse maximum width in uS
#define  Fixed_uS    300   // PPM frame fixed LOW phase
#define AdjustCenter  50   // Offset for PPM signals

int CH_uS[14];
static uint16_t Pulsemid;
static uint8_t PPM_Ok=0;
//End of Advanced settings
//*********************************************************************
//*********************************************************************


void setupHT() { 
  Pulsemid=MIDRC-AdjustCenter;
  PPM_Ok=0;
  for (int i = 0; i < NumCannels; i++) {CH_uS[i]=Pulsemid;}  
  pinMode(PPM_Out, OUTPUT);   // sets the digital pin as output
  // Setup timer
  TCCR1A = B00110001; // Compare register B used in mode '3'
  TCCR1B = B00010010; // WGM13 and CS11 set to 1
  TCCR1C = B00000000; // All set to 0
  TIMSK1 = B00000010; // Interrupt on compare B
  TIFR1  = B00000010; // Interrupt on compare B
  OCR1A = 22000; // 22mS PPM output refresh
  OCR1B = 1000;
}
//*********************************************************************

//*********************************************************************
//  Create the PPM signal to PPM_Out pin
//*********************************************************************
void ppmoutput() { 
  // ShiftUp
  for (int i = 0; i < NumCannels; i++) { //MIDRC
  int Pulse = map(CH_uS[i], 1020, 1980,  pulseMin, pulseMax);
  digitalWrite(PPM_Out, LOW);
  delayMicroseconds(Fixed_uS);    // Hold
  digitalWrite(PPM_Out, HIGH);
  delayMicroseconds(Pulse);  // Hold for CH_uS[0] microseconds  
}
  // Synchro pulse
  digitalWrite(PPM_Out, LOW);
  delayMicroseconds(Fixed_uS);    // Hold
  digitalWrite(PPM_Out, HIGH);  // Start Synchro pulse
  
  
 /* 
  // ShiftDown
    for (int i = 0; i < NumCannels; i++) { //MIDRC
  int Pulse = map(CH_uS[i], 1020, 1980,  pulseMin, pulseMax);
  digitalWrite(PPM_Out, HIGH);
  delayMicroseconds(Fixed_uS);    // Hold
  digitalWrite(PPM_Out, LOW);
  delayMicroseconds(Pulse);  // Hold for CH_uS[0] microseconds  
}
    // Synchro pulse
  digitalWrite(PPM_Out, HIGH);
  delayMicroseconds(Fixed_uS);    // Hold
  digitalWrite(PPM_Out, LOW);  // Start Synchro pulse
  */
}
//*********************************************************************



//*********************************************************************
// Map analogue inputs to PPM rates for each of the channels
//*********************************************************************
  // Main loop
void HTloop() { 
 for (int i = 0; i < NumCannels; i++) {
  if(i== PanChannel-1){CH_uS[i] = HT_Roll-AdjustCenter;} else if(i== NickChannel-1){CH_uS[i] = HT_Tilt-AdjustCenter;}else {CH_uS[i] = Pulsemid;
  }  
PPM_Ok=1;  } 
}
//*********************************************************************

//*********************************************************************
// Timer interupt
//*********************************************************************
ISR(TIMER1_COMPA_vect) {
  if (PPM_Ok) ppmoutput();
}
//*********************************************************************


#endif      
