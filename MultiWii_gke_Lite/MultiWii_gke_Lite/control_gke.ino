
//----------------------------------------------------------------------------------------------------

// MW control routines replaced by those from UAVX by Prof Greg Egan aka gke 2013

//----------------------------------------------------------------------------------------------------

int16_t Threshold(int16_t v, int16_t t) {

  if (v > t)
    v -= t;
  else if (v < -t)
    v += t;
  else
    v = 0;

  return (v);
} // Threshold

uint32_t SmoothRateDerivative(uint8_t axis, int16_t Rate) {
  // simple 3 sample moving average filter
  uint32_t d, deltaSum; 
  static int16_t Ratep[3] = {
    0,0,0                                             };
  static int16_t delta[2][3] = {
    {
      0,0,0    
    }
    ,{
      0,0,0 
    }
  };

  d = Rate - Ratep[axis];
  Ratep[axis] = Rate;

  deltaSum = delta[0][axis] + delta[1][axis] + d;
  delta[1][axis] = delta[0][axis];
  delta[0][axis] = d;

  return ((int32_t)deltaSum*conf.D8[axis]) >> 5;
} // SmoothDerivative  


void doRates(void) {
  uint8_t axis, index;
  int16_t Temp, Scale;

  for (axis = ROLL; axis <= YAW; axis++) 
    rcCommand[axis] = Threshold(Limit1(rcData[axis] - MIDRC, 500), DEADBAND); 

  maxRollPitchStick = min(max(abs(rcCommand[PITCH]), abs(rcCommand[ROLL])), 500);

  Scale = constrain(500 - (abs(rcCommand[YAW] * YAW_SCALE)), 0, 500);
  dynP8[YAW] = ((int32_t)conf.P8[YAW] * Scale) >> 9;

  //(uint16_t)conf.rollPitchRate
  //(uint16_t)conf.yawRate

#ifdef USE_THROTTLE_CURVE
  // division by 100 needs to go!
  Temp = constrain(rcData[THROTTLE], MINCHECK, 2000);
  Temp = ((uint32_t)(Temp - MINCHECK) * 1000) / (2000 - MINCHECK); 
  index = Temp / 100;
  rcCommand[THROTTLE] = lookupThrottleRC[index] + (Temp - index * 100) * (lookupThrottleRC[index+1] - lookupThrottleRC[index]) / 100;
#else
  rcCommand[THROTTLE] = constrain(rcData[THROTTLE], MINTHROTTLE, MAXTHROTTLE);
#endif // USE_THROTTLE_CURVE

} // DoRates

int16_t doIntegral(uint8_t axis) {
  int16_t I;

  if ( axis == YAW ) { 
    if (abs(gyroData[axis]) > 640) 
      RateIntE[axis] = 0;
    else {
      RateIntE[axis]  -= gyroData[axis]; 
      RateIntE[axis] = Limit1(RateIntE[axis], 16000);
    }
    I = ((RateIntE[axis] >> 7) * conf.I8[axis]) >> 6; 
  } 
  else
    I = Limit1(((int32_t)angle[axis] * conf.I8[axis]) >> 5, 500); 

  return (I);
} // doIntegral

void computeControl(void) {
  uint8_t axis;
  int16_t P,I,D;
  int16_t Temp;

  // From UAVX (Wolferl modified rate scheme)

  // Roll/Pitch
  for (axis = ROLL; axis < YAW; axis++) { 

    P =  -((int32_t)gyroData[axis] * conf.P8[axis]) >> 6;

    if ((maxRollPitchStick >= ACROTRAINER_MODE) || (abs(gyroData[axis]) > 640))// zero integral at high rotation rates - Acro
      I = 0;
    else
      if ( f.ANGLE_MODE || f.HORIZON_MODE ) {  
        I = -doIntegral(axis); 
        if ( f.HORIZON_MODE ) 
          I = ((int32_t)I * (512 - maxRollPitchStick)) >> 9;
      }
      else  
        I = 0;

    D = SmoothRateDerivative(axis, gyroData[axis]);

    axisPID[axis] = (P + I - D) + (rcCommand[axis] + GPS_angle[axis]);
  }

  // Yaw
  P = ((int32_t)gyroData[YAW] * dynP8[YAW]) >> 6;
  I = doIntegral(YAW); 

  Temp = Limit1(rcCommand[YAW] - (P + I) , 500); 
  axisPID[YAW] = Limit1(Temp, abs(rcCommand[YAW]) + 100); //prevent "yaw jump" - slew limit?

} // computeControl

//----------------------------------------------------------------------------------------------------

// Original MultiWii Control code for reference only - not flyable

void MWControl(void) {
  uint8_t axis;
  int16_t PTerm, ITerm, DTerm, PTermGYRO, ITermGYRO, error, errorAngle;
  static int16_t lastGyro[3] = {
    0,0,0            };
  static int32_t delta1[3] = {
    0,0,0            };
  static int32_t delta2[3] = {
    0,0,0              };
  int32_t delta, deltaSum, PTermACC, ITermACC;

  //**** PITCH & ROLL & YAW PID ****
  int16_t prop;
  prop = min(max(abs(rcCommand[PITCH]),abs(rcCommand[ROLL])),500); // range [0;500]

  for(axis=0;axis<3;axis++) {
    if ((f.ANGLE_MODE || f.HORIZON_MODE) && (axis != YAW) ) { // MODE relying on ACC
      // 50 degrees max inclination
      errorAngle = constrain((rcCommand[axis]<<1) + GPS_angle[axis],-500,+500);// - angle[axis] + conf.angleTrim[axis]; //16 bits is ok here
      PTermACC = ((int32_t)errorAngle*conf.P8[PIDLEVEL])>>7;  // 32 bits is needed for calculation: errorAngle*P8[PIDLEVEL] could exceed 32768   16 bits is ok for result
      PTermACC = constrain(PTermACC,-conf.D8[PIDLEVEL]*5,+conf.D8[PIDLEVEL]*5);

      AngleIntE[axis] = constrain(AngleIntE[axis]+errorAngle,-10000,+10000); // WindUp 16 bits is ok here
      ITermACC = ((int32_t)AngleIntE[axis]*conf.I8[PIDLEVEL])>>12; // 32 bits is needed for calculation:10000*I8 could exceed 32768   16 bits is ok for result
    }
    if ( !f.ANGLE_MODE || f.HORIZON_MODE || axis == 2 ) { // MODE relying on GYRO or YAW axis
      if (abs(rcCommand[axis])<500) error = (rcCommand[axis]<<6)/conf.P8[axis] ; // 16 bits is needed for calculation: 500*64 = 32000      16 bits is ok for result if P8>5 (P>0.5)
      else error = ((int32_t)rcCommand[axis]<<6)/conf.P8[axis] ; // 32 bits is needed for calculation

      error -= gyroData[axis];

      PTermGYRO = rcCommand[axis];

      RateIntE[axis]  = constrain(RateIntE[axis]+error,-16000,+16000); // WindUp   16 bits is ok here
      if (abs(gyroData[axis])>640) RateIntE[axis] = 0;
      ITermGYRO = ((RateIntE[axis]>>7)*conf.I8[axis])>>6;  // 16 bits is ok here 16000/125 = 128 ; 128*250 = 32000
    }
    if ( f.HORIZON_MODE && (axis != YAW)) {
      PTerm = ((int32_t)PTermACC*(512-prop) + (int32_t)PTermGYRO*prop)>>9; // the real factor should be 500, but 512 is ok
      ITerm = ((int32_t)ITermACC*(512-prop) + (int32_t)ITermGYRO*prop)>>9;
    } 
    else {
      if ( f.ANGLE_MODE && (axis != YAW)) {
        PTerm = PTermACC;
        ITerm = ITermACC;
      } 
      else {
        PTerm = PTermGYRO;
        ITerm = ITermGYRO;
      }
    }

    PTerm -= ((int32_t)gyroData[axis]*dynP8[axis])>>6; // 32 bits is needed for calculation   

    delta          = gyroData[axis] - lastGyro[axis];  // 16 bits is ok here, the diff between 2 consecutive gyro reads is limited to 800
    lastGyro[axis] = gyroData[axis];
    deltaSum       = delta1[axis]+delta2[axis]+delta;
    delta2[axis]   = delta1[axis];
    delta1[axis]   = delta;

    DTerm = ((int32_t)deltaSum*dynD8[axis])>>5;        // 32 bits is needed for calculation

    axisPID[axis] =  PTerm + ITerm - DTerm;
  }

} // MWControl











