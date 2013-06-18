
//----------------------------------------------------------------------------------------------------

// MW control routines replaced by those from UAVX by Prof Greg Egan aka gke 2013

//----------------------------------------------------------------------------------------------------

#ifndef USE_MW_CONTROL

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
    0,0,0                                                         };
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

  Scale = constrain(500 - (((int32_t)abs(rcCommand[YAW]) * conf.yawRate) >> 6), 0, 500);
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

int16_t doIntegral(uint8_t axis, bool negative) {
  int16_t I;

  I = -Limit1(((int32_t)angle[axis] * conf.I8[axis]) >> 5, 500); 
  // if (((I>0) && negative) || ((I<0) && !negative))
  //   I = 0;

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

    if (( f.ANGLE_MODE || f.HORIZON_MODE ) && ( maxRollPitchStick < ACROTRAINER_MODE )) {  
      I = doIntegral(axis, P > 0); 
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

 // RateIntE[YAW]  += gyroData[YAW]; 
 // RateIntE[YAW] = Limit1(RateIntE[YAW], 500);
 // I = (RateIntE[YAW] * conf.I8[YAW]) >> 7;
  I = 0;

  Temp = Limit1(rcCommand[YAW] - (P + I) , 500); 
  axisPID[YAW] = Limit1(Temp, abs(rcCommand[YAW]) + 100); //prevent "yaw jump" - slew limit?

} // computeControl

#endif // !USE_MW_CONTROL














