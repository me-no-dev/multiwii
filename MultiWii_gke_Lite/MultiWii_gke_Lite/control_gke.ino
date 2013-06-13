
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
    0,0,0                       };
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
  int16_t Temp;

  for (axis = ROLL; axis <= YAW; axis++) 
    rcCommand[axis] = Threshold(Limit1(rcData[axis] - MIDRC, 500), DEADBAND); 
    
  maxRollPitchStick = min(max(abs(rcCommand[PITCH]), abs(rcCommand[ROLL])), 500);

  dynP8[YAW] = ((int32_t)conf.P8[YAW] * constrain(500 - (abs(rcCommand[YAW] * YAW_SCALE)), 0, 500)) >> 9;

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

void computeControl(void) {
  uint8_t axis;
  int16_t P,I,D;
  int16_t Temp;

  // From UAVX (Wolferl modified rate scheme)

  for (axis = ROLL; axis < YAW; axis++) { 

    P =  -((int32_t)gyroData[axis] * conf.P8[axis]) >> 6;

    if ((maxRollPitchStick >= ACROTRAINER_MODE) || (abs(gyroData[axis]) > 640))// zero integral at high rotation rates - Acro
      I = 0;
    else
      if ( f.ANGLE_MODE || f.HORIZON_MODE ) {  
        I = -Limit1(((int32_t)angle[axis] * conf.I8[axis]) >> 5, 500); 
        if ( f.HORIZON_MODE ) 
          I = ((int32_t)I * (512 - maxRollPitchStick)) >> 9;
      }
      else  
        I = 0;

    D = SmoothRateDerivative(axis, gyroData[axis]);

    axisPID[axis] = (P + I - D) + (rcCommand[axis] + GPS_angle[axis]);
  }

  Temp = Limit1(rcCommand[YAW] - (((int32_t)gyroData[YAW] * dynP8[YAW]) >> 6) , 500); 
  axisPID[YAW] = Limit1(Temp, abs(rcCommand[YAW]) + 100); //prevent "yaw jump"  - slew limit?

} // computeControl


