
#if defined(BI) || defined(TRI) || defined(SERVO_TILT) || defined(GIMBAL) || defined(FLYING_WING) || defined(CAMTRIG) || defined(VTOL_DUAL) || defined(VTOL_TAIL)
  #define SERVO
#endif

#if defined(GIMBAL) || defined(FLYING_WING)
  #define NUMBER_MOTOR 0
#elif defined(BI)
  #define NUMBER_MOTOR 2
#elif defined(TRI) || defined(VTOL_DUAL) || defined(VTOL_TAIL)
  #define NUMBER_MOTOR 3
#elif defined(QUADP) || defined(QUADX) || defined(Y4)
  #define NUMBER_MOTOR 4
#elif defined(Y6) || defined(HEX6) || defined(HEX6X)
  #define NUMBER_MOTOR 6
#elif defined(OCTOX8) || defined(OCTOFLATP) || defined(OCTOFLATX)
  #define NUMBER_MOTOR 8
#endif

uint8_t PWM_PIN[8] = {MOTOR_ORDER};
volatile uint16_t atomicServo[6] = {250,250,250,250,250,250}; //changed from uint8 to uint16 to allow for servo stretching
static uint8_t carry;

//for HEX Y6 and HEX6/HEX6X flat and for promini
volatile uint8_t atomicPWM_PIN5_lowState;
volatile uint8_t atomicPWM_PIN5_highState;
volatile uint8_t atomicPWM_PIN6_lowState;
volatile uint8_t atomicPWM_PIN6_highState;

void writeServos() {
  #if defined(SERVO)
    servoManipulation();
    for (uint8_t i=0; i<6; i++) {
      if (stretch & 1<<i) {
        atomicServo[i] = constrain((slowServo[i]-1000)/2,100,400);
        if (atomicServo[i] > 255) {
          carry |= 1<<i;
          atomicServo[i] -= 250; }
        else carry &= ~(1<<i); }
      else atomicServo[i] = (slowServo[i]-1000)/4;
    }
  #endif
}

void writeMotors() { // [1000;2000] => [125;250]
  #if defined(MEGA)
    for(uint8_t i=0;i<NUMBER_MOTOR;i++)
      analogWrite(PWM_PIN[i], motor[i]>>3);
  #else
    for(uint8_t i=0;i<min(NUMBER_MOTOR,4);i++)
      analogWrite(PWM_PIN[i], motor[i]>>3);
    #if (NUMBER_MOTOR == 6)
      atomicPWM_PIN5_highState = motor[5]/8;
      atomicPWM_PIN5_lowState = 255-atomicPWM_PIN5_highState;
      atomicPWM_PIN6_highState = motor[4]/8;
      atomicPWM_PIN6_lowState = 255-atomicPWM_PIN6_highState;
    #endif
  #endif
}

void writeAllMotors(int16_t mc) {   // Sends commands to all motors
  for (uint8_t i =0;i<NUMBER_MOTOR;i++)
    motor[i]=mc;
  writeMotors();
}

#if defined(LOG_VALUES) || (POWERMETER == 1)
void logMotorsPower() {
  uint32_t amp;
  /* true cubic function; when divided by vbat_max=126 (12.6V) for 3 cell battery this gives maximum value of ~ 1000 */
  const uint32_t amperes[16] =   {31, 246, 831, 1969, 3845, 6645, 10551, 15750, 22425, 30762, 40944, 53156, 67583, 84410, 103821, 126000 };

  if (vbat) { // by all means - must avoid division by zero 
    for (uint8_t i =0;i<NUMBER_MOTOR;i++) {
      amp = amperes[(motor[i] - 1000)>>6] / vbat; // range mapped from [1000:2000] => [0:1000]; then break that up into 16 ranges; lookup amp
      #ifdef LOG_VALUES
         pMeter[i]+= amp; // sum up over time the mapped ESC input 
      #endif
      #if (POWERMETER == 1)
         pMeter[PMOTOR_SUM]+= amp; // total sum over all motors
      #endif
    }
  }
}
#endif

void initOutput() {
  for(uint8_t i=0;i<NUMBER_MOTOR;i++)
    pinMode(PWM_PIN[i],OUTPUT);
  writeAllMotors(1000);
  delay(300);
  #if defined(SERVO)
    initializeServo();
  #elif (NUMBER_MOTOR == 6) && defined(PROMINI)
    initializeSoftPWM();
  #endif
}

#if defined(SERVO)
void initializeServo() {
  #if defined(TRI) || defined(VTOL_TAIL)
    DIGITAL_SERVO_TRI_PINMODE
  #endif
  #if defined(SERVO_TILT) || defined(GIMBAL) || defined(FLYING_WING) || defined(VTOL_DUAL) || defined(VTOL_TAIL)
    DIGITAL_TILT_ROLL_PINMODE
    DIGITAL_TILT_PITCH_PINMODE
  #endif
  #if defined(VTOL_DUAL) || defined(VTOL_TAIL)
    DIGITAL_TILT_RIGHT_PINMODE
  #endif
  #if defined(VTOL_DUAL)
    DIGITAL_TILT_LEFT_PINMODE
  #endif
  #if defined(CAMTRIG)
    DIGITAL_CAM_PINMODE
  #endif
  #if defined(BI)
    DIGITAL_SERVO_TRI_PINMODE
    DIGITAL_BI_LEFT_PINMODE
  #endif
  TCCR0A = 0; // normal counting mode
  TIMSK0 |= (1<<OCIE0A); // Enable CTC interrupt
}

void servoManipulation() {
  uint8_t i;
  for (i=0;i<6;i++) {
    if ((slow & 1<<i) && (servoSpeed[i] > 0)) {
      if (servo[i] > (slowServo[i] + servoSpeed[i])) slowServo[i] += servoSpeed[i];
      else if (servo[i] < (slowServo[i] - servoSpeed[i])) slowServo[i] -= servoSpeed[i]; }
    else slowServo[i] = servo[i]; }
}
// ****servo yaw with a 50Hz refresh rate****
// prescaler is set by default to 64 on Timer0
// Duemilanove : 16MHz / 64 => 4 us
// 256 steps = 1 counter cycle = 1024 us
// algorithm strategy:
// pulse high servo 0 -> if servo is stretched, do nothing for 500 us, otherwise do nothing for 1000 us -> do nothing for [0 to 2020] us -> pulse down servo 0
// pulse high servo 1 -> if servo is stretched, do nothing for 500 us, otherwise do nothing for 1000 us -> do nothing for [0 to 2020] us -> pulse down servo 1
// pulse high servo 2 -> if servo is stretched, do nothing for 500 us, otherwise do nothing for 1000 us -> do nothing for [0 to 2020] us -> pulse down servo 2
// pulse high servo 3 -> if servo is stretched, do nothing for 500 us, otherwise do nothing for 1000 us -> do nothing for [0 to 2020] us -> pulse down servo 3
// pulse high servo 4 -> if servo is stretched, do nothing for 500 us, otherwise do nothing for 1000 us -> do nothing for [0 to 2020] us -> pulse down servo 4
// pulse high servo 5 -> if servo is stretched, do nothing for 500 us, otherwise do nothing for 1000 us -> do nothing for [0 to 2020] us -> pulse down servo 5
// do nothing for 6 x 1000 us
ISR(TIMER0_COMPA_vect) {
  static uint8_t state = 0;
  static uint8_t count;
  if (state == 0) {
    //http://billgrundmann.wordpress.com/2009/03/03/to-use-or-not-use-writedigital/
    #if defined(TRI) || defined (BI) || defined(VTOL_TAIL)
      DIGITAL_SERVO_TRI_HIGH
    #endif
    if (stretch & 1<<0) OCR0A+= 125; // 500 us
    else OCR0A+= 250; // 1000 us
    state++ ;
  } 
  else if (state == 1) {
    if (carry & (1<<0)) OCR0A+= 250; // another 1000 us
    else OCR0A+= atomicServo[0]; // [0-1020] us
    state++;
  } 
  else if (state == 2) {
    if (carry & (1<<0)) OCR0A+= atomicServo[0]; // [0-1020] us
    else {
      #if defined(TRI) || defined (BI) || defined (VTOL_TAIL)
        DIGITAL_SERVO_TRI_LOW
      #endif
      #if defined(BI)
        DIGITAL_BI_LEFT_HIGH
      #endif
      #if defined(SERVO_TILT) || defined(GIMBAL) || defined(FLYING_WING) || defined(VTOL_DUAL) || defined(VTOL_TAIL)
        DIGITAL_TILT_PITCH_HIGH
      #endif
      if (stretch & 1<<1) OCR0A+= 125; // 500 us
      else OCR0A+= 250; // 1000 us
      state++; } // skip state 3
    state++;
  }
  else if (state == 3) {
    #if defined(TRI) || defined (BI) ||defined(VTOL_TAIL)
      DIGITAL_SERVO_TRI_LOW
    #endif
    #if defined(BI)
      DIGITAL_BI_LEFT_HIGH
    #endif
    #if defined(SERVO_TILT) || defined(GIMBAL) || defined(FLYING_WING) || defined(VTOL_DUAL) || defined(VTOL_TAIL)
      DIGITAL_TILT_PITCH_HIGH
    #endif
    if (stretch & 1<<1) OCR0A+= 125; // 500 us
    else OCR0A+= 250; // 1000 us
    state++;
  } 
  else if (state == 4) {
    if (carry & (1<<1)) OCR0A+= 250; // another 1000 us
    else OCR0A+= atomicServo[1]; // [0-1020] us
    state++;
  } 
  else if (state == 5) {
    if (carry & (1<<1)) OCR0A+= atomicServo[1]; // [0-1020] us
    else {
      #if defined(SERVO_TILT) || defined(GIMBAL) || defined(FLYING_WING) || defined(VTOL_DUAL) || defined(VTOL_TAIL)
        DIGITAL_TILT_PITCH_LOW
        DIGITAL_TILT_ROLL_HIGH
      #endif
      #if defined(BI)
        DIGITAL_BI_LEFT_LOW
      #endif
      if (stretch & 1<<2) OCR0A+= 125; // 500 us
      else OCR0A+= 250; // 1000 us
      state++; } // skip state 6
    state++;
  }
  else if (state == 6) {
    #if defined(SERVO_TILT) || defined(GIMBAL) || defined(FLYING_WING) || defined(VTOL_DUAL) || defined(VTOL_TAIL)
      DIGITAL_TILT_PITCH_LOW
      DIGITAL_TILT_ROLL_HIGH
    #endif
    #if defined(BI)
      DIGITAL_BI_LEFT_LOW
    #endif
    state++;
    if (stretch & 1<<2) OCR0A+= 125; // 500 us
    else OCR0A+= 250; // 1000 us
  } 
  else if (state == 7) {
    if (carry & (1<<2)) OCR0A+= 250; // another 1000 us
    else OCR0A+= atomicServo[2]; // [0-1020] us
    state++;
  } 
  else if (state == 8) {
    if (carry & (1<<2)) OCR0A+= atomicServo[2]; // [0-1020] us
    else {
      #if defined(SERVO_TILT) || defined(GIMBAL) || defined(FLYING_WING) || defined(VTOL_DUAL) || defined(VTOL_TAIL)
        DIGITAL_TILT_ROLL_LOW
      #endif
      #if defined(CAMTRIG)
        DIGITAL_CAM_HIGH
      #endif
      #if defined(VTOL_DUAL) || defined(VTOL_TAIL)
        DIGITAL_RETRACTS_HIGH
      #endif
      if (stretch & 1<<3) OCR0A+= 125; // 500 us
      else OCR0A+= 250; // 1000 us
      state++; } // skip state 9
    state++;
  }
  else if (state == 9) {
    #if defined(SERVO_TILT) || defined(GIMBAL) || defined(FLYING_WING) || defined(VTOL_DUAL) || defined(VTOL_TAIL)
      DIGITAL_TILT_ROLL_LOW
    #endif
    #if defined(CAMTRIG)
      DIGITAL_CAM_HIGH
    #endif
    #if defined(VTOL_DUAL) || defined(VTOL_TAIL)
      DIGITAL_RETRACTS_HIGH
    #endif
    state++;
    if (stretch & 1<<3) OCR0A+= 125; // 500 us
    else OCR0A+= 250; // 1000 us
  } 
  else if (state == 10) {
    if (carry & (1<<3)) OCR0A+= 250; // another 1000 us
    else OCR0A+= atomicServo[3]; // [0-1020] us
    state++;
  } 
  else if (state == 11) {
    if (carry & (1<<3)) OCR0A+= atomicServo[3]; // [0-1020] us
    else {
      #if defined(CAMTRIG)
        DIGITAL_CAM_LOW
      #endif
      #if defined(VTOL_DUAL) || defined(VTOL_TAIL)
        DIGITAL_RETRACTS_LOW
      #endif
      #if defined (VTOL_DUAL)
        DIGITAL_TILT_LEFT_HIGH
      #endif
      if (stretch & 1<<4) OCR0A+= 125; // 500 us
      else OCR0A+= 250; // 1000 us
      state++; } // skip state 12
    state++;
  }
  else if (state == 12) {
    #if defined(CAMTRIG)
      DIGITAL_CAM_LOW
    #endif
    #if defined(VTOL_DUAL) || defined(VTOL_TAIL)
      DIGITAL_RETRACTS_LOW
    #endif
    #if defined (VTOL_DUAL)
      DIGITAL_TILT_LEFT_HIGH
    #endif
    state++;
    if (stretch & 1<<4) OCR0A+= 125; // 500 us
    else OCR0A+= 250; // 1000 us
  } 
  else if (state == 13) {
    if (carry & (1<<4)) OCR0A+= 250; // another 1000 us
    else OCR0A+= atomicServo[4]; // [0-1020] us
    state++;
  } 
  else if (state == 14) {
    if (carry & (1<<4)) OCR0A+= atomicServo[4]; // [0-1020] us
    else {
      #if defined(VTOL_DUAL)
        DIGITAL_TILT_LEFT_LOW
        DIGITAL_TILT_RIGHT_HIGH
      #endif
      if (stretch & 1<<5) OCR0A+= 125; // 500 us
      else OCR0A+= 250; // 1000 us
      state++; } // skip state 15
    state++;
  } 
  else if (state == 15) {
    #if defined(VTOL_DUAL)
      DIGITAL_TILT_LEFT_LOW
      DIGITAL_TILT_RIGHT_HIGH
    #endif
    state++;
    if (stretch & 1<<5) OCR0A+= 125; // 500 us
    else OCR0A+= 250; // 1000 us
  } 
  else if (state == 16) {
    if (carry & (1<<5)) OCR0A+= 250; // another 1000 us
    else OCR0A+= atomicServo[5]; // [0-1020] us
    state++;
  } 
  else if (state == 17) {
    if (carry & (1<<5)) OCR0A+= atomicServo[5]; // [0-1020] us
    else {
      #if defined (VTOL_DUAL)
        DIGITAL_TILT_RIGHT_LOW
      #endif
      count = 6;
      state++; // skip state 18
      OCR0A+= 250; }// 1000 us
    state++;
  } 
  else if (state == 18) {
    #if defined (VTOL_DUAL)
      DIGITAL_TILT_RIGHT_LOW
    #endif
    count = 6; // 6 x 1000 us
    state++;
    OCR0A+= 250; // 1000 us
  } 
  else if (state == 19) {
    if (count > 0) count--;
    else state = 0;
    OCR0A+= 250;
  }
}
#endif

#if (NUMBER_MOTOR == 6) && defined(PROMINI)
void initializeSoftPWM() {
  TCCR0A = 0; // normal counting mode
  TIMSK0 |= (1<<OCIE0A); // Enable CTC interrupt
  TIMSK0 |= (1<<OCIE0B);
}

ISR(TIMER0_COMPA_vect) {
  static uint8_t state = 0;
  if (state == 0) {
    PORTD |= 1<<5; //digital PIN 5 high
    OCR0A+= atomicPWM_PIN5_highState; //250 x 4 microsecons = 1ms
    state = 1;
  } else if (state == 1) {
    OCR0A+= atomicPWM_PIN5_highState;
    state = 2;
  } else if (state == 2) {
    PORTD &= ~(1<<5); //digital PIN 5 low
    OCR0A+= atomicPWM_PIN5_lowState;
    state = 0;
  }
}

ISR(TIMER0_COMPB_vect) { //the same with digital PIN 6 and OCR0B counter
  static uint8_t state = 0;
  if (state == 0) {
    PORTD |= 1<<6;OCR0B+= atomicPWM_PIN6_highState;state = 1;
  } else if (state == 1) {
    OCR0B+= atomicPWM_PIN6_highState;state = 2;
  } else if (state == 2) {
    PORTD &= ~(1<<6);OCR0B+= atomicPWM_PIN6_lowState;state = 0;
  }
}
#endif


void mixTable() {
  int16_t maxMotor,a;
  uint8_t i,axis;

  #define PIDMIX(X,Y,Z) rcCommand[THROTTLE] + axisPID[ROLL]*X + axisPID[PITCH]*Y + YAW_DIRECTION * axisPID[YAW]*Z

  #if NUMBER_MOTOR > 3
    //prevent "yaw jump" during yaw correction
    axisPID[YAW] = constrain(axisPID[YAW],-100-abs(rcCommand[YAW]),+100+abs(rcCommand[YAW]));
  #endif
  #ifdef BI
    motor[0] = PIDMIX(+1, 0, 0); //LEFT
    motor[1] = PIDMIX(-1, 0, 0); //RIGHT        
    servo[0]  = constrain(1500 + YAW_DIRECTION * (axisPID[YAW] + axisPID[PITCH]), 1020, 2000); //LEFT
    servo[1]  = constrain(1500 + YAW_DIRECTION * (axisPID[YAW] - axisPID[PITCH]), 1020, 2000); //RIGHT
  #endif
  #ifdef TRI
    motor[0] = PIDMIX( 0,+4/3, 0); //REAR
    motor[1] = PIDMIX(-1,-2/3, 0); //RIGHT
    motor[2] = PIDMIX(+1,-2/3, 0); //LEFT
    servo[0] = constrain(TRI_YAW_MIDDLE + YAW_DIRECTION * axisPID[YAW], TRI_YAW_CONSTRAINT_MIN, TRI_YAW_CONSTRAINT_MAX); //REAR
  #endif
  #ifdef VTOL_TAIL
    if (abs(rcData[AUX2] - lastRcData[AUX2]) > 40) { // transition detection - increase number to decrease sensitivity
      if (rcData[AUX2]>1600) { // transition to forward flight
        motor[0] = PIDMIX( 0,+4/3, 0); //REAR
        motor[1] = PIDMIX(-1,-2/3, 0); //RIGHT
        motor[2] = PIDMIX(+1,-2/3, 0); //LEFT
        servo[0]  = constrain(TRI_YAW_MIDDLE + YAW_DIRECTION * axisPID[YAW], TRI_YAW_CONSTRAINT_MIN, TRI_YAW_CONSTRAINT_MAX); //REAR
        servo[1]  = constrain(1500 - axisPID[PITCH] + axisPID[ROLL], 1020, 2000); //LEFT the direction of the 2 servo can be changed here: invert the sign before axisPID
        servo[2]  = constrain(1500 + axisPID[PITCH] + axisPID[ROLL], 1020, 2000); //RIGHT
        if (modeState2 == 0 && servo[5] < (rcData[AUX2] + axisPID[PITCH]/5 - 5)) {
          servo[5] += 1; // slowly tilt motors forward every other cycle
          modeState2++; }
        else if (modeState2 == 1) modeState2 = 0;
        else lastRcData[AUX2] = rcData[AUX2]; } // transition complete, update for next cycle!
      else if (rcData[AUX2]>1300) { // transition to fast hover
        motor[0] = PIDMIX( 0,+4/3, 0); //REAR
        motor[1] = PIDMIX(-1,-2/3, 0); //RIGHT
        motor[2] = PIDMIX(+1,-2/3, 0); //LEFT
        servo[0]  = constrain(TRI_YAW_MIDDLE + YAW_DIRECTION * axisPID[YAW], TRI_YAW_CONSTRAINT_MIN, TRI_YAW_CONSTRAINT_MAX); //REAR
        if (servo[1] < (constrain(1500 - axisPID[PITCH] + axisPID[ROLL], 1020, 2000)) && modeState == 0)
          servo[1] += 1; //LEFT the direction of the 2 servo can be changed here: invert the sign before axisPID
        else {
          servo[1] = constrain(1500 - axisPID[PITCH] + axisPID[ROLL], 1020, 2000);
          modeState = 1; }
        if (servo[2] > (constrain(1500 + axisPID[PITCH] + axisPID[ROLL], 1020, 2000)) && modeState == 0)
          servo[2] -= 1; //RIGHT
        else {
          servo[2] = constrain(1500 + axisPID[PITCH] + axisPID[ROLL], 1020, 2000);
          modeState = 1; }
        if (modeState2 == 0 && servo[5] < (rcData[AUX2] + axisPID[PITCH]/5 - 5)) {
          servo[5] += 1; // slowly tilt motors forward every other cycle
          modeState2++; }
        else if (modeState2 == 0 && servo[5] > (rcData[AUX2] + axisPID[PITCH]/5 + 5)) {
          servo[5] -= 1; // slowly tilt motors backward every other cycle
          modeState2++; }
        else if (modeState2 == 1) modeState2 = 0;
        else lastRcData[AUX2] = rcData[AUX2]; } // transition complete, update for next cycle!
      else { // transition to hover
        motor[0] = PIDMIX( 0,+4/3, 0); //REAR
        motor[1] = PIDMIX(-1,-1/3, 0); //RIGHT
        motor[2] = PIDMIX(+1,-1/3, 0); //LEFT
        servo[0]  = constrain(1500 + YAW_DIRECTION * axisPID[YAW], 1020, 2000); //REAR
        servo[1]  = constrain(1500 - axisPID[PITCH] + axisPID[ROLL], 1020, 2000); //LEFT the direction of the 2 servo can be changed here: invert the sign before axisPID
        servo[2]  = constrain(1500 + axisPID[PITCH] + axisPID[ROLL], 1020, 2000); //RIGHT
        if (modeState2 == 0 && servo[5] > (rcData[AUX2] + axisPID[PITCH]/5 + 5)) {
          servo[5] -= 1;
          modeState2++; } // slowly tilt motors backward
        else if (modeState2 == 1) modeState2 = 0;
        else lastRcData[AUX2] = rcData[AUX2]; } } // transition complete, update for next cycle!
    else { // not in any transitional state!
      if (rcData[AUX2]>1600) { // forward flight
        if (modeState == 1) modeState = 0; // this prepares for next transition to forward flight
        motor[0] = MINCOMMAND ;                                                   //REAR
        motor[1] = rcCommand[THROTTLE] - YAW_DIRECTION * rcCommand[YAW]/10 ;      //RIGHT
        motor[2] = rcCommand[THROTTLE] + YAW_DIRECTION * rcCommand[YAW]/10 ;      //LEFT
        servo[0]  = 1500; // tail servo centered in forward flight
        servo[1]  = constrain(1500 - axisPID[PITCH] + axisPID[ROLL], 1020, 2000); //LEFT the direction of the 2 servo can be changed here: invert the sign before axisPID
        servo[2]  = constrain(1500 + axisPID[PITCH] + axisPID[ROLL], 1020, 2000); //RIGHT
        servo[5]  = constrain(rcData[AUX2] + rcCommand[PITCH]/5, 1020, 2000); }   //front motor tilt (regular speed)
      else if (rcData[AUX2]>1300) { // fast hover
        if (modeState == 1) modeState = 0; // this prepares for next transition to fast hover
        motor[0] = PIDMIX( 0,+4/3, 0); //REAR
        motor[1] = PIDMIX(-1,-1/3, 0); //RIGHT
        motor[2] = PIDMIX(+1,-1/3, 0); //LEFT
        servo[0]  = constrain(1500 + YAW_DIRECTION * axisPID[YAW], 1020, 2000);   //REAR
        servo[1]  = constrain(1500 - axisPID[PITCH] + axisPID[ROLL], 1020, 2000); //LEFT the direction of the 2 servo can be changed here: invert the sign before axisPID
        servo[2]  = constrain(1500 + axisPID[PITCH] + axisPID[ROLL], 1020, 2000); //RIGHT
        servo[5]  = constrain(rcData[AUX2] + rcCommand[PITCH]/5, 1020, 2000); }   //front motor tilt (regular speed)
      else { // hover
        motor[0] = PIDMIX( 0,+4/3, 0); //REAR
        motor[1] = PIDMIX(-1,-2/3, 0); //RIGHT
        motor[2] = PIDMIX(+1,-2/3, 0); //LEFT
        servo[0]  = constrain(1500 + YAW_DIRECTION * axisPID[YAW], 1020, 2000); //REAR
        if (modeState2 == 0) { // slowly tilt flaps down
          servo[1]  = constrain(servo[1] - 1, 1020, 2000);   //LEFT the direction of the 2 servo can be changed here: invert the sign before axisPID
          servo[2]  = constrain(servo[2] + 1, 1020, 2000);   //RIGHT
          modeState2++; }
        else if (modeState2 == 1) modeState2 = 0;
        servo[5]  = constrain(rcData[AUX2] + rcCommand[PITCH]/5, 1020, 2000); } } //front motor tilt (regular speed)
  #endif
  #ifdef VTOL_DUAL 
    if (abs(rcData[AUX2] - lastRcData[AUX2]) > 40) { // transition detection - increase number to decrease sensitivity
      if (rcData[AUX2]>1600) { // transition to forward flight
        motor[0] = PIDMIX( 0,+4/3, 0); //REAR
        motor[1] = PIDMIX(-1,-2/3, 0); //RIGHT
        motor[2] = PIDMIX(+1,-2/3, 0); //LEFT
        servo[1]  = constrain(1500 - axisPID[PITCH] - axisPID[ROLL], 1020, 2000); //LEFT the direction of the 2 servo can be changed here: invert the sign before axisPID
        servo[2]  = constrain(1500 + axisPID[PITCH] - axisPID[ROLL], 1020, 2000); //RIGHT
        if (modeState2 == 0 && servo[5] < (rcData[AUX2] + axisPID[PITCH]/5 - 5)) {
          servo[5] += 1; // slowly tilt motors forward every other cycle
          servo[4] -= 1;
          modeState2++; }
        else if (modeState2 == 1) modeState2 = 0;
        else lastRcData[AUX2] = rcData[AUX2]; } // transition complete, update for next cycle!
      else if (rcData[AUX2]>1300) { // transition to fast hover
        motor[0] = PIDMIX( 0,+4/3, 0); //REAR
        motor[1] = PIDMIX(-1,-2/3, 0); //RIGHT
        motor[2] = PIDMIX(+1,-2/3, 0); //LEFT
        if (servo[1] < (constrain(1500 - axisPID[PITCH] - axisPID[ROLL], 1020, 2000)) && modeState == 0)
          servo[1] += 1; //LEFT the direction of the 2 servo can be changed here: invert the sign before axisPID
        else {
          servo[1] = constrain(1500 - axisPID[PITCH] - axisPID[ROLL], 1020, 2000);
          modeState = 1; }
        if (servo[2] > (constrain(1500 + axisPID[PITCH] - axisPID[ROLL], 1020, 2000)) && modeState == 0)
          servo[2] -= 1; //RIGHT
        else {
          servo[2] = constrain(1500 + axisPID[PITCH] - axisPID[ROLL], 1020, 2000);
          modeState = 1; }
        if (modeState2 == 0 && servo[5] < (rcData[AUX2] + axisPID[PITCH]/5)) { //rcData[AUX2] - rcCommand[ROLL]/5 + rcCommand[PITCH]/5
          servo[5] += 1; // slowly tilt motors forward every other cycle
          servo[4] -= 1;
          modeState2++; }
        else if (modeState2 == 0 && servo[5] > (rcData[AUX2] + axisPID[PITCH]/5)) {
          servo[5] -= 1; // slowly tilt motors backward every other cycle
          servo[4] += 1;
          modeState2++; }
        else if (modeState2 == 1) modeState2 = 0;
        else lastRcData[AUX2] = rcData[AUX2]; } // transition complete, update for next cycle!
      else { // transition to hover
        motor[0] = PIDMIX( 0,+4/3, 0); //REAR
        motor[1] = PIDMIX(-1,-1/3, 0); //RIGHT
        motor[2] = PIDMIX(+1,-1/3, 0); //LEFT
        servo[1]  = constrain(1500 - axisPID[PITCH] - axisPID[ROLL], 1020, 2000); //LEFT the direction of the 2 servo can be changed here: invert the sign before axisPID
        servo[2]  = constrain(1500 + axisPID[PITCH] - axisPID[ROLL], 1020, 2000); //RIGHT
        if (modeState2 == 0 && servo[5] > (rcData[AUX2] + axisPID[PITCH]/5 + 5)) {
          servo[5] -= 1;
          servo[4] += 1;
          modeState2++; } // slowly tilt motors backward
        else if (modeState2 == 1) modeState2 = 0;
        else lastRcData[AUX2] = rcData[AUX2]; } } // transition complete, update for next cycle!
    else { // not in any transitional state!
      if (modeState == 1) modeState = 0; // this prepares for next transition
      if (rcData[AUX2]>1600) { // forward flight
        motor[0] = MINCOMMAND ;                                                   //REAR
        motor[1] = rcCommand[THROTTLE] - YAW_DIRECTION * rcCommand[YAW]/10 ;      //RIGHT
        motor[2] = rcCommand[THROTTLE] + YAW_DIRECTION * rcCommand[YAW]/10 ;      //LEFT
        servo[1]  = constrain(1500 - axisPID[PITCH] - axisPID[ROLL], 1020, 2000); //LEFT the direction of the 2 servo can be changed here: invert the sign before axisPID
        servo[2]  = constrain(1500 + axisPID[PITCH] - axisPID[ROLL], 1020, 2000); //RIGHT
        servo[5]  = constrain(rcData[AUX2] - rcCommand[ROLL]/5 + rcCommand[PITCH]/5, 1020, 2000);   //front motor tilt (regular speed)
        servo[4]  = constrain(3000 - rcData[AUX2] - rcCommand[ROLL]/5 - rcCommand[PITCH]/5, 1020, 2000); }   //front motor tilt (regular speed)
      else if (rcData[AUX2]>1300) { // fast hover
        motor[0] = PIDMIX( 0,+4/3, 0); //REAR
        motor[1] = PIDMIX(-1,-1/3,-1/6); //RIGHT
        motor[2] = PIDMIX(+1,-1/3,+1/6); //LEFT
        servo[1]  = constrain(1500 - axisPID[PITCH] - axisPID[ROLL], 1020, 2000); //LEFT the direction of the 2 servo can be changed here: invert the sign before axisPID
        servo[2]  = constrain(1500 + axisPID[PITCH] - axisPID[ROLL], 1020, 2000); //RIGHT
        servo[5]  = constrain(rcData[AUX2] - rcCommand[ROLL]/5 + rcCommand[PITCH]/5, 1020, 2000);     //front motor tilt (regular speed)
        servo[4]  = constrain(3000 - rcData[AUX2] - rcCommand[ROLL]/5 - rcCommand[PITCH]/5, 1020, 2000); }   //front motor tilt (regular speed)
      else { // hover
        motor[0] = PIDMIX( 0,+4/3, 0); //REAR
        motor[1] = PIDMIX(-1,-2/3, 0); //RIGHT
        motor[2] = PIDMIX(+1,-2/3, 0); //LEFT
        if (modeState2 == 0) { // slowly tilt flaps down
          servo[1]  = constrain(servo[1] - 1, 1020, 2000);   //LEFT the direction of the 2 servo can be changed here: invert the sign before axisPID
          servo[2]  = constrain(servo[2] + 1, 1020, 2000);   //RIGHT
          modeState2++; }
        else if (modeState2 == 1) modeState2 = 0;
        servo[5]  = constrain(rcData[AUX2] + rcCommand[YAW]/5, 1020, 2000);     //front motor tilt (regular speed)
        servo[4]  = constrain(3000 - rcData[AUX2] + rcCommand[YAW]/5, 1020, 2000); } } //front motor tilt (regular speed)
  #endif
  #ifdef QUADP
    motor[0] = PIDMIX( 0,+1,-1); //REAR
    motor[1] = PIDMIX(-1, 0,+1); //RIGHT
    motor[2] = PIDMIX(+1, 0,+1); //LEFT
    motor[3] = PIDMIX( 0,-1,-1); //FRONT
  #endif
  #ifdef QUADX
    motor[0] = PIDMIX(-1,+1,-1); //REAR_R
    motor[1] = PIDMIX(-1,-1,+1); //FRONT_R
    motor[2] = PIDMIX(+1,+1,+1); //REAR_L
    motor[3] = PIDMIX(+1,-1,-1); //FRONT_L
  #endif
  #ifdef Y4
    motor[0] = PIDMIX(+0,+1,-1);   //REAR_1 CW
    motor[1] = PIDMIX(-1,-1, 0); //FRONT_R CCW
    motor[2] = PIDMIX(+0,+1,+1);   //REAR_2 CCW
    motor[3] = PIDMIX(+1,-1, 0); //FRONT_L CW
  #endif
  #ifdef Y6
    motor[0] = PIDMIX(+0,+4/3,+1); //REAR
    motor[1] = PIDMIX(-1,-2/3,-1); //RIGHT
    motor[2] = PIDMIX(+1,-2/3,-1); //LEFT
    motor[3] = PIDMIX(+0,+4/3,-1); //UNDER_REAR
    motor[4] = PIDMIX(-1,-2/3,+1); //UNDER_RIGHT
    motor[5] = PIDMIX(+1,-2/3,+1); //UNDER_LEFT    
  #endif
  #ifdef HEX6
    motor[0] = PIDMIX(-1/2,+1/2,+1); //REAR_R
    motor[1] = PIDMIX(-1/2,-1/2,-1); //FRONT_R
    motor[2] = PIDMIX(+1/2,+1/2,+1); //REAR_L
    motor[3] = PIDMIX(+1/2,-1/2,-1); //FRONT_L
    motor[4] = PIDMIX(+0  ,-1  ,+1); //FRONT
    motor[5] = PIDMIX(+0  ,+1  ,-1); //REAR
  #endif
  #ifdef HEX6X
    motor[0] = PIDMIX(-1/2,+1/2,+1); //REAR_R
    motor[1] = PIDMIX(-1/2,-1/2,+1); //FRONT_R
    motor[2] = PIDMIX(+1/2,+1/2,-1); //REAR_L
    motor[3] = PIDMIX(+1/2,-1/2,-1); //FRONT_L
    motor[4] = PIDMIX(-1  ,+0  ,-1); //RIGHT
    motor[5] = PIDMIX(+1  ,+0  ,+1); //LEFT
  #endif
  #ifdef OCTOX8
    motor[0] = PIDMIX(-1,+1,-1); //REAR_R
    motor[1] = PIDMIX(-1,-1,+1); //FRONT_R
    motor[2] = PIDMIX(+1,+1,+1); //REAR_L
    motor[3] = PIDMIX(+1,-1,-1); //FRONT_L
    motor[4] = PIDMIX(-1,+1,+1); //UNDER_REAR_R
    motor[5] = PIDMIX(-1,-1,-1); //UNDER_FRONT_R
    motor[6] = PIDMIX(+1,+1,-1); //UNDER_REAR_L
    motor[7] = PIDMIX(+1,-1,+1); //UNDER_FRONT_L
  #endif
  #ifdef OCTOFLATP
    motor[0] = PIDMIX(+7/10,-7/10,+1); //FRONT_L
    motor[1] = PIDMIX(-7/10,-7/10,+1); //FRONT_R
    motor[2] = PIDMIX(-7/10,+7/10,+1); //REAR_R
    motor[3] = PIDMIX(+7/10,+7/10,+1); //REAR_L
    motor[4] = PIDMIX(+0   ,-1   ,-1); //FRONT
    motor[5] = PIDMIX(-1   ,+0   ,-1); //RIGHT
    motor[6] = PIDMIX(+0   ,+1   ,-1); //REAR
    motor[7] = PIDMIX(+1   ,+0   ,-1); //LEFT 
  #endif
  #ifdef OCTOFLATX
    motor[0] = PIDMIX(+1  ,-1/2,+1); //MIDFRONT_L
    motor[1] = PIDMIX(-1/2,-1  ,+1); //FRONT_R
    motor[2] = PIDMIX(-1  ,+1/2,+1); //MIDREAR_R
    motor[3] = PIDMIX(+1/2,+1  ,+1); //REAR_L
    motor[4] = PIDMIX(+1/2,-1  ,-1); //FRONT_L
    motor[5] = PIDMIX(-1  ,-1/2,-1); //MIDFRONT_R
    motor[6] = PIDMIX(-1/2,+1  ,-1); //REAR_R
    motor[7] = PIDMIX(+1  ,+1/2,-1); //MIDREAR_L 
  #endif

  #ifdef SERVO_TILT
    if (rcOptions & activate[BOXCAMSTAB] ) {
      servo[1] = constrain(TILT_PITCH_MIDDLE + TILT_PITCH_PROP * angle[PITCH] /16 + rcData[CAMPITCH]-1500 , TILT_PITCH_MIN, TILT_PITCH_MAX);
      servo[2] = constrain(TILT_ROLL_MIDDLE  + TILT_ROLL_PROP  * angle[ROLL]  /16 + rcData[CAMROLL]-1500, TILT_ROLL_MIN, TILT_ROLL_MAX);
    } else {
      servo[1] = constrain(TILT_PITCH_MIDDLE  + rcData[CAMPITCH]-1500 , TILT_PITCH_MIN, TILT_PITCH_MAX);
      servo[2] = constrain(TILT_ROLL_MIDDLE   + rcData[CAMROLL]-1500,  TILT_ROLL_MIN, TILT_ROLL_MAX);
    }
  #endif
  #ifdef GIMBAL
    servo[1] = constrain(TILT_PITCH_MIDDLE + TILT_PITCH_PROP * angle[PITCH] /16 + rcCommand[PITCH], TILT_PITCH_MIN, TILT_PITCH_MAX);
    servo[2] = constrain(TILT_ROLL_MIDDLE + TILT_ROLL_PROP   * angle[ROLL]  /16 + rcCommand[ROLL], TILT_ROLL_MIN, TILT_ROLL_MAX);
  #endif
  #ifdef FLYING_WING
    servo[1]  = constrain(1500 + axisPID[PITCH] - axisPID[ROLL], 1020, 2000); //LEFT the direction of the 2 servo can be changed here: invert the sign before axisPID
    servo[2]  = constrain(1500 + axisPID[PITCH] + axisPID[ROLL], 1020, 2000); //RIGHT
  #endif

  maxMotor=motor[0];
  for(i=1;i< NUMBER_MOTOR;i++)
    if (motor[i]>maxMotor) maxMotor=motor[i];
  for (i = 0; i < NUMBER_MOTOR; i++) {
    if (maxMotor > MAXTHROTTLE) // this is a way to still have good gyro corrections if at least one motor reaches its max.
      motor[i] -= maxMotor - MAXTHROTTLE;
    motor[i] = constrain(motor[i], MINTHROTTLE, MAXTHROTTLE);    
    if ((rcData[THROTTLE]) < MINCHECK)
      #ifndef MOTOR_STOP
        motor[i] = MINTHROTTLE;
      #else
        motor[i] = MINCOMMAND;
      #endif
    if (armed == 0)
      motor[i] = MINCOMMAND;
  }
}

