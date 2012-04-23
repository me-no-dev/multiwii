

void initOutput(){
  initMotors();
  writeAllMotors(MINCOMMAND);
  delay(300);
  #if defined(SERVO)
    #if (PRI_SERVO_FROM == 1) || (SEC_SERVO_FROM == 1)
      SERVO_1_PINMODE;
    #endif
    #if (PRI_SERVO_FROM <= 2 && PRI_SERVO_TO >= 2) || (SEC_SERVO_FROM <= 2 && SEC_SERVO_TO >= 2) 
      SERVO_2_PINMODE;
    #endif
    #if (PRI_SERVO_FROM <= 3 && PRI_SERVO_TO >= 3) || (SEC_SERVO_FROM <= 3 && SEC_SERVO_TO >= 3) 
      SERVO_3_PINMODE;
    #endif 
    #if (PRI_SERVO_FROM <= 4 && PRI_SERVO_TO >= 4) || (SEC_SERVO_FROM <= 4 && SEC_SERVO_TO >= 4) 
      SERVO_4_PINMODE;
    #endif 
    #if (PRI_SERVO_FROM <= 5 && PRI_SERVO_TO >= 5) || (SEC_SERVO_FROM <= 5 && SEC_SERVO_TO >= 5) 
      SERVO_5_PINMODE;
    #endif 
    #if (PRI_SERVO_FROM <= 6 && PRI_SERVO_TO >= 6) || (SEC_SERVO_FROM <= 6 && SEC_SERVO_TO >= 6) 
      SERVO_6_PINMODE;
    #endif 
    #if (PRI_SERVO_FROM <= 7 && PRI_SERVO_TO >= 7) || (SEC_SERVO_FROM <= 7 && SEC_SERVO_TO >= 7) 
      SERVO_7_PINMODE;
    #endif 
    #if (PRI_SERVO_FROM <= 8 && PRI_SERVO_TO >= 8) || (SEC_SERVO_FROM <= 8 && SEC_SERVO_TO >= 8) 
      SERVO_8_PINMODE;
    #endif
    init_Servo_SW_PWM();
  #endif 
}

#if (NUMBER_MOTOR > 4) && defined(SWPWM)
  // hexa with old but sometimes better SW PWM method
  // for setups without servos
  #if (NUMBER_MOTOR == 6) && !defined(SERVO)
    ISR(SOFT_PWM_ISR1) { 
      static uint8_t state = 0;
      if(state == 0){
        SOFT_PWM_1_PIN_HIGH;
        SOFT_PWM_CHANNEL1 += atomicPWM_PIN5_highState;
        state = 1;
      }else if(state == 1){
        SOFT_PWM_CHANNEL1 += atomicPWM_PIN5_highState;
        state = 2;
      }else if(state == 2){
        SOFT_PWM_1_PIN_LOW;
        SOFT_PWM_CHANNEL1 += atomicPWM_PIN5_lowState;
        state = 3;  
      }else if(state == 3){
        SOFT_PWM_CHANNEL1 += atomicPWM_PIN5_lowState;
        state = 0;   
      }
    }
    ISR(SOFT_PWM_ISR2) { 
      static uint8_t state = 0;
      if(state == 0){
        SOFT_PWM_2_PIN_HIGH;
        SOFT_PWM_CHANNEL2 += atomicPWM_PIN6_highState;
        state = 1;
      }else if(state == 1){
        SOFT_PWM_CHANNEL2 += atomicPWM_PIN6_highState;
        state = 2;
      }else if(state == 2){
        SOFT_PWM_2_PIN_LOW;
        SOFT_PWM_CHANNEL2 += atomicPWM_PIN6_lowState;
        state = 3;  
      }else if(state == 3){
        SOFT_PWM_CHANNEL2 += atomicPWM_PIN6_lowState;
        state = 0;   
      }
    }
  #else
    // HEXA with just OCR0B 
    ISR(SOFT_PWM_ISR1) { 
      static uint8_t state = 0;
      if(state == 0){
        SOFT_PWM_1_PIN_HIGH;
        SOFT_PWM_CHANNEL1 += atomicPWM_PIN5_highState;
        state = 1;
      }else if(state == 1){
        SOFT_PWM_2_PIN_LOW;
        SOFT_PWM_CHANNEL1 += atomicPWM_PIN6_lowState;
        state = 2;
      }else if(state == 2){
        SOFT_PWM_2_PIN_HIGH;
        SOFT_PWM_CHANNEL1 += atomicPWM_PIN6_highState;
        state = 3;  
      }else if(state == 3){
        SOFT_PWM_1_PIN_LOW;
        SOFT_PWM_CHANNEL1 += atomicPWM_PIN5_lowState;
        state = 0;   
      }
    } 
    //the same with digital PIN A2 & 12 OCR0A counter for OCTO
    #if (NUMBER_MOTOR > 6) && !defined(HWPWM6)
      ISR(SOFT_PWM_ISR2) {
        static uint8_t state = 0;
        if(state == 0){
          SOFT_PWM_3_PIN_HIGH;
          SOFT_PWM_CHANNEL2 += atomicPWM_PINA2_highState;
          state = 1;
        }else if(state == 1){
          SOFT_PWM_4_PIN_LOW;
          SOFT_PWM_CHANNEL2 += atomicPWM_PIN12_lowState;
          state = 2;
        }else if(state == 2){
          SOFT_PWM_4_PIN_HIGH;
          SOFT_PWM_CHANNEL2 += atomicPWM_PIN12_highState;
          state = 3;  
        }else if(state == 3){
          SOFT_PWM_3_PIN_LOW;
          SOFT_PWM_CHANNEL2 += atomicPWM_PINA2_lowState;
          state = 0;   
        }
      }
    #endif
  #endif
#endif


void writeAllMotors(int16_t mc) {   // Sends commands to all motors
  for (uint8_t i =0;i<NUMBER_MOTOR;i++)
    motor[i]=mc;
  writeMotors();
}

#if defined(SERVO)
ISR(SERVO_ISR) {
  static uint8_t state = 0;
  static uint8_t count;
  if (state == 0) {
    #if (PRI_SERVO_FROM == 1) || (SEC_SERVO_FROM == 1)
      SERVO_1_PIN_HIGH;
    #endif
    SERVO_CHANNEL+= SERVO_1K_US; // 1000 us
    state++ ;
  } else if (state == 1) {
    SERVO_CHANNEL+= atomicServo[0]; // 1000 + [0-1020] us
    state++;
  } else if (state == 2) {
    #if (PRI_SERVO_FROM == 1) || (SEC_SERVO_FROM == 1)
      SERVO_1_PIN_LOW;
    #endif
    #if (PRI_SERVO_FROM <= 2 && PRI_SERVO_TO >= 2) || (SEC_SERVO_FROM <= 2 && SEC_SERVO_TO >= 2) 
      SERVO_2_PIN_HIGH;
    #endif
    SERVO_CHANNEL+= SERVO_1K_US; // 1000 us
    state++;
  } else if (state == 3) {
    SERVO_CHANNEL+= atomicServo[1]; // 1000 + [0-1020] us
    state++;
  } else if (state == 4) {
    #if (PRI_SERVO_FROM <= 2 && PRI_SERVO_TO >= 2) || (SEC_SERVO_FROM <= 2 && SEC_SERVO_TO >= 2)  
      SERVO_2_PIN_LOW;
    #endif
    #if (PRI_SERVO_FROM <= 3 && PRI_SERVO_TO >= 3) || (SEC_SERVO_FROM <= 3 && SEC_SERVO_TO >= 3)   
      SERVO_3_PIN_HIGH;
    #endif
    state++;
    SERVO_CHANNEL+= SERVO_1K_US; // 1000 us
  } else if (state == 5) {
    SERVO_CHANNEL+= atomicServo[2]; // 1000 + [0-1020] us
    state++;
  } else if (state == 6) {
    #if (PRI_SERVO_FROM <= 3 && PRI_SERVO_TO >= 3) || (SEC_SERVO_FROM <= 3 && SEC_SERVO_TO >= 3)      
      SERVO_3_PIN_LOW;
    #endif
    #if (PRI_SERVO_FROM <= 4 && PRI_SERVO_TO >= 4) || (SEC_SERVO_FROM <= 4 && SEC_SERVO_TO >= 4)   
      SERVO_4_PIN_HIGH;
    #endif
    state++;
    SERVO_CHANNEL+= SERVO_1K_US; // 1000 us
  } else if (state == 7) {
    SERVO_CHANNEL+= atomicServo[3]; // 1000 + [0-1020] us
    state++;
  } else if (state == 8) {
    #if (PRI_SERVO_FROM <= 4 && PRI_SERVO_TO >= 4) || (SEC_SERVO_FROM <= 4 && SEC_SERVO_TO >= 4)    
      SERVO_4_PIN_LOW;
    #endif  
    #if (PRI_SERVO_FROM <= 5 && PRI_SERVO_TO >= 5) || (SEC_SERVO_FROM <= 5 && SEC_SERVO_TO >= 5)   
      SERVO_5_PIN_HIGH;;
    #endif 
    state++;
    SERVO_CHANNEL+= SERVO_1K_US; // 1000 us
  } else if (state == 9) {
    SERVO_CHANNEL+= atomicServo[4]; // 1000 + [0-1020] us
    state++;
  } else if (state == 10) {
    #if (PRI_SERVO_FROM <= 5 && PRI_SERVO_TO >= 5) || (SEC_SERVO_FROM <= 5 && SEC_SERVO_TO >= 5)    
      SERVO_5_PIN_LOW;
    #endif 
    #if (PRI_SERVO_FROM <= 6 && PRI_SERVO_TO >= 6) || (SEC_SERVO_FROM <= 6 && SEC_SERVO_TO >= 6)    
      SERVO_6_PIN_HIGH;;
    #endif 
    state++;
    SERVO_CHANNEL+= SERVO_1K_US; // 1000 us
  } else if (state == 11) {
    SERVO_CHANNEL+= atomicServo[5]; // 1000 + [0-1020] us
    state++;
  } else if (state == 12) {
    #if (PRI_SERVO_FROM <= 6 && PRI_SERVO_TO >= 6) || (SEC_SERVO_FROM <= 6 && SEC_SERVO_TO >= 6)       
      SERVO_6_PIN_LOW;
    #endif 
    #if (PRI_SERVO_FROM <= 7 && PRI_SERVO_TO >= 7) || (SEC_SERVO_FROM <= 7 && SEC_SERVO_TO >= 7)       
      SERVO_7_PIN_HIGH;
    #endif 
    state++;
    SERVO_CHANNEL+= SERVO_1K_US; // 1000 us
  } else if (state == 13) {
    SERVO_CHANNEL+= atomicServo[6]; // 1000 + [0-1020] us
    state++;
  } else if (state == 14) {
    #if (PRI_SERVO_FROM <= 7 && PRI_SERVO_TO >= 7) || (SEC_SERVO_FROM <= 7 && SEC_SERVO_TO >= 7)       
      SERVO_7_PIN_LOW;
    #endif 
    #if (PRI_SERVO_FROM <= 8 && PRI_SERVO_TO >= 8) || (SEC_SERVO_FROM <= 8 && SEC_SERVO_TO >= 8)    
      SERVO_8_PIN_HIGH;
    #endif     
    state++;
    SERVO_CHANNEL+= SERVO_1K_US; // 1000 us
  } else if (state == 15) {
    SERVO_CHANNEL+= atomicServo[7]; // 1000 + [0-1020] us
    state++;
  } else if (state == 16) {
    #if (PRI_SERVO_FROM <= 8 && PRI_SERVO_TO >= 8) || (SEC_SERVO_FROM <= 8 && SEC_SERVO_TO >= 8)     
      SERVO_8_PIN_LOW;
    #endif 
    count = 2;
    state++;
    SERVO_CHANNEL+= SERVO_1K_US; // 1000 us
  } else if (state == 17) {
    if (count > 0) count--;
    else state = 0;
    SERVO_CHANNEL+= SERVO_1K_US;
  }
}
#endif

void mixTable() {
  int16_t maxMotor;
  uint8_t i;

  #define PIDMIX(X,Y,Z) rcCommand[THROTTLE] + axisPID[ROLL]*X + axisPID[PITCH]*Y + YAW_DIRECTION * axisPID[YAW]*Z

  #if NUMBER_MOTOR > 3
    //prevent "yaw jump" during yaw correction
    axisPID[YAW] = constrain(axisPID[YAW],-100-abs(rcCommand[YAW]),+100+abs(rcCommand[YAW]));
  #endif
  #ifdef BI
    motor[0] = PIDMIX(+1, 0, 0); //LEFT
    motor[1] = PIDMIX(-1, 0, 0); //RIGHT        
    servo[4]  = constrain(1500 + YAW_DIRECTION * (axisPID[YAW] + axisPID[PITCH]), 1020, 2000); //LEFT
    servo[5]  = constrain(1500 + YAW_DIRECTION * (axisPID[YAW] - axisPID[PITCH]), 1020, 2000); //RIGHT
  #endif
  #ifdef TRI
    motor[0] = PIDMIX( 0,+4/3, 0); //REAR
    motor[1] = PIDMIX(-1,-2/3, 0); //RIGHT
    motor[2] = PIDMIX(+1,-2/3, 0); //LEFT
    servo[5] = constrain(tri_yaw_middle + YAW_DIRECTION * axisPID[YAW], TRI_YAW_CONSTRAINT_MIN, TRI_YAW_CONSTRAINT_MAX); //REAR

  #endif
  #ifdef QUADP
    motor[0] = PIDMIX( 0,+1,-1); //REAR
    motor[1] = PIDMIX(-1, 0,+1); //RIGHT
    motor[2] = PIDMIX(+1, 0,+1); //LEFT
    motor[3] = PIDMIX( 0,-1,-1); //FRONT
  #endif
  #ifdef QUADX
    motor[0] = PIDMIX(-1,+1,-1); //REAR_R
    motor[1] = PIDMIX(-2/3,-1,+1); //FRONT_R
    motor[2] = PIDMIX(+1,+1,+1); //REAR_L
    motor[3] = PIDMIX(+2/3,-1,-1); //FRONT_L
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
  #ifdef VTAIL4
    motor[0] = PIDMIX(+0,+1, -1/2);   //REAR_R 
    motor[1] = PIDMIX(-1, -1, +2/10); //FRONT_R 
    motor[2] = PIDMIX(+0,+1, +1/2);   //REAR_L 
    motor[3] = PIDMIX(+1, -1, -2/10); //FRONT_L
  #endif

  #if defined(SERVO_TILT)
    #if defined(A0_A1_PIN_HEX) && (NUMBER_MOTOR == 6) && defined(PROMINI)
      #define S_PITCH servo[2]
      #define S_ROLL  servo[3]
    #else
      #define S_PITCH servo[0]
      #define S_ROLL  servo[1]
    #endif
    S_PITCH = TILT_PITCH_MIDDLE + rcData[AUX3]-1500;
    S_ROLL  = TILT_ROLL_MIDDLE  + rcData[AUX4]-1500;
    if (rcOptions[BOXCAMSTAB]) {
      S_PITCH += TILT_PITCH_PROP * angle[PITCH] /16 ;
      S_ROLL  += TILT_ROLL_PROP  * angle[ROLL]  /16 ;
    }
    S_PITCH = constrain(S_PITCH, TILT_PITCH_MIN, TILT_PITCH_MAX);
    S_ROLL  = constrain(S_ROLL , TILT_ROLL_MIN, TILT_ROLL_MAX  );   
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
