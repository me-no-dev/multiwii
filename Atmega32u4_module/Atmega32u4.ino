#if defined(__AVR_ATmega32U4__)
  /********************************************************************/
  /****           proc specific functions & variables              ****/
  /********************************************************************/
  
  uint8_t serialBufferRX[256][2];
  volatile uint8_t serialHeadRX[2],serialTailRX[2];
  
  //===================================//
  //========= RX Trottle INT  =========//
  //===================================//  
  void init_special_pins(){
    //attachinterrupt dosent works with atmega32u4 ATM.
    //Trottle on pin 7
    pinMode(7,INPUT); // set to input
    PORTE |= (1 << 6); // enable pullups
    EIMSK |= (1 << INT6); // enable interuppt
    EICRB |= (1 << ISC60);    
    #if defined(RCAUX2PINRXO)
      //AUX2 on RX pin
      pinMode(0,INPUT); // set to input
      PORTD |= (1 << 2); // enable pullups
      EIMSK |= (1 << INT2); // enable interuppt
      EICRA |= (1 << ISC20);
    #endif
  }
  ISR(INT6_vect){ 
    static uint16_t now,diff;
    static uint16_t last = 0;
    now = micros();  
    diff = now - last;
    last = now;
    if(900<diff && diff<2200){ 
      rcValue[7] = diff; 
      /*
      #if defined(FAILSAFE)
        if(failsafeCnt > 20) failsafeCnt -= 20; else failsafeCnt = 0;   // If pulse present on THROTTLE pin (independent from ardu version), clear FailSafe counter  - added by MIS
      #endif 
      */
    }  
  }  
  // Aux 2
  #if defined(RCAUX2PINRXO)
    ISR(INT2_vect){
      static uint16_t now,diff;
      static uint16_t last = 0; 
      now = micros();  
      diff = now - last;
      last = now;
      if(900<diff && diff<2200) rcValue[0] = diff;
    }
  #endif
  //===================================//
  //======== Output HW & SW PWM =======//
  //===================================//  
  
  #if !defined(HWPWM6)
    uint8_t PWM_PIN[8] = {9,10,5,6,4,A2,A0,A1};   //for a quad+: rear,right,left,front
  #else
    uint8_t PWM_PIN[8] = {9,10,5,6,11,13,A0,A1};   //for a quad+: rear,right,left,front
  #endif

  #if defined(HWPWM6)
    // 8 bit servos
    volatile uint8_t atomicServo[8] = {125,125,125,125,125,125,125,125};
    //for OCTO on promicro with HWPWM6
    volatile uint8_t atomicPWM_PINA2_lowState;
    volatile uint8_t atomicPWM_PINA2_highState;
    volatile uint8_t atomicPWM_PIN12_lowState;
    volatile uint8_t atomicPWM_PIN12_highState;
  #else
    // 11 bit servos
    volatile uint16_t atomicServo[8] = {8000,8000,8000,8000,8000,8000,8000,8000};
    //for HEX Y6 and HEX6/HEX6X on Promicro
    volatile uint16_t atomicPWM_PIN5_lowState;
    volatile uint16_t atomicPWM_PIN5_highState;
    volatile uint16_t atomicPWM_PIN6_lowState;
    volatile uint16_t atomicPWM_PIN6_highState;
    //for OCTO on Promicro
    volatile uint16_t atomicPWM_PINA2_lowState;
    volatile uint16_t atomicPWM_PINA2_highState;
    volatile uint16_t atomicPWM_PIN12_lowState;
    volatile uint16_t atomicPWM_PIN12_highState;
  #endif
  
  void writeServos() {
    #if defined(SERVO)
      #if defined(PRI_SERVO_FROM)    // write primary servos
        for(uint8_t i = (PRI_SERVO_FROM-1); i < PRI_SERVO_TO; i++){
          #if defined(HWPWM6)
            atomicServo[i] = (servo[i]-1000)>>2;
          #else
            atomicServo[i] = (servo[i]-1000)<<4;
          #endif
        }
      #endif
      #if defined(SEC_SERVO_FROM)   // write secundary servos
        for(uint8_t i = (SEC_SERVO_FROM-1); i < SEC_SERVO_TO; i++){
          #if defined(HWPWM6)
            atomicServo[i] = (servo[i]-1000)>>2;
          #else
            atomicServo[i] = (servo[i]-1000)<<4;
          #endif
        }
      #endif
    #endif
  }
  
  void writeMotors() { // [1000;2000] => [125;250]
    #if (NUMBER_MOTOR > 0) // Timer 1 A & B [1000:2000] => [8000:16000]
      OCR1A = motor[0]<<3; //  pin 9
    #endif
    #if (NUMBER_MOTOR > 1)
      OCR1B = motor[1]<<3; //  pin 10
    #endif
    #if (NUMBER_MOTOR > 2) // Timer 4 A & D [1000:2000] => [1000:2000]
      #if !defined(HWPWM6)
        // to write values > 255 to timer 4 A/B we need to split the bytes
        static uint8_t pwm4_HBA;
        static uint16_t pwm4_LBA; // high and low byte for timer 4 A
        pwm4_LBA = 2047-motor[2]; pwm4_HBA = 0; // channel A is inverted
        while(pwm4_LBA > 255){
          pwm4_HBA++;
          pwm4_LBA-=256;
        }     
        TC4H = pwm4_HBA; OCR4A = pwm4_LBA; //  pin 5
      #else
        OCR3A = motor[2]<<3; //  pin 5
      #endif
    #endif
    #if (NUMBER_MOTOR > 3)
      static uint8_t pwm4_HBD;
      static uint16_t pwm4_LBD; // high and low byte for timer 4 D
      pwm4_LBD = motor[3]; pwm4_HBD = 0;
      while(pwm4_LBD > 255){
        pwm4_HBD++;
        pwm4_LBD-=256;
      }     
      TC4H = pwm4_HBD; OCR4D = pwm4_LBD; //  pin 6
    #endif    
    #if (NUMBER_MOTOR > 4)
      #if !defined(HWPWM6)
        #if (NUMBER_MOTOR == 6) && !defined(SERVO)
          atomicPWM_PIN5_highState = motor[4]<<3;
          atomicPWM_PIN5_lowState = 16383-atomicPWM_PIN5_highState;
          atomicPWM_PIN6_highState = motor[5]<<3;
          atomicPWM_PIN6_lowState = 16383-atomicPWM_PIN6_highState;      
        #else
          atomicPWM_PIN5_highState = ((motor[4]-1000)<<4)+320;
          atomicPWM_PIN5_lowState = 15743-atomicPWM_PIN5_highState;
          atomicPWM_PIN6_highState = ((motor[5]-1000)<<4)+320;
          atomicPWM_PIN6_lowState = 15743-atomicPWM_PIN6_highState;        
        #endif
      #else
        OCR1C = motor[4]<<3; //  pin 11
        static uint8_t pwm4_HBA;
        static uint16_t pwm4_LBA; // high and low byte for timer 4 A
        pwm4_LBA = motor[5]; pwm4_HBA = 0;
        while(pwm4_LBA > 255){
          pwm4_HBA++;
          pwm4_LBA-=256;
        }     
        TC4H = pwm4_HBA; OCR4A = pwm4_LBA; //  pin 13      
      #endif
    #endif
    #if (NUMBER_MOTOR > 6)
      #if !defined(HWPWM6)
        atomicPWM_PINA2_highState = ((motor[6]-1000)<<4)+320;
        atomicPWM_PINA2_lowState = 15743-atomicPWM_PINA2_highState;
        atomicPWM_PIN12_highState = ((motor[7]-1000)<<4)+320;
        atomicPWM_PIN12_lowState = 15743-atomicPWM_PIN12_highState;
      #else
        atomicPWM_PINA2_highState = ((motor[6]-1000)>>2)+5;
        atomicPWM_PINA2_lowState = 245-atomicPWM_PINA2_highState;
        atomicPWM_PIN12_highState = ((motor[7]-1000)>>2)+5;
        atomicPWM_PIN12_lowState = 245-atomicPWM_PIN12_highState;     
      #endif
    #endif
  }
  
  
  void initMotors() {
    for(uint8_t i=0;i<NUMBER_MOTOR;i++)
      pinMode(PWM_PIN[i],OUTPUT);
    #if (NUMBER_MOTOR > 0)
      TCCR1A |= (1<<WGM11); TCCR1A &= ~(1<<WGM10); TCCR1B |= (1<<WGM13);  // phase correct mode
      TCCR1B &= ~(1<<CS11); ICR1 |= 0x3FFF; // no prescaler & TOP to 16383;
      TCCR1A |= _BV(COM1A1); // connect pin 9 to timer 1 channel A
    #endif
    #if (NUMBER_MOTOR > 1)
      TCCR1A |= _BV(COM1B1); // connect pin 10 to timer 1 channel B
    #endif
    #if (NUMBER_MOTOR > 2)
      #if !defined(HWPWM6) // timer 4A
        TCCR4E |= (1<<ENHC4); // enhanced pwm mode
        TCCR4B &= ~(1<<CS41); TCCR4B |= (1<<CS42)|(1<<CS40); // prescaler to 16
        TCCR4D |= (1<<WGM40); TC4H = 0x3; OCR4C = 0xFF; // phase and frequency correct mode & top to 1023 but with enhanced pwm mode we have 2047
        TCCR4A |= (1<<COM4A0)|(1<<PWM4A); // connect pin 5 to timer 4 channel A 
      #else // timer 3A
        TCCR3A |= (1<<WGM31); TCCR3A &= ~(1<<WGM30); TCCR3B |= (1<<WGM33);  // phase correct mode
        TCCR3B &= ~(1<<CS31); ICR3 |= 0x3FFF; // no prescaler & TOP to 16383;
        TCCR3A |= _BV(COM3A1); // connect pin 5 to timer 3 channel A    
      #endif 
    #endif
    #if (NUMBER_MOTOR > 3)
      #if defined(HWPWM6) 
        TCCR4E |= (1<<ENHC4); // enhanced pwm mode
        TCCR4B &= ~(1<<CS41); TCCR4B |= (1<<CS42)|(1<<CS40); // prescaler to 16
        TCCR4D |= (1<<WGM40); TC4H = 0x3; OCR4C = 0xFF; // phase and frequency correct mode & top to 1023 but with enhanced pwm mode we have 2047
      #endif
      TCCR4C |= (1<<COM4D1)|(1<<PWM4D); // connect pin 6 to timer 4 channel D
    #endif
    #if (NUMBER_MOTOR > 4)
      #if defined(HWPWM6) 
        TCCR1A |= _BV(COM1C1); // connect pin 11 to timer 1 channel C
        TCCR4A |= (1<<COM4A1)|(1<<PWM4A); // connect pin 13 to timer 4 channel A 
      #else
        initializeSoftPWM();
      #endif
    #endif
    #if (NUMBER_MOTOR > 6)
      #if defined(HWPWM6) 
        initializeSoftPWM();
      #endif
    #endif
  } 


  void initializeSoftPWM() {
    #if  defined(HWPWM6)
       TCCR0A = 0; // normal counting mode
       #if (NUMBER_MOTOR > 4) && !defined(HWPWM6) 
         TIMSK0 |= (1<<OCIE0B); // Enable CTC interrupt  
       #endif
       #if (NUMBER_MOTOR > 6) || ((NUMBER_MOTOR == 6) && !defined(SERVO))
         TIMSK0 |= (1<<OCIE0A);
       #endif
     #else
       TCCR3A &= ~(1<<WGM30); // normal counting mode
        TCCR3B &= ~(1<<CS31); // no prescaler
        TIMSK3 |= (1<<OCIE3B); // Enable CTC interrupt  
        #if (NUMBER_MOTOR > 6) || ((NUMBER_MOTOR == 6) && !defined(SERVO))
          TIMSK3 |= (1<<OCIE3C);
        #endif   
     #endif
  }

  void init_Servo_SW_PWM(){
    #if defined(HWPWM6)
      TCCR0A = 0; // normal counting mode
      TIMSK0 |= (1<<OCIE0A); // Enable CTC interrupt
    #else
      TCCR3A &= ~(1<<WGM30); // normal counting mode
      TCCR3B &= ~(1<<CS31); // no prescaler
      TIMSK3 |= (1<<OCIE3A); // Enable CTC interrupt   
    #endif  
  }
  
  
  //===================================//
  //=========== Serial & USB ==========//
  //===================================//
  
  void UartSendData() {
    USB_Send(USB_CDC_TX,bufTX,headTX);
    headTX = 0;
  }  
  
  void SerialOpen(uint8_t port, uint32_t baud) {
    uint8_t h = ((F_CPU  / 4 / baud -1) / 2) >> 8;
    uint8_t l = ((F_CPU  / 4 / baud -1) / 2);
    if(port == 1){
      UCSR1A  = (1<<U2X1); UBRR1H = h; UBRR1L = l; UCSR1B |= (1<<RXEN1)|(1<<TXEN1)|(1<<RXCIE1);
    }
  }
  
  void SerialEnd(uint8_t port) {
    if(port == 1){
      UCSR1B &= ~((1<<RXEN1)|(1<<TXEN1)|(1<<RXCIE1));
    }
  }
  #if !defined(SPEKTRUM)
    ISR(USART1_RX_vect){
      uint8_t d = UDR1;
      uint8_t i = serialHeadRX[1] + 1;
      if (i != serialTailRX[1]) {serialBufferRX[serialHeadRX[1]][1] = d; serialHeadRX[1] = i;}
    }
  #endif
  uint8_t SerialRead(uint8_t port) {
    if(port == 0){
      return USB_Recv(USB_CDC_RX);
    }else{
      uint8_t c = serialBufferRX[serialTailRX[port]][port];
      if ((serialHeadRX[port] != serialTailRX[port])) serialTailRX[port] = serialTailRX[port] + 1;
      return c;
    }
  }
  uint8_t SerialAvailable(uint8_t port) {
    if(port == 0){
      return USB_Available(USB_CDC_RX);
    }else{
      return serialHeadRX[port] - serialTailRX[port];
    }
  }
  void SerialWrite(uint8_t port,uint8_t c){
   switch (port) {
      case 0: serialize8(c);UartSendData(); break;                 // Serial0 TX is driven via a buffer and a background intterupt
      case 1: while (!(UCSR1A & (1 << UDRE1))) ; UDR1 = c; break;  // Serial1 Serial2 and Serial3 TX are not driven via interrupts
    }
  } 

#endif

