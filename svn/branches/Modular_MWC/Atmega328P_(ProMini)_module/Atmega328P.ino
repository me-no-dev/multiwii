#if defined(__AVR_ATmega328P__)
  /********************************************************************/
  /****           proc specific functions & variables              ****/
  /********************************************************************/
  
  uint8_t serialBufferRX[256][1];
  volatile uint8_t serialHeadRX[1],serialTailRX[1];
  
  //===================================//
  //========= EXTRA RX AUX PINS  =========//
  //===================================//  
  #if !defined(SPECIAL_RX) 
    void init_special_pins(){
      #if defined(RCAUXPIN)
        PCICR  |= (1<<0) ; // PCINT activated also for PINS [D8-D13] on port B
	 #if defined(RCAUXPIN8)
           PCMSK0 = (1<<0);
	 #endif
	 #if defined(RCAUXPIN12)
           PCMSK0 = (1<<4);
	 #endif
       #endif
     }
     #if defined(RCAUXPIN)
       /* this ISR is a simplification of the previous one for PROMINI on port D
        it's simplier because we know the interruption deals only with one PIN:
        bit 0 of PORT B, ie Arduino PIN 8
        or bit 4 of PORTB, ie Arduino PIN 12
        => no need to check which PIN has changed */
        ISR(PCINT0_vect) {
  	uint8_t pin;
  	uint16_t cTime,dTime;
  	static uint16_t edgeTime;
  	    
  	pin = PINB;
  	sei();
  	cTime = micros();
  	#if defined(RCAUXPIN8)
  	  if (!(pin & 1<<0)) {     //indicates if the bit 0 of the arduino port [B0-B7] is not at a high state (so that we match here only descending PPM pulse)
  	#endif
  	#if defined(RCAUXPIN12)
  	  if (!(pin & 1<<4)) {     //indicates if the bit 4 of the arduino port [B0-B7] is not at a high state (so that we match here only descending PPM pulse)
  	#endif
          dTime = cTime-edgeTime; if (900<dTime && dTime<2200) rcValue[0] = dTime; // just a verification: the value must be in the range [1000;2000] + some margin
  	     } else edgeTime = cTime;    // if the bit 2 is at a high state (ascending PPM pulse), we memorize the time
        }
     #endif
  #endif
  //===================================//
  //======== Output HW & SW PWM =======//
  //===================================//  
  
  uint8_t PWM_PIN[8] = {9,10,11,3,6,5,A2,12};   //for a quad+: rear,right,left,front

  volatile uint8_t atomicServo[8] = {125,125,125,125,125,125,125,125};
  //for HEX Y6 and HEX6/HEX6X flat for promini
  volatile uint8_t atomicPWM_PIN5_lowState;
  volatile uint8_t atomicPWM_PIN5_highState;
  volatile uint8_t atomicPWM_PIN6_lowState;
  volatile uint8_t atomicPWM_PIN6_highState;
  //for OCTO on promini
  volatile uint8_t atomicPWM_PINA2_lowState;
  volatile uint8_t atomicPWM_PINA2_highState;
  volatile uint8_t atomicPWM_PIN12_lowState;
  volatile uint8_t atomicPWM_PIN12_highState;
  
  
  void writeServos() {
    #if defined(SERVO)
      #if defined(PRI_SERVO_FROM)    // write primary servos
        for(uint8_t i = (PRI_SERVO_FROM-1); i < PRI_SERVO_TO; i++){
          atomicServo[i] = (servo[i]-1000)>>2;
        }
      #endif
      #if defined(SEC_SERVO_FROM)   // write secundary servos
        for(uint8_t i = (SEC_SERVO_FROM-1); i < SEC_SERVO_TO; i++){
          atomicServo[i] = (servo[i]-1000)>>2;
        }
      #endif
    #endif
  }
  
  void writeMotors() { // [1000;2000] => [125;250]
    #if (NUMBER_MOTOR > 0)
      #ifndef EXT_MOTOR_RANGE 
        OCR1A = motor[0]>>3; //  pin 9
      #else
        OCR1A = ((motor[0]>>2) - 250) + 2;
      #endif
    #endif
    #if (NUMBER_MOTOR > 1)
      #ifndef EXT_MOTOR_RANGE 
        OCR1B = motor[1]>>3; //  pin 10
      #else
        OCR1B = ((motor[1]>>2) - 250) + 2;
      #endif
    #endif
    #if (NUMBER_MOTOR > 2)
      #ifndef EXT_MOTOR_RANGE
        OCR2A = motor[2]>>3; //  pin 11
      #else
        OCR2A = ((motor[2]>>2) - 250) + 2;
      #endif
    #endif
    #if (NUMBER_MOTOR > 3)
      #ifndef EXT_MOTOR_RANGE
        OCR2B = motor[3]>>3; //  pin 3
      #else
        OCR2B = ((motor[3]>>2) - 250) + 2;
      #endif
    #endif
    #if (NUMBER_MOTOR > 4)
      #if (NUMBER_MOTOR == 6) && !defined(SERVO)
        #ifndef EXT_MOTOR_RANGE 
          atomicPWM_PIN6_highState = motor[4]>>3;
          atomicPWM_PIN5_highState = motor[5]>>3;
        #else
          atomicPWM_PIN6_highState = ((motor[4]>>2) - 250) + 2;
          atomicPWM_PIN5_highState = ((motor[5]>>2) - 250) + 2;       
        #endif
        atomicPWM_PIN6_lowState  = 255-atomicPWM_PIN6_highState;
        atomicPWM_PIN5_lowState  = 255-atomicPWM_PIN5_highState; 
      #else //note: EXT_MOTOR_RANGE not possible here
        atomicPWM_PIN6_highState = ((motor[4]-1000)>>2)+5;
        atomicPWM_PIN6_lowState  = 245-atomicPWM_PIN6_highState;
        atomicPWM_PIN5_highState = ((motor[5]-1000)>>2)+100;
        atomicPWM_PIN5_lowState  = 245-atomicPWM_PIN5_highState;
      #endif
    #endif
    #if (NUMBER_MOTOR > 6) //note: EXT_MOTOR_RANGE not possible here
      atomicPWM_PINA2_highState = ((motor[6]-1000)>>2)+5;
      atomicPWM_PINA2_lowState  = 245-atomicPWM_PINA2_highState;
      atomicPWM_PIN12_highState = ((motor[7]-1000)>>2)+5;
      atomicPWM_PIN12_lowState  = 245-atomicPWM_PIN12_highState;
    #endif
  }
  
  
  void initMotors() {
    for(uint8_t i=0;i<NUMBER_MOTOR;i++)
      pinMode(PWM_PIN[i],OUTPUT);
    #if (NUMBER_MOTOR > 0)
      TCCR1A |= _BV(COM1A1); // connect pin 9 to timer 1 channel A
    #endif
    #if (NUMBER_MOTOR > 1)
      TCCR1A |= _BV(COM1B1); // connect pin 10 to timer 1 channel B
    #endif
    #if (NUMBER_MOTOR > 2)
      TCCR2A |= _BV(COM2A1); // connect pin 11 to timer 2 channel A
    #endif
    #if (NUMBER_MOTOR > 3)
      TCCR2A |= _BV(COM2B1); // connect pin 3 to timer 2 channel B
    #endif
    #if (NUMBER_MOTOR > 5)  // PIN 5 & 6 or A0 & A1
      initializeSoftPWM();
      #if defined(A0_A1_PIN_HEX) || (NUMBER_MOTOR > 6)
        pinMode(5,INPUT);pinMode(6,INPUT);     // we reactivate the INPUT affectation for these two PINs
        pinMode(A0,OUTPUT);pinMode(A1,OUTPUT);
      #endif
    #endif
  } 


  void initializeSoftPWM() {
    TCCR0A = 0; // normal counting mode
    #if (NUMBER_MOTOR > 4) && !defined(HWPWM6) 
       TIMSK0 |= (1<<OCIE0B); // Enable CTC interrupt  
    #endif
    #if (NUMBER_MOTOR > 6) || ((NUMBER_MOTOR == 6) && !defined(SERVO))
      TIMSK0 |= (1<<OCIE0A);
    #endif
  }

  void init_Servo_SW_PWM(){
    TCCR0A = 0; // normal counting mode
    TIMSK0 |= (1<<OCIE0A); // Enable CTC interrupt
  }
  
  
  //===================================//
  //============= Serial ==============//
  //===================================//
  
  void UartSendData(){}  
  
  void SerialOpen(uint8_t port, uint32_t baud) {
    uint8_t h = ((F_CPU  / 4 / baud -1) / 2) >> 8;
    uint8_t l = ((F_CPU  / 4 / baud -1) / 2);
    if(port == 0){
      UCSR0A  = (1<<U2X0); UBRR0H = h; UBRR0L = l; UCSR0B |= (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);
    }
  }
  ISR_UART {
    if (headTX != tailTX) UDR0 = bufTX[tailTX++];  // Transmit next byte in the ring
    if (tailTX == headTX) UCSR0B &= ~(1<<UDRIE0); // Check if all data is transmitted . if yes disable transmitter UDRE interrupt
  }
  void SerialEnd(uint8_t port) {
    if(port == 0){
      UCSR0B &= ~((1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0)|(1<<UDRIE0));
    }
  }
  #if !defined(SERIAL_RX)
    ISR(USART_RX_vect){
      uint8_t d = UDR0;
      uint8_t i = serialHeadRX[0] + 1;
      if (i != serialTailRX[0]) {serialBufferRX[serialHeadRX[0]][0] = d; serialHeadRX[0] = i;}
    }
  #endif
  uint8_t SerialRead(uint8_t port) {
    uint8_t c = serialBufferRX[serialTailRX[port]][port];
    if ((serialHeadRX[port] != serialTailRX[port])) serialTailRX[port] = serialTailRX[port] + 1;
    return c;
  }
  uint8_t SerialAvailable(uint8_t port) {
    return serialHeadRX[port] - serialTailRX[port];
  }
  void SerialWrite(uint8_t port,uint8_t c){
    if(port == 0){
      serialize8(c);
    }
  } 

#endif

