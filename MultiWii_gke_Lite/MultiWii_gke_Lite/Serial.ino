
#define UART_NUMBER 2

#define RX_BUFFER_SIZE 64
#define TX_BUFFER_SIZE 128
#define INBUF_SIZE 64

static volatile uint8_t serialHeadRX[UART_NUMBER],serialTailRX[UART_NUMBER];
static uint8_t serialBufferRX[RX_BUFFER_SIZE][UART_NUMBER];
static volatile uint8_t serialHeadTX[UART_NUMBER],serialTailTX[UART_NUMBER];
static uint8_t serialBufferTX[TX_BUFFER_SIZE][UART_NUMBER];
static uint8_t inBuf[INBUF_SIZE][UART_NUMBER];

// Multiwii Serial Protocol 0 
#define MSP_VERSION              0

//to multiwii developpers/committers : do not add new MSP messages without a proper argumentation/agreement on the forum
#define MSP_IDENT                100   //out message         multitype + multiwii version + protocol version + capability variable
#define MSP_STATUS               101   //out message         cycletime & errors_count & sensor present & box activation & current setting number
#define MSP_RAW_IMU              102   //out message         9 DOF
#define MSP_SERVO                103   //out message         8 servos
#define MSP_MOTOR                104   //out message         8 motors
#define MSP_RC                   105   //out message         8 rc chan and more
#define MSP_RAW_GPS              106   //out message         fix, numsat, lat, lon, alt, speed, ground course
#define MSP_COMP_GPS             107   //out message         distance home, direction home
#define MSP_ATTITUDE             108   //out message         2 angles 1 heading
#define MSP_ALTITUDE             109   //out message         altitude, variometer
#define MSP_ANALOG               110   //out message         vbat, powermetersum, rssi if available on RX
#define MSP_RC_TUNING            111   //out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_PID                  112   //out message         P I D coeff (9 are used currently)
#define MSP_BOX                  113   //out message         BOX setup (number is dependant of your setup)
#define MSP_MISC                 114   //out message         powermeter trig
#define MSP_MOTOR_PINS           115   //out message         which pins are in use for motors & servos, for GUI 
#define MSP_BOXNAMES             116   //out message         the aux switch names
#define MSP_PIDNAMES             117   //out message         the PID names
#define MSP_WP                   118   //out message         get a WP, WP# is in the payload, returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold
#define MSP_BOXIDS               119   //out message         get the permanent IDs associated to BOXes

#define MSP_SET_RAW_RC           200   //in message          8 rc chan
#define MSP_SET_RAW_GPS          201   //in message          fix, numsat, lat, lon, alt, speed
#define MSP_SET_PID              202   //in message          P I D coeff (9 are used currently)
#define MSP_SET_BOX              203   //in message          BOX setup (number is dependant of your setup)
#define MSP_SET_RC_TUNING        204   //in message          rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_ACC_CALIBRATION      205   //in message          no param
#define MSP_MAG_CALIBRATION      206   //in message          no param
#define MSP_SET_MISC             207   //in message          powermeter trig + 8 free for future use
#define MSP_RESET_CONF           208   //in message          no param
#define MSP_SET_WP               209   //in message          sets a given WP (WP#,lat, lon, alt, flags)
#define MSP_SELECT_SETTING       210   //in message          Select Setting Number (0-2)
#define MSP_SET_HEAD             211   //in message          define a new heading hold direction

#define MSP_BIND                 240   //in message          no param

#define MSP_EEPROM_WRITE         250   //in message          no param

#define MSP_DEBUGMSG             253   //out message         debug string buffer
#define MSP_DEBUG                254   //out message         debug1,debug2,debug3,debug4


static uint8_t checksum[UART_NUMBER];
static uint8_t indRX[UART_NUMBER];
static uint8_t cmdMSP[UART_NUMBER];

#if defined(PROMINI)
#define CURRENTPORT 0
#else
static uint8_t CURRENTPORT=0;
#endif

uint32_t read32() {
  uint32_t t = read16();
  t+= (uint32_t)read16()<<16;
  return t;
}
uint16_t read16() {
  uint16_t t = read8();
  t+= (uint16_t)read8()<<8;
  return t;
}
uint8_t read8()  {
  return inBuf[indRX[CURRENTPORT]++][CURRENTPORT]&0xff;
}

void headSerialResponse(uint8_t err, uint8_t s) {
  serialize8('$');
  serialize8('M');
  serialize8(err ? '!' : '>');
  checksum[CURRENTPORT] = 0; // start calculating a new checksum
  serialize8(s);
  serialize8(cmdMSP[CURRENTPORT]);
}

void headSerialReply(uint8_t s) {
  headSerialResponse(0, s);
}

void inline headSerialError(uint8_t s) {
  headSerialResponse(1, s);
}

void tailSerialReply() {
  serialize8(checksum[CURRENTPORT]);
  UartSendData();
}

void serializeNames(PGM_P s) {
  for (PGM_P c = s; pgm_read_byte(c); c++) {
    serialize8(pgm_read_byte(c));
  }
}

void serialCom() {
  uint8_t c,n;  
  static uint8_t offset[UART_NUMBER];
  static uint8_t dataSize[UART_NUMBER];
  static enum _serial_state {
    IDLE,
    HEADER_START,
    HEADER_M,
    HEADER_ARROW,
    HEADER_SIZE,
    HEADER_CMD,
  } 
  c_state[UART_NUMBER];// = IDLE;

  for(n=0;n<UART_NUMBER;n++) {
    CURRENTPORT=n;

#define SPEK_COND
#if defined(SPEKTRUM) && (UART_NUMBER > 1)
#define SPEK_COND  && (SPEK_SERIAL_PORT != CURRENTPORT)
#endif

    while (SerialAvailable(CURRENTPORT) SPEK_COND) {
      uint8_t bytesTXBuff = ((uint8_t)(serialHeadTX[CURRENTPORT]-serialTailTX[CURRENTPORT]))%TX_BUFFER_SIZE; // indicates the number of occupied bytes in TX buffer
      if (bytesTXBuff < TX_BUFFER_SIZE - 50 ) { // ensure there is enough free TX buffer to go further (50 bytes margin)
        c = SerialRead(CURRENTPORT);
        // regular data handling to detect and handle MSP and other data
        if (c_state[CURRENTPORT] == IDLE)
          c_state[CURRENTPORT] = (c=='$') ? HEADER_START : IDLE;
        else if (c_state[CURRENTPORT] == HEADER_START) 
          c_state[CURRENTPORT] = (c=='M') ? HEADER_M : IDLE;
        else if (c_state[CURRENTPORT] == HEADER_M)
          c_state[CURRENTPORT] = (c=='<') ? HEADER_ARROW : IDLE; 
        else if (c_state[CURRENTPORT] == HEADER_ARROW) {
          if (c > INBUF_SIZE) {  // now we are expecting the payload size
            c_state[CURRENTPORT] = IDLE;
            continue;
          }
          dataSize[CURRENTPORT] = c;
          offset[CURRENTPORT] = 0;
          checksum[CURRENTPORT] = 0;
          indRX[CURRENTPORT] = 0;
          checksum[CURRENTPORT] ^= c;
          c_state[CURRENTPORT] = HEADER_SIZE;  // the command is to follow
        } 
        else if (c_state[CURRENTPORT] == HEADER_SIZE) {
          cmdMSP[CURRENTPORT] = c;
          checksum[CURRENTPORT] ^= c;
          c_state[CURRENTPORT] = HEADER_CMD;
        } 
        else if (c_state[CURRENTPORT] == HEADER_CMD && offset[CURRENTPORT] < dataSize[CURRENTPORT]) {
          checksum[CURRENTPORT] ^= c;
          inBuf[offset[CURRENTPORT]++][CURRENTPORT] = c;
        } 
        else if (c_state[CURRENTPORT] == HEADER_CMD && offset[CURRENTPORT] >= dataSize[CURRENTPORT]) {
          if (checksum[CURRENTPORT] == c) // compare calculated and transferred checksum
            evaluateCommand();  // we got a valid packet, evaluate it
          c_state[CURRENTPORT] = IDLE;
        }
      }
    }
  }
} // serialCom


void evaluateCommand() { 
  uint8_t len;
  
  switch(cmdMSP[CURRENTPORT]) {
  case MSP_SET_RAW_RC:
    for(uint8_t i=0;i<8;i++) 
      rcData[i] = read16();
    headSerialReply(0);
    break;
  case MSP_SET_PID:
    for(uint8_t i=0;i<PIDITEMS;i++) {
      conf.P8[i]=read8();
      conf.I8[i]=read8();
      conf.D8[i]=read8();
    }
    headSerialReply(0);
    break;
  case MSP_SET_BOX:
    for(uint8_t i=0;i<CHECKBOXITEMS;i++)
      conf.activate[i]=read16();
    headSerialReply(0);
    break;
  case MSP_SET_RC_TUNING:
    conf.rcRate8 = read8();
    conf.rcExpo8 = read8();
    conf.rollPitchRate = read8();
    conf.yawRate = read8();
    conf.dynThrPID = read8();
    conf.thrMid8 = read8();
    conf.thrExpo8 = read8();
    headSerialReply(0);
    break;
  case MSP_SET_MISC:
    headSerialReply(0);
    break;
  case MSP_SET_HEAD:
    magHold = read16();
    headSerialReply(0);
    break;
  case MSP_IDENT:
    headSerialReply(7);
    serialize8(VERSION);   // multiwii version
    serialize8(MULTITYPE); // type of multicopter
    serialize8(MSP_VERSION); // MultiWii Serial Protocol Version
    serialize32(0);//pgm_read_dword(&(capability))); // "capability"
    break;
  case MSP_STATUS:
    headSerialReply(11);
    serialize16(cycleTime);
    serialize16(i2c_errors_count);
    serialize16(f.ACC_CALIBRATED); // ACC
    serialize32(

    f.ANGLE_MODE<<BOXANGLE|
      f.HORIZON_MODE<<BOXHORIZON|
      f.ARMED<<BOXARM);
    serialize8(global_conf.currentSet);   // current setting
    break;
  case MSP_RAW_IMU:
    headSerialReply(18);
    for(uint8_t i=0;i<3;i++) serialize16(accSmooth[i]);
    for(uint8_t i=0;i<3;i++) serialize16(gyroData[i]);
    for(uint8_t i=0;i<3;i++) serialize16(magADC[i]);
    break;
  case MSP_SERVO:
    headSerialReply(16);
    for(uint8_t i=0;i<8;i++)
      serialize16(0);
    break;
  case MSP_MOTOR:
    headSerialReply(16);
    for(uint8_t i=0;i<8;i++)
      serialize16( (i < NUMBER_MOTOR) ? motor[i] : 0 );
    break;
  case MSP_RC:
    headSerialReply(RC_CHANS * 2);
    for(uint8_t i=0;i<RC_CHANS;i++) serialize16(rcData[i]);
    break;
  case MSP_ATTITUDE:
    headSerialReply(8);
    for(uint8_t i=0;i<2;i++) serialize16(angle[i]);
    serialize16(heading);
    serialize16(headFreeModeHold);
    break;
  case MSP_ALTITUDE:
    headSerialReply(6);
    serialize32(EstAlt);
    serialize16(vario);                  // added since r1172
    break;
  case MSP_ANALOG:
    headSerialReply(5);
    serialize8(vbat);
    serialize16(intPowerMeterSum);
    serialize16(rssi);
    break;
  case MSP_RC_TUNING:
    headSerialReply(7);
    serialize8(conf.rcRate8);
    serialize8(conf.rcExpo8);
    serialize8(conf.rollPitchRate);
    serialize8(conf.yawRate);
    serialize8(conf.dynThrPID);
    serialize8(conf.thrMid8);
    serialize8(conf.thrExpo8);
    break;
  case MSP_PID:
    headSerialReply(3*PIDITEMS);
    for(uint8_t i=0;i<PIDITEMS;i++) {
      serialize8(conf.P8[i]);
      serialize8(conf.I8[i]);
      serialize8(conf.D8[i]);
    }
    break;
  case MSP_PIDNAMES:
    headSerialReply(strlen_P(pidnames));
    serializeNames(pidnames);
    break;
  case MSP_BOX:
    headSerialReply(2*CHECKBOXITEMS);
    for(uint8_t i=0;i<CHECKBOXITEMS;i++)
      serialize16(conf.activate[i]);
    break;
  case MSP_BOXNAMES:
    headSerialReply(strlen_P(boxnames));
    serializeNames(boxnames);
    break;
  case MSP_BOXIDS:
    headSerialReply(CHECKBOXITEMS);
    for(uint8_t i=0;i<CHECKBOXITEMS;i++)
      serialize8(pgm_read_byte(&(boxids[i])));
    break;
  case MSP_MISC:
    headSerialReply(2);
    serialize16(intPowerTrigger1);
    break;
  case MSP_MOTOR_PINS:
    headSerialReply(8);
    for(uint8_t i=0;i<8;i++)
      serialize8(PWM_PIN[i]);
    break;
  case MSP_RESET_CONF:
    if(!f.ARMED) LoadDefaults();
    headSerialReply(0);
    break;
  case MSP_ACC_CALIBRATION:
    if(!f.ARMED) calibratingA=512;
    headSerialReply(0);
    break;
  case MSP_MAG_CALIBRATION:
    if(!f.ARMED) f.CALIBRATE_MAG = true;
    headSerialReply(0);
    break;
  case MSP_EEPROM_WRITE:
    writeParams(0);
    headSerialReply(0);
    break;
  case MSP_DEBUG:
    headSerialReply(8);
    for(uint8_t i=0;i<4;i++)
      serialize16(debug[i]); // 4 variables are here for general monitoring purpose
    break;
  default:  // we do not know how to handle the (valid) message, indicate error MSP $M!
    headSerialError(0);
    break;
  }
  tailSerialReply();
} // evaluatCommand

// *******************************************************
// Interrupt driven UART transmitter - using a ring buffer
// *******************************************************

void serialize32(uint32_t a) {
  serialize8((a    ) & 0xFF);
  serialize8((a>> 8) & 0xFF);
  serialize8((a>>16) & 0xFF);
  serialize8((a>>24) & 0xFF);
}

void serialize16(int16_t a) {
  serialize8((a   ) & 0xFF);
  serialize8((a>>8) & 0xFF);
}

void serialize8(uint8_t a) {
  uint8_t t = serialHeadTX[CURRENTPORT];
  if (++t >= TX_BUFFER_SIZE) t = 0;
  serialBufferTX[t][CURRENTPORT] = a;
  checksum[CURRENTPORT] ^= a;
  serialHeadTX[CURRENTPORT] = t;
}

ISR(USART1_UDRE_vect) { // Serial 1 on a MEGA or on a PROMICRO
  uint8_t t = serialTailTX[1];
  if (serialHeadTX[1] != t) {
    if (++t >= TX_BUFFER_SIZE) t = 0;
    UDR1 = serialBufferTX[t][1];  // Transmit next byte in the ring
    serialTailTX[1] = t;
  }
  if (t == serialHeadTX[1]) UCSR1B &= ~(1<<UDRIE1);
}


void UartSendData(void) {

  switch (CURRENTPORT) {
  case 0:
    while(serialHeadTX[0] != serialTailTX[0]) {
      if (++serialTailTX[0] >= TX_BUFFER_SIZE) serialTailTX[0] = 0;
      USB_Send(USB_CDC_TX,serialBufferTX[serialTailTX[0]],1);
    }
    break;
  case 1: 
    UCSR1B |= (1<<UDRIE1); 
    break;
  }
} // UartSendData

static void inline SerialOpen(uint8_t port, uint32_t baud) {
  uint8_t h = ((F_CPU  / 4 / baud -1) / 2) >> 8;
  uint8_t l = ((F_CPU  / 4 / baud -1) / 2);
  switch (port) {
#if (ARDUINO >= 100) 
  case 0: 
    UDIEN &= ~(1<<SOFE); 
    break;// disable the USB frame interrupt of arduino (it causes strong jitter and we dont need it)
#endif
  case 1: 
    UCSR1A  = (1<<U2X1); 
    UBRR1H = h; 
    UBRR1L = l; 
    UCSR1B |= (1<<RXEN1)|(1<<TXEN1)|(1<<RXCIE1); 
    break;
  }
}

static void inline SerialEnd(uint8_t port) {
  switch (port) {
  case 1: 
    UCSR1B &= ~((1<<RXEN1)|(1<<TXEN1)|(1<<RXCIE1)|(1<<UDRIE1)); 
    break;
  }
}

static void inline store_uart_in_buf(uint8_t data, uint8_t portnum) {
#if defined(SPEKTRUM)
  if (portnum == SPEK_SERIAL_PORT) {
    if (!spekFrameFlags) { 
      sei();
      uint32_t spekTimeNow = (timer0_overflow_count << 8) * (64 / clockCyclesPerMicrosecond()); //Move timer0_overflow_count into registers so we don't touch a volatile twice
      uint32_t spekInterval = spekTimeNow - spekTimeLast;                                       //timer0_overflow_count will be slightly off because of the way the Arduino core timer interrupt handler works; that is acceptable for this use. Using the core variable avoids an expensive call to millis() or micros()
      spekTimeLast = spekTimeNow;
      if (spekInterval > 5000) {  //Potential start of a Spektrum frame, they arrive every 11 or every 22 ms. Mark it, and clear the buffer. 
        serialTailRX[portnum] = 0;
        serialHeadRX[portnum] = 0;
        spekFrameFlags = 0x01;
      }
      cli();
    }
  }
#endif

  uint8_t h = serialHeadRX[portnum];
  if (++h >= RX_BUFFER_SIZE) h = 0;
  if (h == serialTailRX[portnum]) return; // we did not bite our own tail?
  serialBufferRX[serialHeadRX[portnum]][portnum] = data;  
  serialHeadRX[portnum] = h;
} // store_uart_in_buf

ISR(USART1_RX_vect)  { 
  store_uart_in_buf(UDR1, 1); 
}


uint8_t SerialRead(uint8_t port) {

#if (ARDUINO >= 100)
  if(port == 0) USB_Flush(USB_CDC_TX);
#endif
  if(port == 0) return USB_Recv(USB_CDC_RX);      

  uint8_t t = serialTailRX[port];
  uint8_t c = serialBufferRX[t][port];
  if (serialHeadRX[port] != t) {
    if (++t >= RX_BUFFER_SIZE) t = 0;
    serialTailRX[port] = t;
  }
  return c;
}

#if defined(SPEKTRUM)
uint8_t SerialPeek(uint8_t port) {
  uint8_t c = serialBufferRX[serialTailRX[port]][port];
  if ((serialHeadRX[port] != serialTailRX[port])) return c; 
  else return 0;
}
#endif

uint8_t SerialAvailable(uint8_t port) {

  if(port == 0) return USB_Available(USB_CDC_RX);

  return (serialHeadRX[port] - serialTailRX[port])%RX_BUFFER_SIZE;
}

void SerialWrite(uint8_t port,uint8_t c){
  CURRENTPORT=port;
  serialize8(c);
  UartSendData();
} // SerialWrite








