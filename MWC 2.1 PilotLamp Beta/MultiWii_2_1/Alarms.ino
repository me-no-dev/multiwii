static uint8_t cycle_Done[5]={0,0,0,0,0}, 
               channelIsOn[5] = {0,0,0,0,0};
static uint32_t channelLastToggleTime[5] ={0,0,0,0,0};
#if defined(BUZZER)
  static uint8_t beeperOnBox = 0,
                 warn_noGPSfix = 0,
                 warn_failsafe = 0, 
                 warn_runtime = 0,
                 warn_vbat = 0,
                 buzzerSequenceActive=0;
  uint8_t isBuzzerON() { return channelIsOn[1]; } // returns true while buzzer is buzzing; returns 0 for silent periods

/********************************************************************/
/****                      Alarm Handling                        ****/
/********************************************************************/
  void alarmHandler(){
    
    #if defined(VBAT)
      if ( ( (vbat>VBATLEVEL1_3S) 
      #if defined(POWERMETER)
                           && ( (pMeter[PMOTOR_SUM] < pAlarm) || (pAlarm == 0) )
      #endif
                         )  || (NO_VBAT>vbat)                              ) // ToLuSe
      {                                          // VBAT ok AND powermeter ok, alarm off
        warn_vbat = 0;
      #if defined(POWERMETER)
      } else if (pMeter[PMOTOR_SUM] > pAlarm) {                             // sound alarm for powermeter
        warn_vbat = 4;
      #endif
      } else if (vbat>VBATLEVEL2_3S) warn_vbat = 1;
      else if (vbat>VBATLEVEL3_3S)   warn_vbat = 2;
      else                           warn_vbat = 4;
    #endif
 
    if ( rcOptions[BOXBEEPERON] )beeperOnBox = 1;
    else beeperOnBox = 0;
    
    #if defined(RCOPTIONSBEEP)
      static uint8_t i = 0,firstrun = 1, last_rcOptions[CHECKBOXITEMS];
                    
      if (last_rcOptions[i] != rcOptions[i])beep_toggle = 1;
        last_rcOptions[i] = rcOptions[i]; 
        i++;
      if(i >= CHECKBOXITEMS)i=0;
      
      if(firstrun == 1 && beep_confirmation == 0){
        beep_toggle = 0;    //only enable options beep AFTER gyro init
        beeperOnBox = 0;
      }        
      else firstrun = 0;
       
    #endif  
     
    #if defined(FAILSAFE)
      if ( failsafeCnt > (5*FAILSAVE_DELAY) && f.ARMED) {
        warn_failsafe = 1;                                                                   //set failsafe warning level to 1 while landing
        if (failsafeCnt > 5*(FAILSAVE_DELAY+FAILSAVE_OFF_DELAY)) warn_failsafe = 2;          //start "find me" signal after landing   
      }
      if ( failsafeCnt > (5*FAILSAVE_DELAY) && !f.ARMED) warn_failsafe = 2;                  // tx turned off while motors are off: start "find me" signal
      if ( failsafeCnt == 0) warn_failsafe = 0;                                              // turn off alarm if TX is okay
    #endif
    
    #if GPS
      if ((rcOptions[BOXGPSHOME] || rcOptions[BOXGPSHOLD]) && !f.GPS_FIX)warn_noGPSfix = 1;  
      else warn_noGPSfix = 0;
    #endif

    #if defined(ARMEDTIMEWARNING)
      if (armedTime >= ArmedTimeWarningMicroSeconds)warn_runtime = 1;
    #endif

    //it is neccessary to switch beetween buzzer or led
    static uint8_t alarmchannel = 0;
    switch(alarmchannel){
      case 0:
        buzzerHandler();
        #if defined(PILOTLAMP)
            alarmchannel++;
            break;
          case 1:
            PilotLampHandler();
            alarmchannel = 0;
            break;
        #endif
    }
    
  }

  void buzzerHandler(){ 
    /********************************************************************/
    /****                      Buzzer Handling                       ****/
    /********************************************************************/  
    // beepcode(length1,length2,length3,pause)
    //D: Double, L: Long, M: Middle, S: Short, N: None
    if (warn_failsafe == 2)      beep_code('L','N','N','D');                 //failsafe "find me" signal
    else if (warn_failsafe == 1) beep_code('S','L','L','S');                 //failsafe landing active              
    else if (beep_toggle == 1) {beep_code('S','N','N','N');      } 
    else if (beep_toggle == 2)    beep_code('S','S','N','N');       
    else if (beep_toggle > 2)     beep_code('S','S','S','N');         
    else if (warn_noGPSfix == 1) beep_code('S','S','N','S');    
    else if (beeperOnBox == 1)   beep_code('S','S','S','S');                 //beeperon
    else if (warn_runtime == 1 && f.ARMED == 1)beep_code('S','S','S','N'); //Runtime warning      
    else if (warn_vbat == 4)     beep_code('S','S','L','D');       
    else if (warn_vbat == 2)     beep_code('S','L','N','D');       
    else if (warn_vbat == 1)     beep_code('L','N','N','D'); 
    else if (beep_confirmation == 1) beep_code('L','N','N','L');    
    else if (beep_confirmation == 2) beep_code('L','L','N','L');   
    else if (beep_confirmation == 3) beep_code('L','L','L','L');
    else if (beep_confirmation == 4) beep_code('L','M','S','N');
    else if (beep_confirmation > 4) beep_code('L','L','L','L');
    else if (buzzerSequenceActive == 1) beep_code('N','N','N','N');                //if no signal is needed, finish sequence if not finished yet
    else{                                                                   //reset everything and keep quiet
      channelIsOn[1] = 0;
      BUZZERPIN_OFF;
    }  
  }
  void beep_code(char first, char second, char third, char pause){
    static char patternChar[4];
    uint16_t Duration;
    static uint8_t icnt = 0;
    
    if (buzzerSequenceActive == 0){    //only change sequenceparameters if prior sequence is done
      buzzerSequenceActive = 1;
      patternChar[0] = first; 
      patternChar[1] = second;
      patternChar[2] = third;
      patternChar[3] = pause;
    }
    switch(patternChar[icnt]) {
      case 'L': 
        Duration = 200; 
        break;
      case 'M': 
        Duration = 120; 
        break;
      case 'D': 
        Duration = 2000; 
        break;
      case 'N': 
        Duration = 0; 
        break;
      default:
        Duration = 50; 
        break;
    }
    if(icnt <3 && Duration!=0){
      useResource('S',Duration,50);
    }
    if (icnt >=3 && (channelLastToggleTime[1]<millis()-Duration) ){
      icnt=0;
      if (beep_toggle)beep_toggle = 0;
      if (beep_confirmation)beep_confirmation = 0;
      buzzerSequenceActive = 0;                              //sequence is now done, next sequence may begin
      channelIsOn[1] = 0;
      BUZZERPIN_OFF;
      return;
    }
    if (cycle_Done[1] == 1 || Duration == 0){
      if (icnt < 3){icnt++;}   
      channelIsOn[1] = 0;
      cycle_Done[1] = 0;
      BUZZERPIN_OFF;
    }  
  }  
  
    
#endif  //end of buzzer define

#if defined (PILOTLAMP) 
/********************************************************************/
/****                   Pilot Lamp Handling                      ****/
/********************************************************************/
  void PilotLampHandler(){
    static int16_t  i2c_errors_count_old = 0;
    static uint8_t channel = 0;
    //executing more than one channel per cycle leads to severe peaks in cycle time
    //==================I2C Error ===========================
    if (i2c_errors_count > i2c_errors_count_old+100){
      PilotLampBlinkAll(100);
    }else if (beeperOnBox){
    //==================LED Sequence ===========================
      PilotLampSequence(100);
    }else{
      switch(channel) {                          // only use one channel per cycle
        case 0:
          //==================GREEN LED===========================
          if (f.ARMED && f.ACC_MODE) useResource('G',1000,500);
          else if (f.ARMED) useResource('G',100,100);
          else useResource('G',0,0);    //switch off  --> muss noch programmiert werden
          channel++;
          break;
        case 1:
          //==================BLUE LED===========================
          #if GPS
            if (!f.GPS_FIX) useResource('B',100,100);
            else if (rcOptions[BOXGPSHOME] || rcOptions[BOXGPSHOLD]) useResource('B',1000,1000);
            else useResource('B',100,1000);
          #else
            useResource('B',0,0);
          #endif   
          channel++;
          break;
        case 2:
          //==================RED LED===========================
          if (warn_failsafe==1)useResource('R',100,100);
          else if (warn_failsafe==2)useResource('R',1000,2000);
          else useResource('R',0,0);
          channel=0;
          break;
       }     
     }
   }
    
  void PilotLampSequence(uint16_t speed){
    static uint32_t lastswitch = 0;
    static uint8_t state = 0;
       
    if(millis() >= (lastswitch + speed)) {                                
      lastswitch = millis();
      state++;
    }
    switch(state) {                          // Light-chase Cycle the LED's.
      case 0:
        PilotLamp(PL_GRN_ON);
        break;
      case 1:
         PilotLamp(PL_GRN_OFF);
         PilotLamp(PL_BLU_ON);
         break;
      case 2:
        PilotLamp(PL_BLU_OFF);
        PilotLamp(PL_RED_ON);
        break;
      case 3:
        state = 0;
        PilotLamp(PL_RED_OFF);
        break;
    }             
  return;
  }
   
  void PilotLampBlinkAll(uint16_t speed){
    static uint32_t lastswitch = 0;
    static uint8_t state = 0;
       
    if(millis() >= (lastswitch + speed)) {                                
      lastswitch = millis();
      state++;
    }
    switch(state) {                         
      case 0:
        PilotLamp(PL_GRN_ON);
        PilotLamp(PL_BLU_ON);
        PilotLamp(PL_RED_ON);
        break;
      case 1:
        PilotLamp(PL_GRN_OFF);
        PilotLamp(PL_BLU_OFF);
        PilotLamp(PL_RED_OFF);
        state = 0;
        break;
    }             
  return;
  } 
  
  void PilotLampTest(void){
     static uint8_t cam1 = 0;
     static uint8_t cam2 = 0;
     static uint8_t state = 0;
     
     if(cam1++ > 40) {                                // Update rate is perhaps 2Hz, but will vary.
         cam1 = 0;
         if(cam2++ > 20 && state==1) {                // Periodically beep the buzzer (every few seconds).
             cam2 = 0;
             PilotLamp(PL_BZR_ON);
             delay(200);                              // Brute force delay is needed because main loop will immediately turn off buzzer.
         }
         else {
             switch(state) {                          // Light-chase Cycle the LED's.
                 case 0:
                     state++;
                     PilotLamp(PL_BZR_OFF);
                     break;
                 case 1:
                     state++;
                     PilotLamp(PL_GRN_ON);
                     break;
                 case 2:
                     state++;
                     PilotLamp(PL_GRN_OFF);
                     PilotLamp(PL_BLU_ON);
                     break;
                 case 3:
                     state++;
                     PilotLamp(PL_BLU_OFF);
                     PilotLamp(PL_RED_ON);
                     break;
                 case 4:
                     state = 0;
                     PilotLamp(PL_RED_OFF);
                     break;
             }
        }             
     }
    return;
  } 
  
  union pl_reg {
      struct {
          unsigned grn: 1;
          unsigned blu: 1;
          unsigned red: 1;
          unsigned bzr: 1;
      } ctrl;
      uint8_t pl_ctrl_all;
  };
  
  void PilotLamp(uint16_t device){
      uint8_t i;
      static union pl_reg mode;
      
      if(device == PL_INIT) {                             // Initialize the Pilot Lamp State Table, turn off LEDs and Buzzer.
          mode.pl_ctrl_all = 0x00;                        // Reset all LED and Buzzer status states.
          gen_pl_freq(PL_GRN_OFF);                        // Turn off Green LED.
          gen_pl_freq(PL_BLU_OFF);                        // Turn off Blue LED.
          gen_pl_freq(PL_RED_OFF);                        // Turn off Red LED.
          gen_pl_freq(PL_BZR_OFF);                        // Turn off Buzzer.
      }
      else {                                              // Check to see if the new state request is different than the current state.
          if(device==PL_GRN_OFF && mode.ctrl.grn==1) {
              mode.ctrl.grn = 0;
          }
          else if(device==PL_GRN_ON && mode.ctrl.grn==0) {
              mode.ctrl.grn = 1;
          }
          else if(device==PL_BLU_OFF && mode.ctrl.blu==1) {
              mode.ctrl.blu = 0;
          }
          else if(device==PL_BLU_ON && mode.ctrl.blu==0) {
              mode.ctrl.blu = 1;
          }
          else if(device==PL_RED_OFF && mode.ctrl.red==1) {
              mode.ctrl.red = 0;
          }
          else if(device==PL_RED_ON && mode.ctrl.red==0) {
              mode.ctrl.red = 1;
          }
          else if(device==PL_BZR_OFF && mode.ctrl.bzr==1) {
              mode.ctrl.bzr = 0;
          }
          else if(device==PL_BZR_ON && mode.ctrl.bzr==0) {
              mode.ctrl.bzr = 1;
          }
          else {                                          // No state changes
              PL_PIN_OFF;
              return;                                     // Skip signal generation.
          }
          
          gen_pl_freq(device);                            // Send waveform to Pilot Lamp.
      }
      
      return;
  }
  
  // Create the freq signal to activate Pilot Lamp. This is a bit-bang operation.
  void gen_pl_freq(uint16_t device)
  {
      uint8_t i;
      
      for (i=0;i<4;i++) {                                // Four waveforms are required. maybe three??
          PL_PIN_ON;
          delayMicroseconds(device);
          PL_PIN_OFF;
          delayMicroseconds(device);
      }
      
      return;
  }
#endif

/********************************************************************/
/****                         LED Handling                       ****/
/********************************************************************/
//Beware!! Is working with delays, do not use inflight!

void blinkLED(uint8_t num, uint8_t ontime,uint8_t repeat) {
  uint8_t i,r;
  for (r=0;r<repeat;r++) {
    for(i=0;i<num;i++) {
      #if defined(LED_FLASHER)
        switch_led_flasher(1);
      #endif
      #if defined(LANDING_LIGHTS_DDR)
        switch_landing_lights(1);
      #endif
      LEDPIN_TOGGLE; // switch LEDPIN state
      delay(ontime);
      #if defined(LED_FLASHER)
        switch_led_flasher(0);
      #endif
      #if defined(LANDING_LIGHTS_DDR)
        switch_landing_lights(0);
      #endif
    }
    delay(60); //wait 60 ms
  }
}

/********************************************************************/
/****                         Global Handling                    ****/
/********************************************************************/

  int useResource(char resource, uint16_t pulse, uint16_t pause){ 
    static uint8_t channel = 0; 
    channel = ResourceToChannel(resource);
    if (!channelIsOn[channel] && (millis() >= (channelLastToggleTime[channel] + pause))&& pulse != 0) {	         
      channelIsOn[channel] = 1;      
      ChannelToOutput(channel,1);
      channelLastToggleTime[channel]=millis();      
    } else if (channelIsOn[channel] && (millis() >= channelLastToggleTime[channel] + pulse)|| pulse==0 ) {        
      channelIsOn[channel] = 0;
      ChannelToOutput(channel,0);
      channelLastToggleTime[channel]=millis();
      cycle_Done[channel] = 1;     
    } 
  } 
  
  int ResourceToChannel(uint8_t resource){
    uint8_t channel =0;
    switch(resource) {
      case 'L': 
        channel = 0;
        break;
      case 'S': 
        channel = 1;
        break;
      case 'G': 
        channel = 2;
        break;
      case 'B': 
        channel = 3;
        break;
      case 'R': 
        channel = 4;
        break;
      default:
        channel = 0;
        break;
    }
    return channel;
  }
  
  void ChannelToOutput(uint8_t channel, uint8_t activate){
     switch(channel) {
        case 0: 
          if (activate == 1) {LEDPIN_ON;}
          else {LEDPIN_OFF;}
          break; 
        case 1:
          if (activate == 1) {BUZZERPIN_ON;}
          else {BUZZERPIN_OFF;}
          break; 
        #if defined (PILOTLAMP) 
          case 2:
            if (activate == 1)PilotLamp(PL_GRN_ON);
            else PilotLamp(PL_GRN_OFF);
            break;
          case 3: 
            if (activate == 1)PilotLamp(PL_BLU_ON);
            else PilotLamp(PL_BLU_OFF);
            break;
          case 4: 
            if (activate == 1)PilotLamp(PL_RED_ON);
            else PilotLamp(PL_RED_OFF);
            break;
        #endif
        default:
          if (activate == 1){LEDPIN_ON;}
          else {LEDPIN_OFF;}
          break;
      }
      return;
  }


/********************************************************************/
/****                      LED Ring Handling                     ****/
/********************************************************************/

#if defined(LED_RING)
  #define LED_RING_ADDRESS 0x6D //7 bits
  
  void i2CLedRingState() {
    uint8_t b[10];
    static uint8_t state;
    
    if (state == 0) {
      b[0]='z'; 
      b[1]= (180-heading)/2; // 1 unit = 2 degrees;
      i2c_rep_start(LED_RING_ADDRESS<<1);
      for(uint8_t i=0;i<2;i++)
        i2c_write(b[i]);
      i2c_stop();
      state = 1;
    } else if (state == 1) {
      b[0]='y'; 
      b[1]= constrain(angle[ROLL]/10+90,0,180);
      b[2]= constrain(angle[PITCH]/10+90,0,180);
      i2c_rep_start(LED_RING_ADDRESS<<1);
      for(uint8_t i=0;i<3;i++)
        i2c_write(b[i]);
      i2c_stop();
      state = 2;
    } else if (state == 2) {
      b[0]='d'; // all unicolor GREEN 
      b[1]= 1;
      if (f.ARMED) b[2]= 1; else b[2]= 0;
      i2c_rep_start(LED_RING_ADDRESS<<1);
      for(uint8_t i=0;i<3;i++)
        i2c_write(b[i]);
      i2c_stop();
      state = 0;
    }
  }
  
  void blinkLedRing() {
    uint8_t b[3];
    b[0]='k';
    b[1]= 10;
    b[2]= 10;
    i2c_rep_start(LED_RING_ADDRESS<<1);
    for(uint8_t i=0;i<3;i++)
      i2c_write(b[i]);
    i2c_stop();
  
  }
#endif


/********************************************************************/
/****                    LED flasher Handling                    ****/
/********************************************************************/

#if defined(LED_FLASHER)
  static uint8_t led_flasher_sequence = 0;
  /* if we load a specific sequence and do not want it change, set this flag */
  static enum {
  	LED_FLASHER_AUTO,
  	LED_FLASHER_CUSTOM
  } led_flasher_control = LED_FLASHER_AUTO;
  
  void init_led_flasher() {
    LED_FLASHER_DDR |= (1<<LED_FLASHER_BIT);
    LED_FLASHER_PORT &= ~(1<<LED_FLASHER_BIT);
  }
  
  void led_flasher_set_sequence(uint8_t s) {
    led_flasher_sequence = s;
  }
  
  void inline switch_led_flasher(uint8_t on) {
    if (on) {
      LED_FLASHER_PORT |= (1<<LED_FLASHER_BIT);
    } else {
      LED_FLASHER_PORT &= ~(1<<LED_FLASHER_BIT);
    }
  }
  
  void auto_switch_led_flasher() {
    uint8_t seg = (currentTime/1000/125)%8;
    if (led_flasher_sequence & 1<<seg) {
      switch_led_flasher(1);
    } else {
      switch_led_flasher(0);
    }
  }
  
  /* auto-select a fitting sequence according to the
   * copter situation
   */
  void led_flasher_autoselect_sequence() {
    if (led_flasher_control != LED_FLASHER_AUTO) return;
    #if defined(LED_FLASHER_SEQUENCE_MAX)
    /* do we want the complete illumination no questions asked? */
    if (rcOptions[BOXLEDMAX]) {
    #else
    if (0) {
    #endif
      led_flasher_set_sequence(LED_FLASHER_SEQUENCE_MAX);
    } else {
      /* do we have a special sequence for armed copters? */
      #if defined(LED_FLASHER_SEQUENCE_ARMED)
      led_flasher_set_sequence(f.ARMED ? LED_FLASHER_SEQUENCE_ARMED : LED_FLASHER_SEQUENCE);
      #else
      /* Let's load the plain old boring sequence */
      led_flasher_set_sequence(LED_FLASHER_SEQUENCE);
      #endif
    }
  }
  
  #endif
  
  #if defined(LANDING_LIGHTS_DDR)
  void init_landing_lights(void) {
    LANDING_LIGHTS_DDR |= 1<<LANDING_LIGHTS_BIT;
  }
  
  void inline switch_landing_lights(uint8_t on) {
    if (on) {
      LANDING_LIGHTS_PORT |= 1<<LANDING_LIGHTS_BIT;
    } else {
      LANDING_LIGHTS_PORT &= ~(1<<LANDING_LIGHTS_BIT);
    }
  }
  
  void auto_switch_landing_lights(void) {
    if (rcOptions[BOXLLIGHTS]
    #if defined(LANDING_LIGHTS_AUTO_ALTITUDE) & SONAR
  	|| (sonarAlt >= 0 && sonarAlt <= LANDING_LIGHTS_AUTO_ALTITUDE && f.ARMED)
    #endif
    ) {
      switch_landing_lights(1);
    } else {
      switch_landing_lights(0);
    }
  }
#endif
