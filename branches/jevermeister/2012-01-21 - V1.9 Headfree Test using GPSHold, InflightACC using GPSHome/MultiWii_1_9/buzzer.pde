void buzzer(){
  static uint16_t ontime, offtime, beepcount, repeat, repeatcounter;
  static uint32_t buzzerLastTime;
  static uint8_t  buzzerCurrentState = 0;
  static uint8_t BaroActive = 0,
                 MagActive = 0,
                 GPSHoldActive = 0,
                 last_BaroActive = 0,
                 last_MagActive = 0,
                 last_GPSHoldActive = 0;
  
  //=====================  Beeps for activateing baro 
  BaroActive = (rcOptions & activate[BOXBARO]);
  if (BaroActive && !last_BaroActive){
    confirmation_flag = 1;
    }
  else if(!BaroActive && last_BaroActive){
    confirmation_flag = 2;
    }
  last_BaroActive = BaroActive;
  
  //=====================  Beeps for activateing mag
  MagActive = (rcOptions & activate[BOXMAG]);
  if (MagActive && !last_MagActive){
    confirmation_flag = 1;
    }
  else if (!MagActive && last_MagActive){
    confirmation_flag = 2;
    }
  last_MagActive = MagActive;
  
  //=====================  Beeps for activateing gps
  GPSHoldActive = (rcOptions & activate[BOXGPSHOLD]);
  if (GPSHoldActive && !last_GPSHoldActive){
    confirmation_flag = 1;
    }
  else if (!GPSHoldActive && last_GPSHoldActive){
    confirmation_flag = 2;
    }
  last_GPSHoldActive = GPSHoldActive;
  
  
  //=====================  Global Buzzer handling by Jevermeister
  if ( warn_failsafe >0 || warn_powermeter >0 || warn_vbat >0 || confirmation_flag > 0) {
    //the order of the below is the priority from low to high, the last entry has the highest priority
    repeat = 1;                                           // set repeat to default 1
    ontime=100;                                           //ontime is a constant of 100ms
    if (confirmation_flag)offtime = 50;                   //fast confirmation beep
    if (warn_vbat == 1)offtime = 2000;                    //vbat warning level 1
    if (warn_vbat == 2){offtime = 1000; repeat = 2;}      //vbat warning level 2
    if (warn_vbat == 3){offtime = 500; repeat = 3;}     //vbat critical level
    if (warn_powermeter == 1)offtime = 400;              //powermeter warning
    if (warn_failsafe == 1)offtime = 50 ;	            //failsafe landing aktive
    if (warn_failsafe == 2){offtime = 2000;ontime=300;}	              //failsafe "find me" signal
    
    if (beepcount <= confirmation_flag){      //confirmation flag is 0,1 or 2 
      if ( repeatcounter > 1 && !buzzerCurrentState && (millis() >= (buzzerLastTime + 80)) ){    // if the buzzer is off and there is a short pause neccessary (multipe buzzes)
        buzzerCurrentState = 1;
        BUZZERPIN_ON;
        buzzerLastTime=millis();      // save the time the buzer turned on
        repeatcounter--;
      } else if ( !buzzerCurrentState && (millis() >= (buzzerLastTime + offtime)) ) {	          // Buzzer is off and long pause time is up -> turn it on
        buzzerCurrentState = 1;
        BUZZERPIN_ON;
        buzzerLastTime=millis();      // save the time the buzer turned on
        repeatcounter = repeat;  //set the amount of repeats
      } else if (buzzerCurrentState && (millis() >= buzzerLastTime + ontime) ) {         //Buzzer is on and time is up -> turn it off
        buzzerCurrentState = 0;
        BUZZERPIN_OFF;
        buzzerLastTime=millis();                                 // save the time the buzer turned on
        if (confirmation_flag > 0)beepcount++;                           // only increment if confirmation beep, the rest is endless
      }
    }else{                        //counter is done reset flag and counter
        beepcount = 0;            //reset the counter for the next time
        confirmation_flag = 0;    //reset the flag after all beeping is done
    }
  }else{                          //no beeping neccessary:reset everything (just in case)
    beepcount = 0;                //reset the counter for the next time 
    BUZZERPIN_OFF;              
    buzzerCurrentState = 0; 
  }	
}
