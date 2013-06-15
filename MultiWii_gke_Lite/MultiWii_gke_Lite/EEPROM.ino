#include <avr/eeprom.h>


uint8_t calculate_sum(uint8_t *cb , uint8_t siz) {
  uint8_t sum=0x55;  // checksum init
  while(--siz) sum += *cb++;  // calculate checksum (without checksum byte)
  return sum;
} // calculate_sum

void readGlobalSet() {
  eeprom_read_block((void*)&global_conf, (void*)0, sizeof(global_conf));
  if(calculate_sum((uint8_t*)&global_conf, sizeof(global_conf)) != global_conf.checksum) 
    global_conf.currentSet = 0;  

  f.ACC_CALIBRATED = global_conf.accCalibrated;
} // readGlobalSet

void readEEPROM() {
  uint8_t i, y;
  int16_t tmp;

  global_conf.currentSet = 0;

  eeprom_read_block((void*)&conf, (void*)(global_conf.currentSet * sizeof(conf) + sizeof(global_conf)), sizeof(conf));
  if(calculate_sum((uint8_t*)&conf, sizeof(conf)) != conf.checksum) {
    blinkLED(6,100,3);    
    LoadDefaults(); 
  }
   
#ifdef USE_THROTTLE_CURVE
 for(i = 0; i<11; i++ ) {
    tmp = 10*i-conf.thrMid8;
    y = 1;
    if ( tmp > 0 ) y = 100-conf.thrMid8;
    if ( tmp < 0 ) y = conf.thrMid8;
    
    lookupThrottleRC[i] = 10*conf.thrMid8 + tmp*( 100-conf.thrExpo8+(int32_t)conf.thrExpo8*(tmp*tmp)/(y*y) )/10; // [0;1000]
    lookupThrottleRC[i] = conf.minthrottle + (int32_t)(MAXTHROTTLE-conf.minthrottle)* lookupThrottleRC[i]/1000; // [0;1000] -> [conf.minthrottle;MAXTHROTTLE]
  }
#endif // USE_THROTTLE_CURVE

} // readEEPROM

void writeGlobalSet(uint8_t b) {
  global_conf.checksum = calculate_sum((uint8_t*)&global_conf, sizeof(global_conf));
  eeprom_write_block((const void*)&global_conf, (void*)0, sizeof(global_conf));
  if (b == 1) blinkLED(15,20,1);
} // writeGlobalSet

void writeParams(uint8_t b) {

  global_conf.currentSet = 0;
  conf.checksum = calculate_sum((uint8_t*)&conf, sizeof(conf));
  eeprom_write_block((const void*)&conf, (void*)(global_conf.currentSet * sizeof(conf) + sizeof(global_conf)), sizeof(conf));
  readEEPROM();
  if (b == 1) blinkLED(15,20,1);
} // writeParams

void LoadDefaults(void) {

#if defined(USE_MW_PARAM_DEFAULTS)

  conf.P8[ROLL] = 33;  
  conf.I8[ROLL] = 30; 
  conf.D8[ROLL] = 23;
  conf.P8[PITCH] = 33; 
  conf.I8[PITCH] = 30; 
  conf.D8[PITCH] = 23;
  conf.P8[YAW] = 68;  
  conf.I8[YAW] = 45;  
  conf.D8[YAW] = 0;
  conf.P8[PIDALT] = 64; 
  conf.I8[PIDALT] = 25; 
  conf.D8[PIDALT] = 24;

  conf.P8[PIDPOS] = POSHOLD_P * 100;     
  conf.I8[PIDPOS] = POSHOLD_I * 100;       
  conf.D8[PIDPOS] = 0;
  conf.P8[PIDPOSR] = POSHOLD_RATE_P * 10; 
  conf.I8[PIDPOSR] = POSHOLD_RATE_I * 100;  
  conf.D8[PIDPOSR] = POSHOLD_RATE_D * 1000;
  conf.P8[PIDNAVR] = NAV_P * 10;          
  conf.I8[PIDNAVR] = NAV_I * 100;           
  conf.D8[PIDNAVR] = NAV_D * 1000;

  conf.P8[PIDLEVEL] = 90; 
  conf.I8[PIDLEVEL] = 10; 
  conf.D8[PIDLEVEL] = 100;
  conf.P8[PIDMAG]   = 40;

  conf.P8[PIDVEL] = 0;      
  conf.I8[PIDVEL] = 0;    
  conf.D8[PIDVEL] = 0;

  conf.rcRate8 = 90; 
  conf.rcExpo8 = 65;
  conf.rollPitchRate = 0;
  conf.yawRate = 0;
  
  conf.dynThrPID = 0;
  conf.thrMid8 = 50; 
  conf.thrExpo8 = 40;
  
#else 

  // Minimum defaults for MPU6050 sensors only

  memset(&conf, 0, sizeof(conf));

  conf.P8[ROLL] = 30;   
  conf.I8[ROLL] = 30; 
  conf.D8[ROLL] = 25;
  conf.P8[PITCH] = 30;  
  conf.I8[PITCH] = 30; 
  conf.D8[PITCH] = 25;
  conf.P8[YAW] = 80; 
  conf.I8[YAW] = 45; 
 
 conf.rcRate8 = 100; 
   
#ifdef USE_THROTTLE_CURVE
  conf.thrMid8 = 45; 
  conf.thrExpo8 = 40;
#else  
  conf.thrMid8 = 50; 
  conf.thrExpo8 = 100; 
#endif // USE_THROTTLE_CURVE

#endif // USE_MW_PARAM_DEFAULTS

  for(uint8_t i = 0; i < CHECKBOXITEMS; i++)
    conf.activate[i] = 0;

  conf.accTrim[ROLL] = conf.accTrim[PITCH] = conf.accTrim[YAW] = 0;
  conf.powerTrigger1 = 0;
  conf.failsafe_throttle = FAILSAFE_THROTTLE;
  conf.cycletimeuS = CYCLETIME;
  conf.minthrottle = MINTHROTTLE;

  writeParams(0); // this will also (p)reset checkNewConf with the current version number again.
  
  memset(&global_conf, 0, sizeof(global_conf));
  writeGlobalSet(1);
  readGlobalSet();
  
} // loadDefaults




