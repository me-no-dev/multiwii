#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
  #include "Atmega328P_config.h"
#endif
#if defined(__AVR_ATmega32U4__)
  #include "Atmega32u4_config.h"
#endif
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
  #include "Atmega128x_256x_config.h"
#endif


#if defined(TRI)
  #define MULTITYPE 1
#elif defined(QUADP)
  #define MULTITYPE 2
#elif defined(QUADX)
  #define MULTITYPE 3
#elif defined(BI)
  #define MULTITYPE 4
#elif defined(GIMBAL)
  #define MULTITYPE 5
#elif defined(Y6)
  #define MULTITYPE 6
#elif defined(HEX6)
  #define MULTITYPE 7
#elif defined(FLYING_WING)
  #define MULTITYPE 8
#elif defined(Y4)
  #define MULTITYPE 9
#elif defined(HEX6X)
  #define MULTITYPE 10
#elif defined(OCTOX8)
  #define MULTITYPE 11   //the JAVA GUI is the same for all 8 motor configs 
#elif defined(OCTOFLATP)
  #define MULTITYPE 12   //12  for MultiWinGui
#elif defined(OCTOFLATX)
  #define MULTITYPE 13   //13  for MultiWinGui 
#elif defined(AIRPLANE)    
  #define MULTITYPE 14    
#elif defined (HELI_120_CCPM)   
  #define MULTITYPE 15      // Simple model 
#elif defined (HELI_90_DEG)   
  #define MULTITYPE 16      // Simple model  
#elif defined(VTAIL4)
 #define MULTITYPE 17
#endif

/* motor and servo numbers */

#if defined(BI) || defined(TRI) || defined(SERVO_TILT) || defined(GIMBAL) || defined(FLYING_WING) || defined(AIRPLANE) || defined(CAMTRIG)
  #define SERVO
#endif

#if defined(GIMBAL)
  #define NUMBER_MOTOR     0
  #define PRI_SERVO_FROM   1 // use servo from 1 to 2
  #define PRI_SERVO_TO     2
#elif defined(FLYING_WING)
  #define NUMBER_MOTOR     1
  #define PRI_SERVO_FROM   1 // use servo from 1 to 2
  #define PRI_SERVO_TO     2
#elif defined(AIRPLANE)
  #define NUMBER_MOTOR     0
  #define PRI_SERVO_FROM   4 // use servo from 4 to 8
  #define PRI_SERVO_TO     8
#elif defined(BI)
  #define NUMBER_MOTOR     2
  #define PRI_SERVO_FROM   5 // use servo from 5 to 6
  #define PRI_SERVO_TO     6
#elif defined(TRI)
  #define NUMBER_MOTOR     3
  #define PRI_SERVO_FROM   6 // use only servo 6
  #define PRI_SERVO_TO     6
#elif defined(QUADP) || defined(QUADX) || defined(Y4)|| defined(VTAIL4)
  #define NUMBER_MOTOR     4
#elif defined(Y6) || defined(HEX6) || defined(HEX6X)
  #define NUMBER_MOTOR     6
#elif defined(OCTOX8) || defined(OCTOFLATP) || defined(OCTOFLATX)
  #define NUMBER_MOTOR     8
#endif

#if defined(SERVO_TILT) && defined(CAMTRIG)
  #define SEC_SERVO_FROM   1 // use servo from 1 to 3
  #define SEC_SERVO_TO     3
#else
  #if defined(SERVO_TILT)
    #define SEC_SERVO_FROM   1 // use servo from 1 to 2
    #define SEC_SERVO_TO     2
  #endif
  #if defined(CAMTRIG)
    #define SEC_SERVO_FROM   3 // use servo 3
    #define SEC_SERVO_TO     3
  #endif
#endif


#if !defined(ACC)
  #define ACC 0
#endif
#if !defined(BARO)
  #define BARO 0
#endif
#if !defined(MAG)
  #define MAG 0
#endif
#if !defined(SONAR)
  #define SONAR 0
#endif
#if !defined(GPS)
  #define GPS 0
#endif

#if defined(SERIAL_SUM_PPM)
  #define SPECIAL_RX
  #define SPECIAL_RX_PINLAYOUT SERIAL_SUM_PPM
#endif


