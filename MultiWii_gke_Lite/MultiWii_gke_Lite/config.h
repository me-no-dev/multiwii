/*************************************************************************************************/
/****           CONFIGURABLE PARAMETERS                                                       ****/
/*************************************************************************************************/

/* this file consists of several sections
 * to create a working combination you must at least make your choices in section 1.
 * 1 - BASIC SETUP - you must select an option in every block.
 *      this assumes you have 4 channels connected to your board with standard ESCs and servos.
 * 2 - COPTER TYPE SPECIFIC OPTIONS - you likely want to check for options for your copter type
 * 3 - RC SYSTEM SETUP
 * 4 - ALTERNATE CPUs & BOARDS - if you have
 * 5 - ALTERNATE SETUP - select alternate RX (SBUS, PPM, etc.), alternate ESC-range, etc. here
 * 6 - OPTIONAL FEATURES - enable nice to have features here (FlightModes, LCD, telemetry, battery monitor etc.)
 * 7 - TUNING & DEVELOPER - if you know what you are doing; you have been warned
 */

/* Notes:
 * 1. parameters marked with (*) in the comment are stored in eeprom and can be tweaked via serial monitor or LCD.
 *    Changing those values in config.h and upload will require a 'Reset' from the GUI to take effect
 */
 
#define GENERAL_USE // default settings that should work for most

#if defined(GENERAL_USE)

  #define FAILSAFE
   
  #define ALLOW_ARM_DISARM_VIA_TX_YAW
  //#define ALLOW_ARM_DISARM_VIA_TX_ROLL
  #define USE_MW_SPEKTRUM_SCALING // simple original scaling but does not reflect the actual pulse widths & neutrals!
  #define SPEKTRUM 1024
  //#define SERIAL_SUM_PPM  PITCH,YAW,THROTTLE,ROLL,AUX1,AUX2,AUX3,AUX4,8,9,10,11 //For Graupner/Spektrum

  //#define ACROTRAINER_MODE 200 // inactive if commented out or > 500
  #define DEADBAND 20 // 20 // uSec  
 
  // set PWM frequency top Output.ino
  #define HK_PocketQuad
  //#define MPU6050_LPF_42HZ
  //#define MPU6050_LPF_98HZ
  #define MPU6050_LPF_188Hz
  
  //#define MOTOR_STOP // comment out for slow motor run after arming
  #define MINCOMMAND  1000
  #define MINTHROTTLE 1050
  #define MAXTHROTTLE 1850
  
//_____________________________________________________________________________________________

// Configurations for gke's aircraft - YOU SHOULD NOT CHANGE THESE

#else

 //#define MW_ECKS

  #define FAILSAFE
  
  //#define USE_MW_CONTROL
  
  //#define ALLOW_ARM_DISARM_VIA_TX_YAW
  #define ALLOW_ARM_DISARM_VIA_TX_ROLL

  //#define ACROTRAINER_MODE 200 // inactive if commented out or > 500
  #define DEADBAND 20 // uSec  
 
  #ifdef MW_ECKS
  
    #define NANOWII
    #define MPU6050_LPF_98HZ
    //#define MPU6050_LPF_188HZ
    #define MINCOMMAND  1000
    #define MINTHROTTLE 1100 // 1064 // special ESC (simonk)
    #define MAXTHROTTLE 1850
    #define SERIAL_SUM_PPM  PITCH,YAW,THROTTLE,ROLL,AUX1,AUX2,AUX3,AUX4,8,9,10,11 //For Graupner/Spektrum
    
  #else // POCKET QUAD
  
    // set PWM frequency top Output.ino
    #define HK_PocketQuad
    #define MPU6050_LPF_98HZ
    //#define MPU6050_LPF_188HZ
    #define MINCOMMAND  1000
    #define MINTHROTTLE 1050
    #define MAXTHROTTLE 1850
    #define SPEKTRUM 1024
    //#define SERIAL_SUM_PPM  PITCH,YAW,THROTTLE,ROLL,AUX1,AUX2,AUX3,AUX4,8,9,10,11 //For Graupner/Spektrum
    
  #endif // MW_ECKS

#endif // GENERAL_USE

// Best not to change these ;)

  #define USE_THROTTLE_CURVE // MW GUI throttle curve active
  //#define USE_MW_PARAM_DEFAULTS 
  //#define USE_MW_RC_FILTER // moving average RC jitter filter
  #define USE_RC_FILTER // simple average
  #define ACC_LPF_FACTOR 4  // 1, 2, 4 acc smoothing

  #define CYCLETIME 2000 // no point in going faster:)

//____________________________________________________________________________________________



/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  1 - BASIC SETUP                                                *******/
/*****************                                                                 ***************/
/*************************************************************************************************/

  /**************************    The type of multicopter    ****************************/

    //#define QUADP
    //#define QUADX
    
    #define YAW_DIRECTION 1

  /****************************    Motor minthrottle    *******************************/
    /* Set the minimum throttle command sent to the ESC (Electronic Speed Controller)
       This is the minimum value that allow motors to run at a idle speed  */
    //#define MINTHROTTLE 1300 // for Turnigy Plush ESCs 10A
    //#define MINTHROTTLE 1120 // for Super Simple ESCs 10A
    //#define MINTHROTTLE 1064 // special ESC (simonk)
    //#define MINTHROTTLE 1050 // for brushed ESCs like ladybird
    //#define MINTHROTTLE 1150 // (*)

  /****************************    Motor maxthrottle    *******************************/
    /* this is the maximum value for the ESCs at full power, this value can be increased up to 2000 */
    //#define MAXTHROTTLE 1850

  /****************************    Mincommand          *******************************/
    /* this is the value for the ESCs when they are not armed
       in some cases, this value must be lowered down to 900 for some specific ESCs, otherwise they failed to initiate */
    //#define MINCOMMAND  1000

  /**********************************    I2C speed   ************************************/

    #define I2C_SPEED 400000L   //400kHz fast mode, it works only with some WMP clones

  /***************************    Internal i2c Pullups   ********************************/
    /* enable internal I2C pull ups (in most cases it is better to use external pullups) */
    //#define INTERNAL_I2C_PULLUPS

  /**************************************************************************************/
  /*****************          boards and sensor definitions            ******************/
  /**************************************************************************************/

    /***************************    Combined IMU Boards    ********************************/
      /* if you use a specific sensor board:
         please submit any correction to this list.

      //#define NANOWII         // the smallest multiwii FC based on MPU6050 + pro micro based proc <- confirmed by Alex
      //#define HK_PocketQuad      // Hobbyking board Ultra-Micro with MPU6050

      
    /***************************    independent sensors    ********************************/
      /* leave it commented if you already checked a specific board above */
      /* I2C gyroscope */

      //#define MPU6050       //combo + ACC

      /* enforce your individual sensor orientation - even overrides board specific defaults */
      //#define FORCE_ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  =  Y; accADC[PITCH]  = -X; accADC[YAW]  = Z;}
      //#define FORCE_GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] = -Y; gyroADC[PITCH] =  X; gyroADC[YAW] = Z;}


/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  2 - COPTER TYPE SPECIFIC OPTIONS                               *******/
/*****************                                                                 ***************/
/*************************************************************************************************/

   /********************************    ARM/DISARM    *********************************/
   /* optionally disable stick combinations to arm/disarm the motors.
     * In most cases one of the two options to arm/disarm via TX stick is sufficient */
    //#define ALLOW_ARM_DISARM_VIA_TX_YAW
    //#define ALLOW_ARM_DISARM_VIA_TX_ROLL

    //#define LEAVE_HEADROOM_FOR_MOTORS 4 // leave room for gyro corrrections only for first 4 motors

/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  3 - RC SYSTEM SETUP                                            *******/
/*****************                                                                 ***************/
/*************************************************************************************************/

  /* note: no need to uncomment something in this section if you use a standard receiver */

  /**************************************************************************************/
  /********                       special receiver types             ********************/
  /**************************************************************************************/

    /****************************    PPM Sum Reciver    ***********************************/
      /* The following lines apply only for specific receiver with only one PPM sum signal, on digital PIN 2
         Select the right line depending on your radio brand. Feel free to modify the order in your PPM order is different */
      //#define SERIAL_SUM_PPM         PITCH,YAW,THROTTLE,ROLL,AUX1,AUX2,AUX3,AUX4,8,9,10,11 //For Graupner/Spektrum
      //#define SERIAL_SUM_PPM         ROLL,PITCH,THROTTLE,YAW,AUX1,AUX2,AUX3,AUX4,8,9,10,11 //For Robe/Hitec/Futaba
      //#define SERIAL_SUM_PPM         ROLL,PITCH,YAW,THROTTLE,AUX1,AUX2,AUX3,AUX4,8,9,10,11 //For Multiplex
      //#define SERIAL_SUM_PPM         PITCH,ROLL,THROTTLE,YAW,AUX1,AUX2,AUX3,AUX4,8,9,10,11 //For some Hitec/Sanwa/Others

      // Uncommenting following line allow to connect PPM_SUM receiver to standard THROTTLE PIN on MEGA boards (eg. A8 in CRIUS AIO)
      //#define PPM_ON_THROTTLE

    /**********************    Spektrum Satellite Reciver    *******************************/
      /* The following lines apply only for Spektrum Satellite Receiver
         Spektrum Satellites are 3V devices.  DO NOT connect to 5V!
         For MEGA boards, attach sat grey wire to RX1, pin 19. Sat black wire to ground. Sat orange wire to Mega board's 3.3V (or any other 3V to 3.3V source).
         For PROMINI, attach sat grey to RX0.  Attach sat black to ground. */
      //#define SPEKTRUM 1024
      //#define SPEKTRUM 2048
      //#define SPEK_SERIAL_PORT 1    // Forced to 0 on Pro Mini and single serial boards; Set to your choice of 0, 1, or 2 on any Mega based board (defaults to 1 on Mega).
      //**************************
      // Defines that allow a "Bind" of a Spektrum or Compatible Remote Receiver (aka Satellite) via Configuration GUI.
      //   Bind mode will be same as declared above, if your TX is capable.
      //   Ground, Power, and Signal must come from three adjacent pins. 
      //   By default, these are Ground=4, Power=5, Signal=6.  These pins are in a row on most MultiWii shield boards. Pins can be overriden below.  
      //   Normally use 3.3V regulator is needed on the power pin!!  If your satellite hangs during bind (blinks, but won't complete bind with a solid light), go direct 5V on all pins. 
      //**************************
      //   For Pro Mini, the connector for the Satellite that resides on the FTDI can be unplugged and moved to these three adjacent pins. 
      //#define SPEK_BIND             //Un-Comment for Spektrum Satellie Bind Support.  Code is ~420 bytes smaller without it. 
      //#define SPEK_BIND_GROUND 4
      //#define SPEK_BIND_POWER  5
      //#define SPEK_BIND_DATA   6

    /*******************************    SBUS RECIVER    ************************************/
      /* The following line apply only for Futaba S-Bus Receiver on MEGA boards at RX1 only (Serial 1).
         You have to invert the S-Bus-Serial Signal e.g. with a Hex-Inverter like IC SN74 LS 04 */
      //#define SBUS

/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  4 - ALTERNATE CPUs & BOARDS                                    *******/
/*****************                                                                 ***************/
/*************************************************************************************************/

  /**************************************************************************************/
  /********   Settings for ProMicro, Leonardo and other Atmega32u4 Boards     ***********/
  /**************************************************************************************/

    /*********************************    pin Layout     **********************************/
      /* activate this for a better pinlayout if all pins can be used => not possible on ProMicro */
      //#define A32U4ALLPINS

    /**********************************    PWM Setup     **********************************/
      /* activate all 6 hardware PWM outputs Motor 5 = D11 and 6 = D13. 
         note: not possible on the sparkfun promicro (pin 11 & 13 are not broken out there)
         if activated:
         Motor 1-6 = 10-bit hardware PWM
         Motor 7-8 = 8-bit Software PWM
         Servos    = 8-bit Software PWM
         if deactivated:
         Motor 1-4 = 10-bit hardware PWM
         Motor 5-8 = 10-bit Software PWM
         Servos    = 10-bit Software PWM */
      //#define HWPWM6

    /**********************************    Aux 2 Pin     **********************************/
      /* AUX2 pin on pin RXO */
      //#define RCAUX2PINRXO

      /* aux2 pin on pin D17 (RXLED) */
      //#define RCAUX2PIND17


    /***********************      Promicro version related     ****************************/
      /* Inverted status LED for Promicro ver 10 */
      //#define PROMICRO10


  /**************************************************************************************/
  /********                      override default pin assignments    ********************/
  /**************************************************************************************/

  /* only enable any of this if you must change the default pin assignment, e.g. your board does not have a specific pin */
  /* you may need to change PINx and PORTx plus #shift according to the desired pin! */
  //#define OVERRIDE_V_BATPIN                   A0 // instead of A3    // Analog PIN 3

  //#define OVERRIDE_LEDPIN_PINMODE             pinMode (A1, OUTPUT); // use A1 instead of d13
  //#define OVERRIDE_LEDPIN_TOGGLE              PINC |= 1<<1; // PINB |= 1<<5;     //switch LEDPIN state (digital PIN 13)
  //#define OVERRIDE_LEDPIN_OFF                 PORTC &= ~(1<<1); // PORTB &= ~(1<<5);
  //#define OVERRIDE_LEDPIN_ON                  PORTC |= 1<<1;    // was PORTB |= (1<<5);

  //#define OVERRIDE_BUZZERPIN_PINMODE          pinMode (A2, OUTPUT); // use A2 instead of d8
  //#define OVERRIDE_BUZZERPIN_ON               PORTC |= 1<<2 //PORTB |= 1;
  //#define OVERRIDE_BUZZERPIN_OFF              PORTC &= ~(1<<2); //PORTB &= ~1;

/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  5 - ALTERNATE SETUP                                            *******/
/*****************                                                                 ***************/
/*************************************************************************************************/

  /******                Serial com speed    *********************************/
    /* This is the speed of the serial interfaces */
    #define SERIAL0_COM_SPEED 115200
    #define SERIAL1_COM_SPEED 115200
    #define SERIAL2_COM_SPEED 115200
    #define SERIAL3_COM_SPEED 115200

    /* interleaving delay in micro seconds between 2 readings WMP/NK in a WMP+NK config
       if the ACC calibration time is very long (20 or 30s), try to increase this delay up to 4000
       it is relevent only for a conf with NK */
    #define INTERLEAVING_DELAY 3000

    /* when there is an error on I2C bus, we neutralize the values during a short time. expressed in microseconds
       it is relevent only for a conf with at least a WMP */
    #define NEUTRALIZE_DELAY 100000


  /**************************************************************************************/
  /********                              Gyro filters                ********************/
  /**************************************************************************************/

 
      /* MPU6050 Low pass filter setting. In case you cannot eliminate all vibrations to the Gyro, you can try
         to decrease the LPF frequency, only one step per try. As soon as twitching gone, stick with that setting.
         It will not help on feedback wobbles, so change only when copter is randomly twiching and all dampening and
         balancing options ran out. Uncomment only one option!
         IMPORTANT! Change low pass filter setting changes PID behaviour, so retune your PID's after changing LPF.*/
      //#define MPU6050_LPF_256HZ     // This is the default setting, no need to uncomment, just for reference
      //#define MPU6050_LPF_188HZ
      //#define MPU6050_LPF_98HZ
      //#define MPU6050_LPF_42HZ
      //#define MPU6050_LPF_20HZ
      //#define MPU6050_LPF_10HZ
      //#define MPU6050_LPF_5HZ       // Use this only in extreme cases, rather change motors and/or props


/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  6 - OPTIONAL FEATURES                                          *******/
/*****************                                                                 ***************/
/*************************************************************************************************/

  /************************        continuous gyro calibration        ********************/
  /* Gyrocalibration will be repeated if copter is moving during calibration. */
    //#define GYROCALIBRATIONFAILSAFE

  /************************        AP FlightMode        **********************************/
    /* Temporarily Disables GPS_HOLD_MODE to be able to make it possible to adjust the Hold-position when moving the sticks.*/
    #define AP_MODE 40  // Create a deadspan for GPS.
        
  /************************   Assisted AcroTrainer    ************************************/
    /* Train Acro with auto recovery. Value set the point where ANGLE_MODE takes over.
       Remember to activate ANGLE_MODE first!...
       A Value on 200 will give a very distinct transfer */
    #if !defined(ACROTRAINER_MODE)
       #define ACROTRAINER_MODE 1000
    #endif
    //#define ACROTRAINER_MODE 200   // http://www.multiwii.com/forum/viewtopic.php?f=16&t=1944#p17437


  /********                          Failsafe settings                 ********************/
    /* Failsafe check pulses on four main control channels CH1-CH4. If the pulse is missing or bellow 985us (on any of these four channels) 
       the failsafe procedure is initiated. After FAILSAFE_DELAY time from failsafe detection, the level mode is on (if ACC or nunchuk is avaliable),
       PITCH, ROLL and YAW is centered and THROTTLE is set to FAILSAFE_THR0TTLE value. You must set this value to descending about 1m/s or so 
       for best results. This value is depended from your configuration, AUW and some other params.  Next, afrer FAILSAFE_OFF_DELAY the copter is disarmed, 
       and motors is stopped. If RC pulse coming back before reached FAILSAFE_OFF_DELAY time, after the small quard time the RC control is returned to normal. */

    #define FAILSAFE_DELAY     10                     // Guard time for failsafe activation after signal lost. 1 step = 0.1sec - 1sec in example
    #define FAILSAFE_OFF_DELAY (FAILSAFE_DELAY*5)                    // Time for Landing before motors stop in 0.1sec. 
    #define FAILSAFE_THROTTLE  (MINTHROTTLE + 200)    // (*) Throttle level used for landing - may be relative to MINTHROTTLE - as in this case


  /**************************************************************************************/
  /***********************                  TX-related         **************************/
  /**************************************************************************************/

    /* introduce a deadband around the stick center
       Must be greater than zero, comment if you dont want a deadband on roll, pitch and yaw */
       #if !defined(DEADBAND)
          #define DEADBAND 0
       #endif

/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  7 - TUNING & DEVELOPER                                  **************/
/*****************                                                                 ***************/
/*************************************************************************************************/


  /**************************************************************************************/
  /********   special ESC with extended range [0-2000] microseconds  ********************/
  /**************************************************************************************/
    //#define DC_MOTORS

  /**************************************************************************************/
  /***********************     motor, servo and other presets     ***********************/
  /**************************************************************************************/
    /* motors will not spin when the throttle command is in low position
       this is an alternative method to stop immediately the motors */
    //#define MOTOR_STOP

    /* some radios have not a neutral point centered on 1500. can be changed here */
    #define MIDRC 1500


  /********************************************************************/
  /****           Serial command handling - MSP and other          ****/
  /********************************************************************/

    /* to reduce memory footprint, it is possible to suppress handling of serial commands.
     * This does _not_ affect handling of RXserial, Spektrum or GPS. Those will not be affected and still work the same.
     * Enable either one or both of the following options  */

    /* Remove handling of all commands of the New MultiWii Serial Protocol.
     * This will disable use of the GUI, winGUI, android apps and any other program that makes use of the MSP.
     * You must find another way (like LCD_CONF) to tune the parameters or live with the defaults.
     * If you run a LCD/OLED via i2c or serial/Bluetooth, this is safe to use */
    //#define SUPPRESS_ALL_SERIAL_MSP // saves approx 2700 bytes

    /* Remove handling of other serial commands.
     * This includes navigating via serial the lcd.configuration menu, lcd.telemetry and permanent.log .
     * Navigating via stick inputs on tx is not affected and will work the same.  */
    //#define SUPPRESS_OTHER_SERIAL_COMMANDS // saves  approx 0 to 100 bytes, depending on features enabled

 
  /********************************************************************/
  /****           ESCs calibration                                 ****/
  /********************************************************************/

    /* to calibrate all ESCs connected to MWii at the same time (useful to avoid unplugging/re-plugging each ESC)
       Warning: this creates a special version of MultiWii Code
       You cannot fly with this special version. It is only to be used for calibrating ESCs
       Read How To at http://code.google.com/p/multiwii/wiki/ESCsCalibration */
    #define ESC_CALIB_LOW  MINCOMMAND
    #define ESC_CALIB_HIGH 2000
    //#define ESC_CALIB_CANNOT_FLY  // uncomment to activate


/*************************************************************************************************/
/****           END OF CONFIGURABLE PARAMETERS                                                ****/
/*************************************************************************************************/
