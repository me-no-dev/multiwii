
// LIMIT1 MACRO TO REPLACE SYMMETRIC CONSTRAINS

#define Limit1(i,l) (((i) < -(l)) ? -(l) : (((i) > (l)) ? (l) : (i)))

/**************************************************************************************/
/***************             Proc specific definitions             ********************/
/**************************************************************************************/
// Proc auto detection

#if defined(__AVR_ATmega32U4__) 
#define PROMICRO
#endif

/**************************************************************************************/
/***************             motor and servo numbers               ********************/
/**************************************************************************************/

#define NUMBER_MOTOR     4

#ifdef MOTOR_STOP
#define EFF_MINCOMMAND MINCOMMAND
#else
#define EFF_MINCOMMAND conf.minthrottle
#endif

/**************************  atmega32u4 (Promicro)  ***********************************/

#define LEDPIN_PINMODE             //
#define LEDPIN_TOGGLE              PIND |= 1<<5     //switch LEDPIN state (Port D5)
#if !defined(PROMICRO10)
#define LEDPIN_OFF                 PORTD |= (1<<5)
#define LEDPIN_ON                  PORTD &= ~(1<<5)
#else
#define LEDPIN_OFF                PORTD &= ~(1<<5)
#define LEDPIN_ON                 PORTD |= (1<<5)
#endif

#define POWERPIN_PINMODE           //
#define POWERPIN_ON                //
#define POWERPIN_OFF               //
#define I2C_PULLUPS_ENABLE         PORTD |= 1<<0; PORTD |= 1<<1   // PIN 2&3 (SDA&SCL)
#define I2C_PULLUPS_DISABLE        PORTD &= ~(1<<0); PORTD &= ~(1<<1)
#define STABLEPIN_PINMODE          ;
#define STABLEPIN_ON               ;
#define STABLEPIN_OFF              ;
#define PPM_PIN_INTERRUPT          DDRE &= ~(1 << 6);PORTE |= (1 << 6);EIMSK |= (1 << INT6);EICRB |= (1 << ISC61)|(1 << ISC60)
#if !defined(SPEK_SERIAL_PORT)
#define SPEK_SERIAL_PORT           1
#endif
#define USB_CDC_TX                 3
#define USB_CDC_RX                 2

//Standard RX
#define THROTTLEPIN                  3
#if defined(A32U4ALLPINS)
#define ROLLPIN                    6
#define PITCHPIN                   2
#define YAWPIN                     4
#define AUX1PIN                    5
#else
#define ROLLPIN                    4
#define PITCHPIN                   5
#define YAWPIN                     2
#define AUX1PIN                    6
#endif
#define AUX2PIN                      7 
#define AUX3PIN                      1 // unused 
#define AUX4PIN                      0 // unused 
#if !defined(RCAUX2PIND17)
#define PCINT_PIN_COUNT          4
#define PCINT_RX_BITS            (1<<1),(1<<2),(1<<3),(1<<4)
#else
#define PCINT_PIN_COUNT          5 // one more bit (PB0) is added in RX code
#define PCINT_RX_BITS            (1<<1),(1<<2),(1<<3),(1<<4),(1<<0)
#endif
#define PCINT_RX_PORT                PORTB
#define PCINT_RX_MASK                PCMSK0
#define PCIR_PORT_BIT                (1<<0)
#define RX_PC_INTERRUPT              PCINT0_vect
#define RX_PCINT_PIN_PORT            PINB

#if !defined(A32U4ALLPINS) 
#define V_BATPIN                  A3    // Analog PIN 3
#elif defined(A32U4ALLPINS)
#define V_BATPIN                  A4    // Analog PIN 4
#else
#define V_BATPIN                  A2    // Analog PIN 3
#endif

#define PSENSORPIN                A2    // Analog PIN 2 


/**************************************************************************************/
/***************      IMU Orientations and Sensor definitions      ********************/
/**************************************************************************************/


#if defined(NANOWII)
#define QUADX
#define MPU6050
#define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -Y; accADC[PITCH]  =  X; accADC[YAW]  =  Z;}
#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] = -X; gyroADC[PITCH] = -Y; gyroADC[YAW] = -Z;}
#undef INTERNAL_I2C_PULLUPS

#define LEDPIN_PINMODE             DDRD |= (1<<4)           //D4 to output
#define LEDPIN_TOGGLE              PIND |= (1<<5)|(1<<4)     //switch LEDPIN state (Port D5) & pin D4
#define LEDPIN_OFF                 PORTD |= (1<<5); PORTD &= ~(1<<4)
#define LEDPIN_ON                  PORTD &= ~(1<<5); PORTD |= (1<<4)  
#endif

#if defined(HK_PocketQuad)
#define QUADX
#define DC_MOTORS
#define MPU6050
#define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  =  -Y; accADC[YAW]  =  Z;}
#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] = Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;}
#undef INTERNAL_I2C_PULLUPS
#endif

/**************************************************************************************/
/***************      Multitype decleration for the GUI's          ********************/
/**************************************************************************************/

#if defined(QUADP)
#define MULTITYPE 2
#elif defined(QUADX)
#define MULTITYPE 3
#endif

/**************************************************************************************/
/***************          Some unsorted "chain" defines            ********************/
/**************************************************************************************/

//all new Special RX's must be added here
//this is to avoid confusion :)

#if !defined(SERIAL_SUM_PPM) && !defined(SPEKTRUM) && !defined(SBUS)
#define STANDARD_RX
#endif

#if defined(SPEKTRUM)
#define SPEK_FRAME_SIZE 16
#if (SPEKTRUM == 1024)
#define SPEK_CHAN_SHIFT  2       // Assumes 10 bit frames, that is 1024 mode.
#define SPEK_CHAN_MASK   0x03    // Assumes 10 bit frames, that is 1024 mode.
#define SPEK_DATA_SHIFT          // Assumes 10 bit frames, that is 1024 mode.
#define SPEK_BIND_PULSES 3
#endif
#if (SPEKTRUM == 2048)
#define SPEK_CHAN_SHIFT  3       // Assumes 11 bit frames, that is 2048 mode.
#define SPEK_CHAN_MASK   0x07    // Assumes 11 bit frames, that is 2048 mode.
#define SPEK_DATA_SHIFT >> 1     // Assumes 11 bit frames, that is 2048 mode.
#define SPEK_BIND_PULSES 5
#endif
#endif // SPEKTRUM

#if defined(SBUS)
#define RC_CHANS 18
#elif defined(SPEKTRUM) 
#define RC_CHANS 12
#elif defined(SERIAL_SUM_PPM)
#define RC_CHANS 12
#else
#define RC_CHANS 8
#endif // SBUS

#if !defined(ACROTRAINER_MODE)
#define ACROTRAINER_MODE 1000
#endif

///////////////////////////////////////////////////////////////////////////////////////////////////
// End register definition 
///////////////////////////////////////////////////////////////////////////////////////////////////

/**************************************************************************************/
/********* enforce your sensors orientation - possibly overriding board defaults  *****/
/**************************************************************************************/
#ifdef FORCE_GYRO_ORIENTATION
#define GYRO_ORIENTATION FORCE_GYRO_ORIENTATION
#endif
#ifdef FORCE_ACC_ORIENTATION
#define ACC_ORIENTATION FORCE_ACC_ORIENTATION
#endif

/**************************************************************************************/
/***************               Error Checking Section              ********************/
/**************************************************************************************/

#ifndef NUMBER_MOTOR
#error "NUMBER_MOTOR is not set, most likely you have not defined any type of multicopter"
#endif

#if (!defined(PROMICRO) || ((NUMBER_MOTOR !=4) || defined(SERVO)))
#error "Implementation restriction: must be exactly 4 DC_MOTORS and no SERVOS and use Atmel 32u4 (Leonard)"
#endif


