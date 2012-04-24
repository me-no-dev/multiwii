
/********************************************************************/
/***    Settings for Pro Mini and other atmega328P boards        ****/
/********************************************************************/

/* PIN A0 and A1 instead of PIN D5 & D6 for 6 motors config and promini config
   This mod allow the use of a standard receiver on a pro mini
   (no need to use a PPM sum receiver)
*/
//#define A0_A1_PIN_HEX

/* possibility to use PIN8 or PIN12 as the AUX2 RC input
   it deactivates in this case the POWER PIN (pin 12) or the BUZZER PIN (pin 8)
*/
//#define RCAUXPIN8
//#define RCAUXPIN12
  
/*      end of Settings for Pro Mini and other atmega328P boards      */
/*--------------------------------------------------------------------*/



/********************************************************************/
/****               Atmega328P defines                           ****/
/********************************************************************/
#define LEDPIN_PINMODE             pinMode (13, OUTPUT);
#define LEDPIN_TOGGLE              PINB |= 1<<5;     //switch LEDPIN state (digital PIN 13)
#define LEDPIN_OFF                 PORTB &= ~(1<<5);
#define LEDPIN_ON                  PORTB |= (1<<5);
#if !defined(RCAUXPIN8)
  #define BUZZERPIN_PINMODE          pinMode (8, OUTPUT);
  #define BUZZERPIN_ON               PORTB |= 1;
  #define BUZZERPIN_OFF              PORTB &= ~1;
#else
  #define BUZZERPIN_PINMODE          ;
  #define BUZZERPIN_ON               ;
  #define BUZZERPIN_OFF              ;
  #define RCAUXPIN
#endif
#if !defined(RCAUXPIN12)
  #define POWERPIN_PINMODE           pinMode (12, OUTPUT);
  #define POWERPIN_ON                PORTB |= 1<<4;
  #define POWERPIN_OFF               PORTB &= ~(1<<4); //switch OFF WMP, digital PIN 12
#else
  #define POWERPIN_PINMODE           ;
  #define POWERPIN_ON                ;
  #define POWERPIN_OFF               ;
  #define RCAUXPIN
#endif
#define I2C_PULLUPS_ENABLE         PORTC |= 1<<4; PORTC |= 1<<5;   // PIN A4&A5 (SDA&SCL)
#define I2C_PULLUPS_DISABLE        PORTC &= ~(1<<4); PORTC &= ~(1<<5);
#define PINMODE_LCD                pinMode(0, OUTPUT);
#define LCDPIN_OFF                 PORTD &= ~1; //switch OFF digital PIN 0
#define LCDPIN_ON                  PORTD |= 1;
#define STABLEPIN_PINMODE          ;
#define STABLEPIN_ON               ;
#define STABLEPIN_OFF              ;
#define PPM_PIN_INTERRUPT          attachInterrupt(0, rxInt, RISING); //PIN 0
#define SPEK_SERIAL_VECT           USART_RX_vect
#define SPEK_DATA_REG              UDR0
//RX PIN assignment inside the port //for PORTD
#define THROTTLEPIN                2
#define ROLLPIN                    4
#define PITCHPIN                   5
#define YAWPIN                     6
#define AUX1PIN                    7
#define AUX2PIN                    0 // optional PIN 8 or PIN 12
#define AUX3PIN                    1 // unused 
#define AUX4PIN                    3 // unused 
  
#define PCINT_PIN_COUNT              5
#define PCINT_RX_BITS                (1<<2),(1<<4),(1<<5),(1<<6),(1<<7)
#define PCINT_RX_PORT                PORTD
#define PCINT_RX_MASK                PCMSK2
#define PCIR_PORT_BIT                (1<<2)
#define RX_PC_INTERRUPT              PCINT2_vect
#define RX_PCINT_PIN_PORT            PIND
// we use a INT pin for trottle
#define SPECIAL_PINS

#define ISR_UART                   ISR(USART_UDRE_vect)
#define V_BATPIN                   A3    // Analog PIN 3
#define PSENSORPIN                 A2    // Analog PIN 2

#if defined(A0_A1_PIN_HEX) && (NUMBER_MOTOR < 8)
  #define SOFT_PWM_1_PIN_HIGH        PORTC |= 1<<0;
  #define SOFT_PWM_1_PIN_LOW         PORTC &= ~(1<<0);
  #define SOFT_PWM_2_PIN_HIGH        PORTC |= 1<<1;
  #define SOFT_PWM_2_PIN_LOW         PORTC &= ~(1<<1);  
#else
  #define SOFT_PWM_1_PIN_HIGH        PORTD |= 1<<5;
  #define SOFT_PWM_1_PIN_LOW         PORTD &= ~(1<<5);
  #define SOFT_PWM_2_PIN_HIGH        PORTD |= 1<<6;
  #define SOFT_PWM_2_PIN_LOW         PORTD &= ~(1<<6);
#endif
#define SOFT_PWM_3_PIN_HIGH        PORTC |= 1<<2;
#define SOFT_PWM_3_PIN_LOW         PORTC &= ~(1<<2);
#define SOFT_PWM_4_PIN_HIGH        PORTB |= 1<<4;
#define SOFT_PWM_4_PIN_LOW         PORTB &= ~(1<<4);
#if defined(A0_A1_PIN_HEX) && defined(SERVO_TILT)
  #define SERVO_1_PINMODE            pinMode(A2,OUTPUT); // TILT_PITCH - WING left
  #define SERVO_1_PIN_HIGH           PORTC |= 1<<2;
  #define SERVO_1_PIN_LOW            PORTC &= ~(1<<2);
  #define SERVO_2_PINMODE            pinMode(12,OUTPUT); // TILT_ROLL  - WING right
  #define SERVO_2_PIN_HIGH           PORTB |= 1<<4;
  #define SERVO_2_PIN_LOW            PORTB &= ~(1<<4);
#else
  #define SERVO_1_PINMODE            pinMode(A0,OUTPUT); // TILT_PITCH - WING left
  #define SERVO_1_PIN_HIGH           PORTC |= 1<<0;
  #define SERVO_1_PIN_LOW            PORTC &= ~(1<<0);
  #define SERVO_2_PINMODE            pinMode(A1,OUTPUT); // TILT_ROLL  - WING right
  #define SERVO_2_PIN_HIGH           PORTC |= 1<<1;
  #define SERVO_2_PIN_LOW            PORTC &= ~(1<<1);
#endif
#define SERVO_3_PINMODE            pinMode(A2,OUTPUT); // CAM TRIG  - alt TILT_PITCH
#define SERVO_3_PIN_HIGH           PORTC |= 1<<2;
#define SERVO_3_PIN_LOW            PORTC &= ~(1<<2);
#define SERVO_4_PINMODE            pinMode(12,OUTPUT); // new       - alt TILT_ROLL
#define SERVO_4_PIN_HIGH           PORTB |= 1<<4;
#define SERVO_4_PIN_LOW            PORTB &= ~(1<<4);
#define SERVO_5_PINMODE            pinMode(11,OUTPUT); // BI LEFT
#define SERVO_5_PIN_HIGH           PORTB |= 1<<3;
#define SERVO_5_PIN_LOW            PORTB &= ~(1<<3);
#define SERVO_6_PINMODE            pinMode(3,OUTPUT);  // TRI REAR - BI RIGHT
#define SERVO_6_PIN_HIGH           PORTD|= 1<<3;
#define SERVO_6_PIN_LOW            PORTD &= ~(1<<3);
#define SERVO_7_PINMODE            pinMode(10,OUTPUT); // new
#define SERVO_7_PIN_HIGH           PORTB |= 1<<2;
#define SERVO_7_PIN_LOW            PORTB &= ~(1<<2);
#define SERVO_8_PINMODE            pinMode(9,OUTPUT); // new
#define SERVO_8_PIN_HIGH           PORTB |= 1<<1;
#define SERVO_8_PIN_LOW            PORTB &= ~(1<<1);
//PWM

#define SERVO_ISR                  TIMER0_COMPA_vect
#define SERVO_CHANNEL              OCR0A
#define SERVO_1K_US                250

#define SOFT_PWM_ISR1              TIMER0_COMPB_vect
#define SOFT_PWM_ISR2              TIMER0_COMPA_vect
#define SOFT_PWM_CHANNEL1          OCR0B
#define SOFT_PWM_CHANNEL2          OCR0A 

// may use SW PWM
#define SWPWM

