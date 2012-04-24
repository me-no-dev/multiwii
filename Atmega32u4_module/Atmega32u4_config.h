
/********************************************************************/
/*** Settings for ProMicro, Leonardo and other Atmega32u4 Boards ****/
/********************************************************************/

  // activate this for a better pinlayout if all pins can be used => not possible on ProMicro!
  //#define A32U4ALLPINS

  // activate all 6 hardware PWM outputs Motor 5 = D11 and 6 = D13. => not possible on ProMicro! 
  // if activated: 
  // Motor 1-6 = 10-bit hardware PWM
  // Motor 7-8 = 8-bit Software PWM
  // Servos    = 8-bit Software PWM
  // if deactivated:
  // Motor 1-4 = 10-bit hardware PWM
  // Motor 5-8 = 10-bit Software PWM
  // Servos    = 10-bit Software PWM
  //#define HWPWM6

  // aux2 pin on pin RXO 
  //#define RCAUX2PINRXO

  // aux2 pin on pin D17 (RXLED)
  //#define RCAUX2PIND17

  // this moves the Buzzer pin from TXO to D8 for use with ppm sum or spectrum sat. RX (not needed if A32U4ALLPINS is active)
  //#define D8BUZZER

  // Inverted status LED for Promicro ver 10.
  //#define PROMICRO10
  
/* end of Settings for ProMicro, Leonardo and other Atmega32u4 Boards */
/*--------------------------------------------------------------------*/



/********************************************************************/
/****           Atmega32u4 defines                               ****/
/********************************************************************/
#define LEDPIN_PINMODE               
#define LEDPIN_TOGGLE                PIND |= 1<<5;     //switch LEDPIN state (Port D5)
#if defined(PROMICRO10)
  #define LEDPIN_OFF                 PORTD &= ~(1<<5);
  #define LEDPIN_ON                  PORTD |= (1<<5);
#else
  #define LEDPIN_OFF                 PORTD |= (1<<5);
  #define LEDPIN_ON                  PORTD &= ~(1<<5);  
#endif
#if defined(D8BUZZER)
  #define BUZZERPIN_PINMODE          pinMode (8, OUTPUT);
  #define BUZZERPIN_ON               PORTB |= 1<<4;
  #define BUZZERPIN_OFF              PORTB &= ~(1<<4); 
#elif defined(A32U4ALLPINS)
  #define BUZZERPIN_PINMODE          pinMode (4, OUTPUT);
  #define BUZZERPIN_ON               PORTD |= 1<<4;
  #define BUZZERPIN_OFF              PORTD &= ~(1<<4);    
#else
  #define BUZZERPIN_PINMODE          pinMode (1, OUTPUT);
  #define BUZZERPIN_ON               PORTD |= 1<<3;
  #define BUZZERPIN_OFF              PORTD &= ~(1<<3); 
#endif
#define POWERPIN_PINMODE           
#define POWERPIN_ON                
#define POWERPIN_OFF               
#define I2C_PULLUPS_ENABLE           PORTD |= 1<<0; PORTD |= 1<<1;   // PIN 2&3 (SDA&SCL)
#define I2C_PULLUPS_DISABLE          PORTD &= ~(1<<0); PORTD &= ~(1<<1);
#define PINMODE_LCD                  pinMode(0, OUTPUT);
#define LCDPIN_OFF                   PORTD &= ~1;
#define LCDPIN_ON                    PORTD |= 1;
#define STABLEPIN_PINMODE            ;
#define STABLEPIN_ON                 ;
#define STABLEPIN_OFF                ;
#define PPM_PIN_INTERRUPT            pinMode(7,INPUT);PORTE |= (1 << 6);EIMSK |= (1 << INT6);EICRB |= (1 << ISC61)|(1 << ISC60);
#define SPEK_SERIAL_VECT             USART1_RX_vect
#define SPEK_DATA_REG                UDR1
#define USB_CDC_TX                   3
#define USB_CDC_RX                   2

//soft PWM Pins  
#define SOFT_PWM_1_PIN_HIGH          PORTD |= 1<<4;
#define SOFT_PWM_1_PIN_LOW           PORTD &= ~(1<<4);
#define SOFT_PWM_2_PIN_HIGH          PORTF |= 1<<5;
#define SOFT_PWM_2_PIN_LOW           PORTF &= ~(1<<5);
#define SOFT_PWM_3_PIN_HIGH          PORTF |= 1<<7;
#define SOFT_PWM_3_PIN_LOW           PORTF &= ~(1<<7);
#define SOFT_PWM_4_PIN_HIGH          PORTF |= 1<<6;
#define SOFT_PWM_4_PIN_LOW           PORTF &= ~(1<<6);

// Servos
#define SERVO_1_PINMODE              pinMode(A0,OUTPUT);
#define SERVO_1_PIN_HIGH             PORTF|= 1<<7;
#define SERVO_1_PIN_LOW              PORTF &= ~(1<<7);
#define SERVO_2_PINMODE              pinMode(A1,OUTPUT);
#define SERVO_2_PIN_HIGH             PORTF |= 1<<6;
#define SERVO_2_PIN_LOW              PORTF &= ~(1<<6);
#define SERVO_3_PINMODE              pinMode(A2,OUTPUT);
#define SERVO_3_PIN_HIGH             PORTF |= 1<<5;
#define SERVO_3_PIN_LOW              PORTF &= ~(1<<5);
#if defined(A32U4ALLPINS)
  #define SERVO_4_PINMODE            pinMode(A3,OUTPUT);
  #define SERVO_4_PIN_HIGH           PORTF |= 1<<4;
  #define SERVO_4_PIN_LOW            PORTF &= ~(1<<4);  
#else
  #define SERVO_4_PINMODE            pinMode(4,OUTPUT);
  #define SERVO_4_PIN_HIGH           PORTD |= 1<<4;
  #define SERVO_4_PIN_LOW            PORTD &= ~(1<<4);
#endif
#define SERVO_5_PINMODE              pinMode(6,OUTPUT);
#define SERVO_5_PIN_HIGH             PORTD |= 1<<7;
#define SERVO_5_PIN_LOW              PORTD &= ~(1<<7);
#define SERVO_6_PINMODE              pinMode(5,OUTPUT);
#define SERVO_6_PIN_HIGH             PORTC|= 1<<6;
#define SERVO_6_PIN_LOW              PORTC &= ~(1<<6);
#define SERVO_7_PINMODE              pinMode(10,OUTPUT);
#define SERVO_7_PIN_HIGH             PORTB |= 1<<6;
#define SERVO_7_PIN_LOW              PORTB &= ~(1<<6);
#define SERVO_8_PINMODE              pinMode(9,OUTPUT);
#define SERVO_8_PIN_HIGH             PORTB |= 1<<5;
#define SERVO_8_PIN_LOW              PORTB &= ~(1<<5);
//Standart RX
#define THROTTLEPIN                7
#if defined(A32U4ALLPINS)
  #define ROLLPIN                    6
  #define PITCHPIN                   2
  #define YAWPIN                     4
  #define AUX1PIN                    5
  #define PCINT_PIN_COUNT            4
  #define PCINT_RX_BITS              (1<<1),(1<<2),(1<<3),(1<<4)
#else
  #define ROLLPIN                    4
  #define PITCHPIN                   5
  #define YAWPIN                     2
  #define AUX1PIN                    6
  #define PCINT_PIN_COUNT            4
  #define PCINT_RX_BITS              (1<<1),(1<<2),(1<<3),(1<<4)
#endif
#define AUX2PIN                      0 
#define AUX3PIN                      1 // unused 
#define AUX4PIN                      3 // unused 
#define PCINT_RX_PORT                PORTB
#define PCINT_RX_MASK                PCMSK0
#define PCIR_PORT_BIT                (1<<0)
#define RX_PC_INTERRUPT              PCINT0_vect
#define RX_PCINT_PIN_PORT            PINB
// we use a INT pin for trottle
#define SPECIAL_PINS
//PWM
#if defined(HWPWM6)
  #define SERVO_ISR                  TIMER0_COMPA_vect
  #define SERVO_CHANNEL              OCR0A
  #define SERVO_1K_US                250
#else
  #define SERVO_ISR                  TIMER3_COMPA_vect
  #define SERVO_CHANNEL              OCR3A
  #define SERVO_1K_US                16000
#endif
#if defined(HWPWM6)
  #define SOFT_PWM_ISR1              TIMER0_COMPB_vect
  #define SOFT_PWM_ISR2              TIMER0_COMPA_vect
  #define SOFT_PWM_CHANNEL1          OCR0B
  #define SOFT_PWM_CHANNEL2          OCR0A 
#else
  #define SOFT_PWM_ISR1              TIMER3_COMPB_vect
  #define SOFT_PWM_ISR2              TIMER3_COMPC_vect
  #define SOFT_PWM_CHANNEL1          OCR3B
  #define SOFT_PWM_CHANNEL2          OCR3C   
#endif
// may use SW PWM
#define SWPWM

#define ISR_UART                     ISR(USART_UDRE_vect)
#if defined(A32U4ALLPINS)
  #define V_BATPIN                   A4    // Analog PIN 3
#else
  #define V_BATPIN                   A3    // Analog PIN 4
#endif
#define PSENSORPIN                   A2    // Analog PIN 2 
#define USB_OB


