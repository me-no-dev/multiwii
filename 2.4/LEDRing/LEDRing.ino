/* 
 Multiwiicopter LED Ring
  - I2C implementation for Multiwii
  - standalone implementation 
  
 by Alexander Dubus
 adapted by Shikra
 
 For all documenation and files:
 http://code.google.com/p/ledring/ 
 
 v2.1.1  12 10 05
 + Reverse mag direction for inverted LEDring
 + BARO indicator wilst in GPS/sat indicator mode
 + MAG indicator whilst in GPS/sat indicator mode 
 + support for horizon mode
 - Correct bug - GPS/RTH indicator reversed
 
 UNTESTED elements...
 LEDboard v2 hardware
 Reverse LED
 
 ROADMAP elements...
 PWM control
 */



/*************************************************************************************************/
/****    CONFIGURABLE PARAMETERS                                                              ****/
/*************************************************************************************************/

/* Operation mode *///Choose only 1
#define MultiWii_I2C_v2 // - to use Multiwii 2.2rc LEDring functionality. (or 2.1 with alternative LEDring.ino
//#define MultiWii_I2C_v1 // - to use standard Multiwii 2.0/2.1 LED functionality. 
// #define Standalone              // - runs standalone using stitches to determine flashe sequences

/* The LED board type *///Choose only 1
//#define LEDBOARDv2 - untested
#define LEDBOARDv3

/* LED board inverted - uncomment to reverse MAG direction*///
//#define reverse_mag

/* The LED board I2C address *///
#define I2C_address 0x6D

/* PWM sequences */
#define PWM_LOW_LED 1
#define PWM_MID_LED 2
#define PWM_HIGH_LED 3

/* Only ever enable if you fully understand why!! */
//#define INTERNAL_I2C_PULLUPS


/****    Definitions   **************************************************************************/
#if defined (MultiWii_I2C_v2) || defined (MultiWii_I2C_v1)
#include <Wire.h>
#endif
#if defined Standalone       
#include <EEPROM.h>
#endif
uint8_t brightness[3][12];    //* 12 LEDS - 3 COLORS - 64 brightness levels per color   max brightness = 63 */
int16_t	param[10];            //* Parameters passed from Multiwii */
uint8_t singleLED = 0;
int16_t magLED = 0;
uint32_t ledlastflashtime = millis();
uint8_t Switch1_state=0;
uint8_t Switch2_state=0;
uint8_t Switch1 = 3;               //Arduino Pin-->switch assignment
uint8_t Switch2 = 4;               //Arduino Pin-->switch assignment  
uint8_t Switch1_val = 0;           //Switch toggle value. 
uint8_t Switch2_val = 1;           //Switch toggle value. 0=LED disabled/1=standalone or Normal wii controlled LED
uint32_t lastPressTime = millis(); //Last time button pressed


/****    Main program   **************************************************************************/
void setup() {
  InitIO();  
#if defined (MultiWii_I2C_v2) || defined (MultiWii_I2C_v1)
  Wire.begin(I2C_address);          // join i2c bus with address #2
  Wire.onReceive(receiveEvent);     // register event
#endif
#if defined Standalone       
  eepromread();
  pinMode(Switch1, INPUT);     
  pinMode(Switch2, INPUT);
  digitalWrite(Switch1, HIGH);
  digitalWrite(Switch2, HIGH);
#endif
#if defined PWM              
#endif
#if defined(INTERNAL_I2C_PULLUPS)
  PORTC |= 1<<4; 
  PORTC |= 1<<5;   // PIN A4&A5 (SDA&SCL)
#else
  PORTC &= ~(1<<4); 
  PORTC &= ~(1<<5);
#endif

  LED_sequence(15); // Default waiting status 
}


void loop() {
#if defined MultiWii_I2C_v2 
  I2C_enhanced_mode();
#endif
#if defined MultiWii_I2C_v1
  I2C_standard_mode();
#endif
#if defined Standalone       
  check_switch_input();          // Check for any keypress
  if (Switch2_val == 0){         // All lights off
    LED_sequence(99);
  }
  else if (Switch2_val == 1){    // Standalone mode (only if I2C not controlled by Wii) 
    Standalone_mode();
  }
#endif
#if defined PWM              
#endif
  delay(50); // a little time to settle...
}


/**** Standalone Control Subroutines  *******************************************************************************************/

#if defined Standalone       
void Standalone_mode(){
  if (Switch1_val==9) Switch1_val=10;
  if (Switch1_val==18) Switch1_val=20;
  if (Switch1_val>22) Switch1_val=0;
  LED_sequence(Switch1_val);
}


void check_switch_input(){  // Check if switch input detected
  uint8_t val1=digitalRead(Switch1);
  uint8_t val2=digitalRead(Switch2);
  if ((val1==LOW)||(val2==LOW))  {
    if ((millis()-lastPressTime)>500){
      if(val1==LOW){
        Switch1_val++;
        Switch2_val=1;
      }
      if(val2==LOW){
        if (Switch2_val==0) {
          Switch2_val=1;
        } 
        else {
          Switch2_val=0;
        } 
      }
      eepromwrite();
      lastPressTime = millis();  
    } 
    delay(250); 
  }
}


void eepromread(){ // Read last switch status on power up
  uint8_t eevalue = EEPROM.read(0);
  if (eevalue==104) {
    Switch1_val = EEPROM.read(1);
    //    Switch2_val = EEPROM.read(2); //Add if want to keep the on/off position
    delay(1000);
  }
  else {
    Switch1_val=14; // Default LED sequence
    Switch2_val=1; 
    eepromwrite();
  }
}


void eepromwrite(){ // Save switch status for next power up
  EEPROM.write(0,102);
  EEPROM.write(1,Switch1_val);
  //  EEPROM.write(2,Switch2_val); //Add if want to keep the on/off position
  delay(1000);

}
#endif


/**** I2C Control Subroutines  *******************************************************************************************/

#if defined MultiWii_I2C_v2
void I2C_enhanced_mode(){
  uint8_t i;
  switch (param[0]){
  case 'x': // Motors on - ACRO mode
    LED_sequence(11);    
    break;
  case 'y': // Motors on - HORIZON mode
    LED_sequence(14);    
    break;
  case 'w': // Motors on - GPS position hold mode
    LED_sequence(5);    
    break;
  case 'v': // Motors on - GPS RTH mode mode
    LED_sequence(2);    
    break;
  case 'u': // Motors on - Angle mode
    LED_sequence(14);    
    break;
  case 't': // Motors off - Uncalibrated acc / not level
    LED_sequence(4);    
    break;
  case 's': // Motors off - MultiWii sending status info
    if (param[1]==0&&param[2]==0&&param[3]==0&&param[4]==0){ //ACRO only
      LED_sequence(11);    
    }
    else if (param[4]>0){ // GPS mode
      param[6]=constrain (param[6],0,12);
      if (param[6]==0){ // No sats - circle LED
        LED_sequence(15);
      }
      else if (millis()<(ledlastflashtime+500)){ // Light SAT LED's
        for(uint8_t i=0;i<param[6];i++){
          set_led_rgb(i, 63 ,0, 0);
        }
        for(uint8_t i=param[6];i<12;i++){
          if (param[2]==1) { // baro on
            set_led_rgb (i,63, 63, 63);
          }
          else {
            set_led_rgb(i, 0 ,0, 0);
          }
          if (param[3]==1) { // mag on show the led
#if defined(reverse_mag)
            magLED=90+param[5];
#else
            magLED=90-param[5];
#endif
            if (magLED<0) magLED=magLED+180;
            set_led_rgb(magLED*2*12/360, 0 ,0, 63);
          }  
        }
      }
      else if ((millis()<(ledlastflashtime+1000)&&param[6]<5)){ // Flash LED's if < FIX sats
        set_all_rgb (0, 0, 0);
      }
      else{
        ledlastflashtime=millis();
      }
    }
    else{
      if (millis()<(ledlastflashtime+2000)){
        if (param[1]==1) {
          set_all_rgb (0, 63, 0);
        }
        else if (param[1]==2) {
          set_all_rgb (0, 63, 0);
          for(i=1;i<12;i++) {
            set_led_rgb(i, 0 ,0, 0);
            i++;
          }
        }
        else {
          set_all_rgb (63, 0, 0);
        }
        if (param[3]==1) { // mag on
#if defined(reverse_mag)
          magLED=90+param[5];
#else
          magLED=90-param[5];
#endif
          if (magLED<0) magLED=magLED+180;
          set_led_rgb(magLED*2*12/360, 0 ,0, 63);
        }  
      }
      else if (millis()<(ledlastflashtime+2250)){
        if (param[2]==1) { // baro on
          set_all_rgb (0, 0, 63);
        }
      }
      else {
        ledlastflashtime=millis();
      }
    }
    break;
  case 'r': //Battery Voltage low
    LED_sequence(7);    
    break;
  }
  param[0]=0;
}
#endif


#if defined MultiWii_I2C_v1
void I2C_standard_mode(){
  uint8_t i,bright;
  if ( 'a' <= param[0] &&  param[0] <= 'z') {
    switch (param[0]){
    case 'a':
      set_led_rgb(param[1], param[2], param[3],param[4]);                                      
      break;
    case 'b':
      set_all_rgb(param[1], param[2], param[3]);                                       
      break;
    case 'c':
      set_led_unicolor(param[1], param[2], param[3]);                                       
      break;
    case 'd':
      set_all_unicolor(param[1], param[2]);
      break;
    case 'e': //all black
      set_all_rgb(0,0,0);
      break;
    case 'f': //random , param: selected led
      set_led_rgb(param[1], random(63),random(63),random(63));
      break;
    case 'g': //random all led
      set_all_rgb(random(63),random(63),random(63));
      break;
    case 'h': //turnover 2 params: color , direction
      turnover(param[1],param[2]);
      break;
    case 'i': // one effect
      set_all_rgb(0,0,0);
      for(i=0;i<3;i++) {
        for (bright = 0;bright<64;bright+=1) {
          set_all_rgb(bright, 0, 0);
          delay(5);
        }
        for (bright = 0;bright<64;bright+=1) {
          set_all_rgb(63-bright, 0, 0);
          delay(5);
        }
      }
      for(i=0;i<3;i++) {
        for (bright = 0;bright<64;bright+=1) {
          set_all_rgb(0, bright, 0);
          delay(5);
        }
        for (bright = 0;bright<64;bright+=1) {
          set_all_rgb(0,63-bright, 0);
          delay(5);
        }
      }
      for(i=0;i<3;i++) {
        for (bright = 0;bright<64;bright+=1) {
          set_all_rgb(0,0,bright);
          delay(5);
        }
        for (bright = 0;bright<64;bright+=1) {
          set_all_rgb(0,0,63-bright);
          delay(5);
        }
      }
      set_all_rgb(0,0,0);
      break;
    case 'j': // one effect
      set_led_rgb(2, 20,0, 0);
      set_led_rgb(3, 63,0, 0);
      set_led_rgb(1, 0,10, 0);
      set_led_rgb(0, 0,0,30);
      for (i = 0; i < 100; i++) {
        turnover(0,2);
        turnover(1,1);
        turnover(2,2);
        delay(90);
      }
      break;
    case 'k': //strobe 2 params: number, delay
      set_all_rgb(0,0,0);
      for(i=0;i<param[1];i++) {
        set_all_rgb(63,63,63);
        delay(param[2]);
        set_all_rgb(0,0,0);
        delay(param[2]);
      }
      set_all_rgb(0,0,0);
      break;
    case 'z': //multiwii heading 1 param: heading [0;11]
      set_all_unicolor(2, 0); // all BLUE LEDs black
      set_led_unicolor(param[1]*2*12/360, 2, 63);
      break;
    case 'y': //multiwii angles  2 params: angle ROLL [0;180] ; angle PITCH [0;180]
      uint8_t l[12];
      uint8_t right,left,up,down;
      float a;
      a = atan2(param[1]-90,90-param[2])*180/PI;
      if (abs(param[1]-90) >2 || abs(param[2]-90) > 2) {
        uint8_t f = max(abs(param[1]-90),abs(param[2]-90));
        for(i=0;i<12;i++) {
          uint8_t p = 12-(a+180)*12/360;
          if ( i == p ) set_led_unicolor(i, 0, 1+60*f/90);
          else if ( i == (p +2)%12 )  set_led_unicolor(i, 0, 1);
          else if ( i == (p +1)%12 )  set_led_unicolor(i, 0, 1+8*f/90);
          else if ( i == (p +11)%12 ) set_led_unicolor(i, 0, 1+8*f/90);
          else if ( i == (p +10)%12 ) set_led_unicolor(i, 0, 1);
          else set_led_unicolor(i, 0, 0);
        }
      } 
      else {
        set_all_unicolor(0, 0); 
      }
      break;
    }
  }
  param[0]=0;
}
#endif


#if defined (MultiWii_I2C_v2) || defined (MultiWii_I2C_v1)
void receiveEvent(int16_t n) {
  uint8_t p=0;  
  while(Wire.available()) {
    param[p++]=Wire.read();
    if (p>9) p=9;
  }
}
#endif

/**** Hardware Subroutines  *******************************************************************************************/


#if defined LEDBOARDv3
ISR (TIMER2_OVF_vect){
  uint8_t b,t,l,tmp;
  static uint8_t transistor[6]={
    0x01,0x02,0x04,0x08,0x10,0x20                                                    }; //transistor selection
  sei(); //it's important to release the CPU as soon as possible to not freeze I2C communications
  for (t = 0; t < 6; t++) {
    PORTB = transistor[t];
    l= 2*t;
    for (b = 0; b < 64; b++)    {
      tmp = 0;
      if (b < brightness[0][l])   tmp |= (1 << PORTD5);  
      else tmp &=~(1 << PORTD5); //red port C0
      if (b < brightness[0][l+1]) tmp |= (1 << PORTC1);  
      else tmp &=~(1 << PORTC1); //red port D5

      if (b < brightness[1][l])   tmp |= (1 << PORTD6);  
      else tmp &=~(1 << PORTD6); //green port C2
      if (b < brightness[1][l+1]) tmp |= (1 << PORTC0);  
      else tmp &=~(1 << PORTC0); //green port D7

      if (b < brightness[2][l])   tmp |= (1 << PORTD7);  
      else tmp &=~(1 << PORTD7); //blue port C1
      if (b < brightness[2][l+1]) tmp |= (1 << PORTC2);  
      else tmp &=~(1 << PORTC2); //blue port D6

      PORTC = tmp; 
      PORTD = tmp|0x18;
    }
  }
  TCNT2 = 254;
}
#endif


#if defined LEDBOARDv2
ISR (TIMER2_OVF_vect){
  uint8_t b,t,l,tmp;
  static uint8_t transistor[6]={
    0x20,0x10,0x08,0x04,0x80,0x40                                                      }; //transistor selection
  sei(); //it's important to release the CPU as soon as possible to not freeze I2C communications
  for (t = 0; t < 6; t++) {
    PORTB = transistor[t];
    l= 2*t;
    for (b = 0; b < 64; b++)    {
      tmp = 0;
      if (b < brightness[1][l])   tmp |= (1 << PORTC2);  
      else tmp &=~(1 << PORTC2); //green port C2
      if (b < brightness[2][l])   tmp |= (1 << PORTC1);  
      else tmp &=~(1 << PORTC1); //blue port C1
      if (b < brightness[0][l])   tmp |= (1 << PORTC0);  
      else tmp &=~(1 << PORTC0); //red port C0
      if (b < brightness[1][l+1]) tmp |= (1 << PORTD7);  
      else tmp &=~(1 << PORTD7); //green port D7
      if (b < brightness[2][l+1]) tmp |= (1 << PORTD6);  
      else tmp &=~(1 << PORTD6); //blue port D6
      if (b < brightness[0][l+1]) tmp |= (1 << PORTD5);  
      else tmp &=~(1 << PORTD5); //red port D5
      PORTC = tmp; 
      PORTD = tmp|0x18;
    }
  }
  TCNT2 = 254;
}
#endif


/**** LED Subroutines  *******************************************************************************************/

void turnover(uint8_t rgb,uint8_t dir){
  uint8_t led, temp, i;
  if(rgb>2) return;
  if(dir==1){
    temp=brightness[rgb][0]; 
    for (led = 0; led < 11; led++)    {
      brightness[rgb][led]=brightness[rgb][led+1];
    }
    brightness[rgb][led]=temp;
  }
  if(dir==2){
    temp=brightness[rgb][11];
    for (led = 11; led >0; led--)    {
      brightness[rgb][led]=brightness[rgb][led-1];
    }
    brightness[rgb][0]=temp;
  }
}

void set_led_rgb (uint8_t led, uint8_t red, uint8_t green, uint8_t blue){
  if (led>11) return;
  if (red>63) red = 63; 
  brightness[0][led] = red;
  if (green>63) green = 63; 
  brightness[1][led] = green;
  if (blue>63) blue = 63; 
  brightness[2][led] = blue;
}

void set_all_rgb (uint8_t red, uint8_t green, uint8_t blue) {
  uint8_t led;
  for (led = 0; led < 12; led++)    {
    set_led_rgb (led, red, green, blue);
  }
}

void set_led_unicolor(uint8_t led, uint8_t rgb, uint8_t var){
  if(rgb>2 || led>11) return;
  if (var>63) var = 63;
  brightness[rgb][led] = var;
}

void set_all_unicolor(uint8_t rgb, uint8_t var){
  uint8_t led;
  if (var>63) var = 63;
  for (led = 0; led < 12; led++)    {
    set_led_unicolor (led, rgb, var);
  }
}

void InitIO(void){
#if defined LEDBOARDv3
  DDRB |=   ((1<<0)|(1<<1)|(1<<2)|(1<<3)|(1<<4)|(1<<5));    //transistors located on PIN B0->B5  : set PORTB as output
#else if
  DDRB |=   ((1<<2)|(1<<3)|(1<<4)|(1<<5)|(1<<6)|(1<<7));    //transistors located on PIN B2->B7  : set PORTB as output
#endif
  PORTB &=~ ((1<<2)|(1<<3)|(1<<4)|(1<<5)|(1<<6)|(1<<7));    // all pins HIGH --> cathodes HIGH --> LEDs off
  DDRC |= ((1 << PORTC0) | (1 << PORTC2) | (1 << PORTC1));  // R G B LEDs on port C : set COLORPORT #5-7 as output
  DDRD |= ((1 << PORTD5) | (1 << PORTD7) | (1 << PORTD6));  // R G B LEDs on port D : set COLORPORT #5-7 as output
  set_all_rgb (0, 0, 0);
  // Timer2 Settings
  // prescaler (frequency divider)
  TCCR2B |= ((1 << CS22) | (1 << CS20) | ((1 << CS21))); //1024
  //normal mode
  TCCR2B &=~(1 << WGM22);
  TCCR2A =0;
  // enable_timer2_ovf
  TIMSK2 |= (1 << TOIE2);
}

/****    LED sequences    **************************************************************************/

void LED_sequence(uint8_t LED_seq){
  uint8_t i;
  uint8_t bright;

  if (LED_seq == 0) { // ***** Static RED
    set_all_rgb (63, 0, 0);
  }
  if (LED_seq == 1) { // ***** Static GREEN
    set_all_rgb (0, 63, 0);
  }  
  if (LED_seq == 2) { // ***** Static BLUE
    set_all_rgb (0, 0, 63);
  }
  if (LED_seq == 3) { // ***** Flashing Red
    if (millis()<(ledlastflashtime+500)){
      set_all_rgb (63, 0, 0);
    }
    else if (millis()<(ledlastflashtime+1000)){
      set_all_rgb (0, 0, 0);
    }
    else {
      ledlastflashtime=millis();
    }
  }
  if (LED_seq == 4) { // ***** Flashing Green
    if (millis()<(ledlastflashtime+500)){
      set_all_rgb (0, 63, 0);
    }
    else if (millis()<(ledlastflashtime+1000)){
      set_all_rgb (0, 0, 0);
    }
    else {
      ledlastflashtime=millis();
    }
  }
  if (LED_seq == 5) { // ***** Flashing Blue
    if (millis()<(ledlastflashtime+500)){
      set_all_rgb (0, 0, 63);
    }
    else if (millis()<(ledlastflashtime+1000)){
      set_all_rgb (0, 0, 0);
    }
    else {
      ledlastflashtime=millis();
    }
  }
  if (LED_seq == 6) { // ***** Fast RED Flash
    if (millis()<(ledlastflashtime+250)){
      set_all_rgb (63, 0, 0);
    }
    else if (millis()<(ledlastflashtime+500)){
      set_all_rgb (0, 0, 0);
    }
    else {
      ledlastflashtime=millis();
    }
  }
  if (LED_seq == 7) { // ***** Fast GREEN Flash
    if (millis()<(ledlastflashtime+250)){
      set_all_rgb (0, 63, 0);
    }
    else if (millis()<(ledlastflashtime+500)){
      set_all_rgb (0, 0, 0);
    }
    else {
      ledlastflashtime=millis();
    }
  }
  if (LED_seq == 8) { // ***** Fast Blue Flash
    if (millis()<(ledlastflashtime+250)){
      set_all_rgb (0, 0, 63);
    }
    else if (millis()<(ledlastflashtime+500)){
      set_all_rgb (0, 0, 0);
    }
    else {
      ledlastflashtime=millis();
    }
  }
  if (LED_seq == 10) { // *****  - navi lights
    if (millis()<(ledlastflashtime+200)){
      set_all_rgb (63, 63, 63);
    }
    else if (millis()<(ledlastflashtime+700)){
      LED_layout(0);
    }
    else if (millis()<(ledlastflashtime+900)){
      set_all_rgb (63, 63, 63);
    }
    else if (millis()<(ledlastflashtime+1400)){
      LED_layout(0);
    }
    else if (millis()<(ledlastflashtime+1600)){
      set_all_rgb (63, 0, 0);
    }
    else if (millis()<(ledlastflashtime+3300)){
      LED_layout(0);
    }
    else {
      ledlastflashtime=millis();
    }
  }
  if (LED_seq == 11) { // *****  - navi lights #2 MWC
    if (millis()<(ledlastflashtime+200)){
      set_all_rgb (63, 63, 63);
    }
    else if (millis()<(ledlastflashtime+700)){
      LED_layout(1);
    }
    else if (millis()<(ledlastflashtime+900)){
      set_all_rgb (63, 63, 63);
    }
    else if (millis()<(ledlastflashtime+1400)){
      LED_layout(1);
    }
    else if (millis()<(ledlastflashtime+1600)){
      set_all_rgb (63, 0, 0);
    }
    else if (millis()<(ledlastflashtime+3300)){
      LED_layout(1);
    }
    else {
      ledlastflashtime=millis();
    }
  }
  if (LED_seq == 12) { // ***** Flashing RED+GREEN. Red facing forward
    if (millis()<(ledlastflashtime+500)){
      for(i=10;i<12;i++) {
        set_led_rgb(i, 63 ,0, 0);
      }
      for(i=0;i<3;i++) {
        set_led_rgb(i, 63, 0, 0);
      }
    }
    else if (millis()<(ledlastflashtime+1000)){
      set_all_rgb (0, 0, 0);
    }
    else {
      set_all_rgb (0, 63, 0);
      ledlastflashtime=millis();
    }
  }
  if (LED_seq == 13) { // ***** Static RED+GREEN. Red facing forward
    for(i=0;i<3;i++) {
      set_led_rgb(i, 63, 0, 0);
    }
    for(i=3;i<10;i++) {
      set_led_rgb(i, 0, 63, 0);
    }
    for(i=10;i<12;i++) {
      set_led_rgb(i, 63 ,0, 0);
    }
  }
  if (LED_seq == 14) { // ***** Andromeda layout
    for(i=0;i<2;i++) {
      set_led_rgb(i, 0, 0, 63);
    }
    for(i=2;i<5;i++) {
      set_led_rgb(i, 63, 63, 63);
    }
    for(i=5;i<8;i++) {
      set_led_rgb(i, 63, 0, 0);
    }
    for(i=8;i<11;i++) {
      set_led_rgb(i, 63 ,63, 63);
    }
    set_led_rgb(11, 0, 0, 63);
  }
  if (LED_seq == 15) { // ***** Circling Red on White
    if (millis()<(ledlastflashtime+125)){
    }
    else{
      singleLED++;
      if (singleLED>11) singleLED=0;
      set_all_rgb(63,63,63);
      ledlastflashtime=millis();
      set_led_rgb(singleLED, 63, 0, 0);
    }
  }
  if (LED_seq == 16) { // ***** Circling Green on White
    if (millis()<(ledlastflashtime+125)){
    }
    else{
      singleLED++;
      if (singleLED>11) singleLED=0;
      set_all_rgb(63,63,63);
      ledlastflashtime=millis();
      set_led_rgb(singleLED, 0, 63, 0);
    }
  }
  if (LED_seq == 17) { // ***** Circling Blue on White
    if (millis()<(ledlastflashtime+125)){
    }
    else{
      singleLED++;
      if (singleLED>11) singleLED=0;
      set_all_rgb(63,63,63);
      ledlastflashtime=millis();
      set_led_rgb(singleLED, 0, 0, 63);
    }
  }
  if (LED_seq == 20) { // ***** Alexander the great effect 1
    set_all_rgb(0,0,0);
    for(i=0;i<3;i++) {
      for (bright = 0;bright<64;bright+=1) {
        set_all_rgb(bright, 0, 0);
        delay(5);
      }
      for (bright = 0;bright<64;bright+=1) {
        set_all_rgb(63-bright, 0, 0);
        delay(5);
      }
    }
    for(i=0;i<3;i++) {
      for (bright = 0;bright<64;bright+=1) {
        set_all_rgb(0, bright, 0);
        delay(5);
      }
      for (bright = 0;bright<64;bright+=1) {
        set_all_rgb(0,63-bright, 0);
        delay(5);
      }
    }
    for(i=0;i<3;i++) {
      for (bright = 0;bright<64;bright+=1) {
        set_all_rgb(0,0,bright);
        delay(5);
      }
      for (bright = 0;bright<64;bright+=1) {
        set_all_rgb(0,0,63-bright);
        delay(5);
      }
    }
    set_all_rgb(0,0,0);
  }
  if (LED_seq == 21) { // ***** Alexander the great effect 2
    set_led_rgb(2, 20,0, 0);
    set_led_rgb(3, 63,0, 0);
    set_led_rgb(1, 0,10, 0);
    set_led_rgb(0, 0,0,30);
    for (i = 0; i < 100; i++) {
      turnover(0,2);
      turnover(1,1);
      turnover(2,2);
      delay(90);
    }
  }
  if (LED_seq == 22) { // ***** Randomised colours
    set_all_rgb(random(63),random(63),random(63));
  }
  if (LED_seq == 99) { // ***** All off
    set_all_rgb (0, 0, 0);
  }
}


/****    LED layouts    **************************************************************************/

void LED_layout(uint8_t LED_seq){
  if (LED_seq == 0) { // *****  std navi light layout
    set_led_rgb (0,0,0,0);
    set_led_rgb (1,0,0,0);
    set_led_rgb (2,63,0,0);
    set_led_rgb (3,63,0,0);
    set_led_rgb (4,0,0,0);
    set_led_rgb (5,0,0,0);
    set_led_rgb (6, 63,63,63);
    set_led_rgb (7,0,0,0);
    set_led_rgb (8,0,0,0);
    set_led_rgb (9,0,63,0);
    set_led_rgb (10,0,63,0);
    set_led_rgb (11,0,0,0);
  }
  if (LED_seq == 1) { // *****  MWC navi light layout
    set_led_rgb (0,0,0,63);
    set_led_rgb (1,0,0,63);
    set_led_rgb (2,63,63,63);
    set_led_rgb (3,63,63,63);
    set_led_rgb (4,0,0,0);
    set_led_rgb (5,63,0,0);
    set_led_rgb (6, 63,0,0);
    set_led_rgb (7,63,0,0);
    set_led_rgb (8,0,0,0);
    set_led_rgb (9,63,63,63);
    set_led_rgb (10,63,63,63);
    set_led_rgb (11,0,0,63);
  }
}








































