/* 
 DFRobot LED Ring I2C Driver adaptation
 by Alexandre Dubus
*/

#include <Wire.h>
int8_t brightness[3][12];	/* 12 LEDS - 3 COLORS - 64 brightness levels per color   max brightness = 63*/

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
  if (red>63) red = 63; brightness[0][led] = red;
  if (green>63) green = 63; brightness[1][led] = green;
  if (blue>63) blue = 63; brightness[2][led] = blue;
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
  DDRB |=   ((1<<2)|(1<<3)|(1<<4)|(1<<5)|(1<<6)|(1<<7));    //transistors located on PIN B2->B7  : set PORTB as output
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

ISR (TIMER2_OVF_vect){
  uint8_t b,t,l,tmp;
  static uint8_t transistor[6]={0x20,0x10,0x08,0x04,0x80,0x40}; //transistor selection
  sei(); //it's important to release the CPU as soon as possible to not freeze I2C communications
  for (t = 0; t < 6; t++) {
    PORTB = transistor[t];
    l= 2*t;
    for (b = 0; b < 64; b++)    {
      tmp = 0;
      if (b < brightness[1][l])   tmp |= (1 << PORTC2);  else tmp &=~(1 << PORTC2); //green port C2
      if (b < brightness[2][l])   tmp |= (1 << PORTC1);  else tmp &=~(1 << PORTC1); //blue port C1
      if (b < brightness[0][l])   tmp |= (1 << PORTC0);  else tmp &=~(1 << PORTC0); //red port C0
      if (b < brightness[1][l+1]) tmp |= (1 << PORTD7);  else tmp &=~(1 << PORTD7); //green port D7
      if (b < brightness[2][l+1]) tmp |= (1 << PORTD6);  else tmp &=~(1 << PORTD6); //blue port D6
      if (b < brightness[0][l+1]) tmp |= (1 << PORTD5);  else tmp &=~(1 << PORTD5); //red port D5
      PORTC = tmp; PORTD = tmp;
    }
  }
  TCNT2 = 254;
}

uint8_t	param[10];
void receiveEvent(int16_t n) {
  uint8_t p=0;  
  while(Wire.available()) {
    param[p++]=Wire.receive();
    if (p>9) p=9;
  }
}


void setup() {
  InitIO();
  Wire.begin(0x6D);               // join i2c bus with address #2
  Wire.onReceive(receiveEvent);   // register event
}

void loop() {
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
            set_all_rgb(bright, 0, 0);delay(5);}
          for (bright = 0;bright<64;bright+=1) {
            set_all_rgb(63-bright, 0, 0);delay(5);}
        }
        for(i=0;i<3;i++) {
          for (bright = 0;bright<64;bright+=1) {
            set_all_rgb(0, bright, 0);delay(5);}
          for (bright = 0;bright<64;bright+=1) {
            set_all_rgb(0,63-bright, 0);delay(5);}
        }
        for(i=0;i<3;i++) {
          for (bright = 0;bright<64;bright+=1) {
            set_all_rgb(0,0,bright);delay(5);}
          for (bright = 0;bright<64;bright+=1) {
            set_all_rgb(0,0,63-bright);delay(5);}
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
        } else {
          set_all_unicolor(0, 0); 
        }
        break;
    }
    param[0]=0;
  }
}
