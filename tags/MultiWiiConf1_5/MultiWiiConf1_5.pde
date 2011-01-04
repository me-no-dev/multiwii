import processing.serial.*; // serial library
import controlP5.*; // controlP5 library

Serial g_serial;
ControlP5 controlP5; // create the handler to allow for controlP5 items
Textlabel txtlblWhichcom; // text label displaying which comm port is being used
ListBox commListbox; // list of available comm ports
Textlabel version;

cGraph g_graph;
int windowsX    = 800;
int windowsY    = 520;
int xGraph      = 10;
int yGraph      = 300;
int xObj        = 700;
int yObj        = 450;
int xParam      = 120;
int yParam      = 10;
int xRC         = 650;
int yRC         = 15;
int xMot        = 490;
int yMot        = 30;

int xNunchuk    = 485;
int yNunchuk    = 185;
int xLevel      = 485;
int yLevel      = 210;


boolean axGraph =true,ayGraph=true,azGraph=true,gxGraph=true,gyGraph=true,gzGraph=true,baroGraph=true;

int multiType;  // 1 for tricopter, 2 for quad+, 3 for quadX

cDataArray accPITCH   = new cDataArray(100);
cDataArray accROLL    = new cDataArray(100);
cDataArray accYAW     = new cDataArray(100);
cDataArray gyroPITCH  = new cDataArray(100);
cDataArray gyroROLL   = new cDataArray(100);
cDataArray gyroYAW    = new cDataArray(100);
cDataArray baroData   = new cDataArray(100);

Numberbox confP_ROLL;
Numberbox confP_PITCH;
Numberbox confP_YAW;
Numberbox confI_ROLL;
Numberbox confI_PITCH;
Numberbox confI_YAW;
Numberbox confD_ROLL;
Numberbox confD_PITCH;
Numberbox confD_YAW;
Numberbox confRC_RATE;
Numberbox confRC_EXPO;
Numberbox confACC_STRENGTH;

Numberbox rollPitchRate;
Numberbox yawRate;

Numberbox dynamic_THR_PID;

Slider rcStickThrottleSlider;
Slider rcStickRollSlider;
Slider rcStickPitchSlider;
Slider rcStickservoSliderH1;
Slider rcStickAUX1Slider;

Slider motSliderV1;
Slider motSliderV2;
Slider motSliderV0;
Slider motSliderV4;
Slider motSliderV5;
Slider motSliderV3;

Slider servoSliderH1;
Slider servoSliderH2;

Slider axSlider;
Slider aySlider;
Slider azSlider;
Slider gxSlider;
Slider gySlider;
Slider gzSlider;
Slider baroSlider;

Slider scaleSlider;

Button buttonREAD;
Button buttonWRITE;
Button buttonSTART;
Button buttonSTOP;

Button buttonNunchuk;
Button buttonLevel;

color yellow_ = color(200, 200, 20);
color green_ = color(30, 120, 30);
color red_ = color(120, 30, 30);
boolean graphEnable = false;
boolean readEnable = false;
boolean writeEnable = false;

float gx,gy,gz = 0;
float ax,ay,az = 0;
float baro = 0;
float angx,angy = 0;
float r;
int init_com = 0;
int graph_on = 0;

float mot0=1000,mot1=1000,mot2=1000,mot3=1000,mot4=1000,mot5=1000;
float servo1=1500,servo2=1500;
float rcThrottle = 1500,rcRoll = 1500,rcPitch = 1500,rcYaw =1500,rcAUX1=1500;
int nunchukPresent;
int levelMode;

float time1;
float time2;
int cycleTime;

void setup() {
  size(windowsX,windowsY);
  frameRate(20); 
  controlP5 = new ControlP5(this); // initialize the GUI controls
  controlP5.setControlFont(createFont("Arial bold",12));
  background(0);
 
  g_graph  = new cGraph(120,510, 480, 220);
  // make a listbox and populate it with the available comm ports
  commListbox = controlP5.addListBox("portComList",5,65,80,180); //addListBox(name,x,y,width,height)
  commListbox.captionLabel().toUpperCase(true);
  commListbox.captionLabel().set("PORT COM");
  commListbox.setColorBackground(red_);
  for(int i=0;i<Serial.list().length;i++) {
    commListbox.addItem("port: "+Serial.list()[i],i); // addItem(name,value)
  }

  // text label for which comm port selected
  txtlblWhichcom = controlP5.addTextlabel("txtlblWhichcom","No Port Selected",5,42); // textlabel(name,text,x,y)
    
  buttonSTART = controlP5.addButton("START",1,xGraph+110,yGraph-30,40,19); // buton(name,value,x,y,width,height)
  buttonSTART.setColorBackground(red_);
  
  buttonSTOP = controlP5.addButton("STOP",1,xGraph+160,yGraph-30,40,19); // buton(name,value,x,y,width,height)
  buttonSTOP.setColorBackground(red_);

  buttonNunchuk = controlP5.addButton("---",1,xNunchuk,yNunchuk,150,19); // buton(name,value,x,y,width,height)
  buttonNunchuk.setColorBackground(red_);

  buttonLevel = controlP5.addButton("----",1,xLevel,yLevel,150,19); // buton(name,value,x,y,width,height)
  buttonLevel.setColorBackground(red_);

  controlP5.addToggle("ACC_ROLL",true,xGraph-7,yGraph,20,15);
  controlP5.addToggle("ACC_PITCH",true,xGraph-7,yGraph+30,20,15);
  controlP5.addToggle("ACC_Z",true,xGraph-7,yGraph+60,20,15);
  controlP5.addToggle("GYRO_ROLL",true,xGraph-7,yGraph+90,20,15);
  controlP5.addToggle("GYRO_PITCH",true,xGraph-7,yGraph+120,20,15);
  controlP5.addToggle("GYRO_YAW",true,xGraph-7,yGraph+150,20,15);
  controlP5.addToggle("BARO",true,xGraph-7,yGraph+180,20,15);

  axSlider   = controlP5.addSlider("1",-400,+400,0,xGraph+60,yGraph+10,50,10);axSlider.setDecimalPrecision(0);
  aySlider   = controlP5.addSlider("2",-400,+400,0,xGraph+60,yGraph+40,50,10);aySlider.setDecimalPrecision(0);
  azSlider   = controlP5.addSlider("3",-400,+400,0,xGraph+60,yGraph+70,50,10);azSlider.setDecimalPrecision(0);
  gxSlider   = controlP5.addSlider("4",-500,+500,0,xGraph+60,yGraph+100,50,10);gxSlider.setDecimalPrecision(0);
  gySlider   = controlP5.addSlider("5",-500,+500,0,xGraph+60,yGraph+130,50,10);gySlider.setDecimalPrecision(0);
  gzSlider   = controlP5.addSlider("6",-500,+500,0,xGraph+60,yGraph+160,50,10);gzSlider.setDecimalPrecision(0);
  baroSlider = controlP5.addSlider("7",-500,+500,0,xGraph+60,yGraph+190,50,10);baroSlider.setDecimalPrecision(0);

  confP_ROLL = controlP5.addNumberbox("P_ROLL",0,xParam+5,yParam+5,40,14);
  confP_ROLL.setDecimalPrecision(1);
  confP_ROLL.setMultiplier(0.1);
  confP_ROLL.setDirection(Controller.HORIZONTAL);
  confP_ROLL.setMin(0);
  confP_ROLL.setMax(20);
  confP_ROLL.setColorBackground(red_);

  confI_ROLL = controlP5.addNumberbox("I_ROLL",0,xParam+65,yParam+5,40,14);
  confI_ROLL.setDecimalPrecision(3);
  confI_ROLL.setMultiplier(0.001);
  confI_ROLL.setDirection(Controller.HORIZONTAL);
  confI_ROLL.setMin(0);
  confI_ROLL.setMax(0.250);
  confI_ROLL.setColorBackground(red_);
  
  confD_ROLL = controlP5.addNumberbox("D_ROLL",0,xParam+125,yParam+5,40,14);
  confD_ROLL.setDecimalPrecision(1);
  confD_ROLL.setMultiplier(1);
  confD_ROLL.setDirection(Controller.HORIZONTAL);
  confD_ROLL.setMin(-40);
  confD_ROLL.setMax(0);
  confD_ROLL.setColorBackground(red_);

  confP_PITCH = controlP5.addNumberbox("P_PITCH",0,xParam+5,yParam+35,40,14);
  confP_PITCH.setDecimalPrecision(1);
  confP_PITCH.setMultiplier(0.1);
  confP_PITCH.setDirection(Controller.HORIZONTAL);
  confP_PITCH.setMin(0);
  confP_PITCH.setMax(20);
  confP_PITCH.setColorBackground(red_);

  confI_PITCH = controlP5.addNumberbox("I_PITCH",0,xParam+65,yParam+35,40,14);
  confI_PITCH.setDecimalPrecision(3);
  confI_PITCH.setMultiplier(0.001);
  confI_PITCH.setDirection(Controller.HORIZONTAL);
  confI_PITCH.setMin(0);
  confI_PITCH.setMax(0.250);
  confI_PITCH.setColorBackground(red_);

  confD_PITCH = controlP5.addNumberbox("D_PITCH",0,xParam+125,yParam+35,40,14);
  confD_PITCH.setDecimalPrecision(1);
  confD_PITCH.setMultiplier(1);
  confD_PITCH.setDirection(Controller.HORIZONTAL);
  confD_PITCH.setMin(-40);
  confD_PITCH.setMax(0);
  confD_PITCH.setColorBackground(red_);
  
  confP_YAW = controlP5.addNumberbox("P_YAW",0,xParam+5,yParam+65,40,14);
  confP_YAW.setDecimalPrecision(1);
  confP_YAW.setMultiplier(0.1);
  confP_YAW.setDirection(Controller.HORIZONTAL);
  confP_YAW.setMin(0);
  confP_YAW.setMax(20);
  confP_YAW.setColorBackground(red_);

  confI_YAW = controlP5.addNumberbox("I_YAW",0,xParam+65,yParam+65,40,14);
  confI_YAW.setDecimalPrecision(3);
  confI_YAW.setMultiplier(0.001);
  confI_YAW.setDirection(Controller.HORIZONTAL);
  confI_YAW.setMin(0);
  confI_YAW.setMax(0.250);
  confI_YAW.setColorBackground(red_);

  confD_YAW = controlP5.addNumberbox("D_YAW",0,xParam+125,yParam+65,40,14);
  confD_YAW.setDecimalPrecision(1);
  confD_YAW.setMultiplier(1);
  confD_YAW.setDirection(Controller.HORIZONTAL);
  confD_YAW.setMin(-40);
  confD_YAW.setMax(0);
  confD_YAW.setColorBackground(red_);

  rollPitchRate = controlP5.addNumberbox("P1",0,xParam+220,yParam+15,30,14);
  rollPitchRate.setDecimalPrecision(2);
  rollPitchRate.setMultiplier(0.01);
  rollPitchRate.setDirection(Controller.HORIZONTAL);
  rollPitchRate.setMin(0);
  rollPitchRate.setMax(1);
  rollPitchRate.setColorBackground(red_);
  rollPitchRate.setCaptionLabel("PITCH/ROLL rate");

  yawRate = controlP5.addNumberbox("P2",0,xParam+220,yParam+50,30,14);
  yawRate.setDecimalPrecision(2);
  yawRate.setMultiplier(0.01);
  yawRate.setDirection(Controller.HORIZONTAL);
  yawRate.setMin(0);
  yawRate.setMax(1);
  yawRate.setColorBackground(red_);
  yawRate.setCaptionLabel("YAW rate");

  dynamic_THR_PID = controlP5.addNumberbox("PID1",0,xParam+220,yParam+100,30,14);
  dynamic_THR_PID.setDecimalPrecision(2);
  dynamic_THR_PID.setMultiplier(0.01);
  dynamic_THR_PID.setDirection(Controller.HORIZONTAL);
  dynamic_THR_PID.setMin(0);
  dynamic_THR_PID.setMax(1);
  dynamic_THR_PID.setColorBackground(red_);
  dynamic_THR_PID.setCaptionLabel("Throttle rate PID");

  confRC_RATE = controlP5.addNumberbox("RC RATE",1,xParam+5,yParam+105,40,14);
  confRC_RATE.setDecimalPrecision(2);
  confRC_RATE.setMultiplier(0.01);
  confRC_RATE.setDirection(Controller.HORIZONTAL);
  confRC_RATE.setMin(0);
  confRC_RATE.setMax(1);
  confRC_RATE.setColorBackground(red_);

  confRC_EXPO = controlP5.addNumberbox("RC EXPO",0,xParam+5,yParam+135,40,14);
  confRC_EXPO.setDecimalPrecision(2);
  confRC_EXPO.setMultiplier(0.01);
  confRC_EXPO.setDirection(Controller.HORIZONTAL);
  confRC_EXPO.setMin(0);
  confRC_EXPO.setMax(1);
  confRC_EXPO.setColorBackground(red_);

  confACC_STRENGTH = controlP5.addNumberbox("AUTOLEVEL STRENGTH",10,xParam+5,yParam+175,40,14);
  confACC_STRENGTH.setDecimalPrecision(1);
  confACC_STRENGTH.setMultiplier(0.1);
  confACC_STRENGTH.setDirection(Controller.HORIZONTAL);
  confACC_STRENGTH.setMin(0);
  confACC_STRENGTH.setMax(25);
  confACC_STRENGTH.setColorBackground(red_);

  buttonREAD = controlP5.addButton("READ",1,xParam+5,yParam+225,60,16); // buton(name,value,x,y,width,height)
  buttonREAD.setColorBackground(red_);
  
  buttonWRITE = controlP5.addButton("WRITE",1,xParam+290,yParam+225,60,16);
  buttonWRITE.setColorBackground(red_);

  rcStickThrottleSlider = controlP5.addSlider("Throttle",900,2100,1500,xRC,yRC,10,100);
  rcStickThrottleSlider.setDecimalPrecision(0);

  rcStickPitchSlider = controlP5.addSlider("Pitch",900,2100,1500,xRC+80,yRC,10,100);
  rcStickPitchSlider.setDecimalPrecision(0);
  rcStickRollSlider = controlP5.addSlider("Roll",900,2100,1500,xRC,yRC+125,100,10);
  rcStickRollSlider.setDecimalPrecision(0);
  rcStickservoSliderH1 = controlP5.addSlider("Yaw",900,2100,1500,xRC,yRC+150,100,10);
  rcStickservoSliderH1.setDecimalPrecision(0);
  rcStickAUX1Slider = controlP5.addSlider("AUX1",900,2100,1500,xRC,yRC+175,100,10);
  rcStickAUX1Slider.setDecimalPrecision(0);


  motSliderV0  = controlP5.addSlider("motSliderV0",1000,2000,1500,xMot+50,yMot+15,10,100);
  motSliderV0.setDecimalPrecision(0);
  motSliderV0.hide();
  
  motSliderV1  = controlP5.addSlider("motSliderV1",1000,2000,1500,xMot+100,yMot-15,10,100);
  motSliderV1.setDecimalPrecision(0);
  motSliderV1.hide();
  
  motSliderV2  = controlP5.addSlider("motSliderV2",1000,2000,1500,xMot,yMot-15,10,100);
  motSliderV2.setDecimalPrecision(0);
  motSliderV2.hide();

  motSliderV3  = controlP5.addSlider("motSliderV3",1000,2000,1500,xMot+50,yMot+15,10,100);
  motSliderV3.setDecimalPrecision(0);
  motSliderV3.hide();
  
  motSliderV4  = controlP5.addSlider("motSliderV4",1000,2000,1500,xMot+100,yMot-15,10,100);
  motSliderV4.setDecimalPrecision(0);
  motSliderV4.hide();
  
  motSliderV5  = controlP5.addSlider("motSliderV5",1000,2000,1500,xMot,yMot-15,10,100);
  motSliderV5.setDecimalPrecision(0);
  motSliderV5.hide();


  servoSliderH1  = controlP5.addSlider("Servo1",1000,2000,1500,xMot,yMot+135,100,10);
  servoSliderH1.setDecimalPrecision(0);
  servoSliderH1.hide();

  servoSliderH2 = controlP5.addSlider("Servo2",1000,2000,1500,xMot,yMot-15,100,10);
  servoSliderH2.setDecimalPrecision(0);
  servoSliderH2.hide();

  scaleSlider = controlP5.addSlider("SCALE",0,10,1,xGraph+400,yGraph-30,150,20);

}

void draw() {
  int i;
  float val;
  float inter;
  float a;
  float b;
  float sina;
  float sinb;
  float cosa;
  float cosb;

  background(80);
  
  textFont(createFont("Arial bold",12), 15);
  text("MultiWii conf", 0, 16);
  text("v1.5", 0, 32);

  textFont(createFont("Arial bold",12), 15); 
  text("Cycle Time:", 230, 285);
  text(cycleTime, 330, 285);

  time1=millis();
  if (init_com==1) {
    if  (g_serial.available() >70) g_serial.clear();
    while (g_serial.available() >59)
      processSerialData();
    if ((time1-time2)>100 && graph_on==1) {
      g_serial.write('A');
      time2=time1;
    }
  }
  controlP5.draw();
  
  axSlider.setValue(ax);
  aySlider.setValue(ay);
  azSlider.setValue(az);
  gxSlider.setValue(gx);
  gySlider.setValue(gy);
  gzSlider.setValue(gz);
  baroSlider.setValue(baro);

  motSliderV0.setValue(mot0);
  motSliderV1.setValue(mot1);
  motSliderV2.setValue(mot2);
  motSliderV3.setValue(mot3);
  motSliderV4.setValue(mot4);
  motSliderV5.setValue(mot5);

  servoSliderH1.setValue(servo1);
  servoSliderH2.setValue(servo2);
  
  rcStickThrottleSlider.setValue(rcThrottle);
  rcStickRollSlider.setValue(rcRoll);
  rcStickPitchSlider.setValue(rcPitch);
  rcStickservoSliderH1.setValue(rcYaw);
  rcStickAUX1Slider.setValue(rcAUX1);


  stroke(255); 
  a=angx*PI/180;
  b=angy*PI/180;
  sina = sin(a);
  sinb = sin(b);
  cosa = cos(a);
  cosb = cos(b);

  float size = 30.0;

  if (multiType == 1) { //TRI
    ellipse(xObj-size*cosa, yObj-size*cosb, size*cosa *(1+sina/3) *(1-sinb/3) , size*cosb *(1+sina/3) *(1-sinb/3) );
    ellipse(xObj+size*cosa, yObj-size*cosb, size*cosa *(1-sina/3) *(1-sinb/3) , size*cosb *(1-sina/3) *(1-sinb/3) );
    ellipse(xObj,  yObj+size*cosb,          size*cosa *(1+sinb/3), size*cosb *(1+sinb/3));
    line(xObj-size*cosa,yObj-size*cosb, xObj,yObj);
    line(xObj+size*cosa,yObj-size*cosb, xObj,yObj);  
    line(xObj,yObj+size*cosb, xObj,yObj);
    textFont(createFont("Arial bold",12), 15);
    text("TRICOPTER", xObj-200, yObj-190);
  
    motSliderV0.setPosition(xMot+50,yMot+15);
    motSliderV0.setHeight(100);
    motSliderV0.setCaptionLabel("REAR");
    motSliderV0.show();

    motSliderV1.setPosition(xMot+100,yMot-15);
    motSliderV1.setHeight(100);
    motSliderV1.setCaptionLabel("RIGHT");
    motSliderV1.show();

    motSliderV2.setPosition(xMot,yMot-15);
    motSliderV2.setHeight(100);
    motSliderV2.setCaptionLabel("LEFT");
    motSliderV2.show();

    servoSliderH1.setPosition(xMot,yMot+135);
    servoSliderH1.setCaptionLabel("SERVO");
    servoSliderH1.show(); 

    motSliderV3.hide();
    motSliderV4.hide();
    motSliderV5.hide();
    servoSliderH2.hide();

  } else if (multiType == 2) { //QUAD+
    ellipse(xObj,  yObj-size*cosb,   size*cosa *(1-sinb/3), size*cosb *(1-sinb/3));
    ellipse(xObj,  yObj+size*cosb,   size*cosa *(1+sinb/3), size*cosb *(1+sinb/3));
    ellipse(xObj+size*cosa, yObj,    size*cosa *(1-sina/3) , size*cosb *(1-sina/3) );
    ellipse(xObj-size*cosa, yObj,    size*cosa *(1+sina/3) , size*cosb *(1+sina/3) );
    line(xObj-size*cosa,yObj, xObj,yObj);
    line(xObj+size*cosa,yObj, xObj,yObj);  
    line(xObj,yObj+size*cosb, xObj,yObj);  
    line(xObj,yObj-size*cosb, xObj,yObj);
    textFont(createFont("Arial bold",12), 15);
    text("QUADRICOPTER +", xObj-220, yObj-190);
      
    motSliderV0.setPosition(xMot+50,yMot+75);
    motSliderV0.setHeight(60);
    motSliderV0.setCaptionLabel("REAR");
    motSliderV0.show();

    motSliderV1.setPosition(xMot+100,yMot+35);
    motSliderV1.setHeight(60);
    motSliderV1.setCaptionLabel("RIGHT");
    motSliderV1.show();

    motSliderV2.setPosition(xMot,yMot+35);
    motSliderV2.setHeight(60);
    motSliderV2.setCaptionLabel("LEFT");
    motSliderV2.show();

    motSliderV3.setPosition(xMot+50,yMot-15);
    motSliderV3.setHeight(60);
    motSliderV3.setCaptionLabel("FRONT");
    motSliderV3.show();
    
    motSliderV4.hide();
    motSliderV5.hide();
    servoSliderH1.hide();
    servoSliderH2.hide();  
  } else if (multiType == 3) { //QUAD X
    ellipse(xObj-size*cosa,  yObj-size*cosb, size*cosa *(1+sina/3)*(1-sinb/3), size*cosb *(1+sina/3)*(1-sinb/3));
    ellipse(xObj+size*cosa,  yObj-size*cosb, size*cosa *(1-sina/3)*(1-sinb/3), size*cosb *(1-sina/3)*(1-sinb/3));
    ellipse(xObj-size*cosa,  yObj+size*cosb, size*cosa *(1+sina/3)*(1+sinb/3), size*cosb *(1+sina/3)*(1+sinb/3));
    ellipse(xObj+size*cosa,  yObj+size*cosb, size*cosa *(1-sina/3)*(1+sinb/3), size*cosb *(1-sina/3)*(1+sinb/3));
    line(xObj-size*cosa,yObj-size*cosb, xObj,yObj);
    line(xObj+size*cosa,yObj-size*cosb, xObj,yObj);
    line(xObj-size*cosa,yObj+size*cosb, xObj,yObj);
    line(xObj+size*cosa,yObj+size*cosb, xObj,yObj);
    textFont(createFont("Arial bold",12), 15);
    text("QUADRICOPTER X", xObj-220, yObj-190);
    
    motSliderV0.setPosition(xMot+90,yMot+75);
    motSliderV0.setHeight(60);
    motSliderV0.setCaptionLabel("REAR_R");
    motSliderV0.show();

    motSliderV1.setPosition(xMot+90,yMot-15);
    motSliderV1.setHeight(60);
    motSliderV1.setCaptionLabel("FRONT_R");
    motSliderV1.show();

    motSliderV2.setPosition(xMot+10,yMot+75);
    motSliderV2.setHeight(60);
    motSliderV2.setCaptionLabel("REAR_L");
    motSliderV2.show();

    motSliderV3.setPosition(xMot+10,yMot-15);
    motSliderV3.setHeight(60);
    motSliderV3.setCaptionLabel("FRONT_L");
    motSliderV3.show(); 
    
    motSliderV4.hide();
    motSliderV5.hide();
    servoSliderH1.hide();
    servoSliderH2.hide();

  } else if (multiType == 4) { //BI
    ellipse(xObj,  yObj-size*cosb,   size*cosa *(1-sinb/3), size*cosb *(1-sinb/3));
    ellipse(xObj,  yObj+size*cosb,   size*cosa *(1+sinb/3), size*cosb *(1+sinb/3));
    line(xObj,yObj+size*cosb, xObj,yObj);  
    line(xObj,yObj-size*cosb, xObj,yObj);
    textFont(createFont("Arial bold",12), 15);
    text("BICOPTER", xObj-200, yObj-190);
   
    motSliderV0.setPosition(xMot+50,yMot+73);
    motSliderV0.setHeight(55);
    motSliderV0.setCaptionLabel("");
    motSliderV0.show();

    motSliderV1.setPosition(xMot+50,yMot+2);
    motSliderV1.setHeight(55);
    motSliderV1.setCaptionLabel("MOT");
    motSliderV1.show();

    servoSliderH2.setPosition(xMot,yMot+135);
    servoSliderH2.setCaptionLabel("SERVO");
    servoSliderH2.show();
    
    servoSliderH1.setPosition(xMot,yMot-15);
    servoSliderH1.setCaptionLabel("SERVO");
    servoSliderH1.show();

    motSliderV2.hide();
    motSliderV3.hide();
    motSliderV4.hide();
    motSliderV5.hide();

  } else if (multiType == 5) { //GIMBAL
    textFont(createFont("Arial bold",12), 15);
    text("GIMBLE", xObj-200, yObj-190);

    textFont(createFont("Arial bold",12), 15);
    text("GIMBLE", xMot,yMot+25);
 
    servoSliderH2.setPosition(xMot,yMot+75);
    servoSliderH2.setCaptionLabel("ROLL");
    servoSliderH2.show();

    servoSliderH1.setPosition(xMot,yMot+35);
    servoSliderH1.setCaptionLabel("PITCH");
    servoSliderH1.show();

    motSliderV0.hide();
    motSliderV1.hide();
    motSliderV2.hide();
    motSliderV3.hide();
    motSliderV4.hide();
    motSliderV5.hide();
  } else if (multiType == 6) { //Y6
    //stroke(100, 255, 255);
    fill(150,255,255);
    ellipse(xObj-size*cosa, yObj-size*cosb, size*cosa *(1+sina/3) *(1-sinb/3) , size*cosb *(1+sina/3) *(1-sinb/3) );
    ellipse(xObj+size*cosa, yObj-size*cosb, size*cosa *(1-sina/3) *(1-sinb/3) , size*cosb *(1-sina/3) *(1-sinb/3) );
    ellipse(xObj,  -2+yObj+size*cosb,          size*cosa *(1+sinb/3), size*cosb *(1+sinb/3));
    fill(200, 150, 150);
    ellipse(-5+xObj-size*cosa, -5+yObj-size*cosb, size*cosa *(1+sina/3) *(1-sinb/3) , size*cosb *(1+sina/3) *(1-sinb/3) );
    ellipse(5+xObj+size*cosa, -5+yObj-size*cosb, size*cosa *(1-sina/3) *(1-sinb/3) , size*cosb *(1-sina/3) *(1-sinb/3) );
    ellipse(xObj, 3+yObj+size*cosb,          size*cosa *(1+sinb/3), size*cosb *(1+sinb/3));
    stroke(255, 255, 255);
    fill(255, 255, 255);
    line(xObj-size*cosa,yObj-size*cosb, xObj,yObj);
    line(xObj+size*cosa,yObj-size*cosb, xObj,yObj);  
    line(xObj,yObj+size*cosb, xObj,yObj);
    textFont(createFont("Arial bold",12), 15);
    text("TRICOPTER Y6", xObj-200, yObj-190);

    motSliderV0.setPosition(xMot+50,yMot+23);
    motSliderV0.setHeight(50);
    motSliderV0.setCaptionLabel("REAR");
    motSliderV0.show();
    
    motSliderV1.setPosition(xMot+100,yMot-18);
    motSliderV1.setHeight(50);
    motSliderV1.setCaptionLabel("RIGHT");
    motSliderV1.show();

    motSliderV2.setPosition(xMot,yMot-18);
    motSliderV2.setHeight(50);
    motSliderV2.setCaptionLabel("LEFT");
    motSliderV2.show();

    motSliderV3.setPosition(xMot+50,yMot+87);
    motSliderV3.setHeight(50);
    motSliderV3.setCaptionLabel("U_REAR");
    motSliderV3.show();

    motSliderV4.setPosition(xMot+100,yMot+48);
    motSliderV4.setHeight(50);
    motSliderV4.setCaptionLabel("U_RIGHT");
    motSliderV4.show();

    motSliderV5.setPosition(xMot,yMot+48);
    motSliderV5.setHeight(50);
    motSliderV5.setCaptionLabel("U_LEFT");
    motSliderV5.show();

    servoSliderH1.hide();
    servoSliderH2.hide();
  } else if (multiType == 7) { //HEX6
    ellipse(xObj-size*cosa,  yObj-0.55*size*cosb, size*cosa *(1+sina/3)*(1-sinb/3), size*cosb *(1+sina/3)*(1-sinb/3));
    ellipse(xObj+size*cosa,  yObj-0.55*size*cosb, size*cosa *(1-sina/3)*(1-sinb/3), size*cosb *(1-sina/3)*(1-sinb/3));
    ellipse(xObj-size*cosa,  yObj+0.55*size*cosb, size*cosa *(1+sina/3)*(1+sinb/3), size*cosb *(1+sina/3)*(1+sinb/3));
    ellipse(xObj+size*cosa,  yObj+0.55*size*cosb, size*cosa *(1-sina/3)*(1+sinb/3), size*cosb *(1-sina/3)*(1+sinb/3));
    ellipse(xObj,  yObj-size*cosb,   size*cosa *(1-sinb/3), size*cosb *(1-sinb/3));
    ellipse(xObj,  yObj+size*cosb,   size*cosa *(1+sinb/3), size*cosb *(1+sinb/3));
    
    line(xObj-size*cosa,yObj-0.55*size*cosb, xObj,yObj);
    line(xObj+size*cosa,yObj-0.55*size*cosb, xObj,yObj);
    line(xObj-size*cosa,yObj+0.55*size*cosb, xObj,yObj);
    line(xObj+size*cosa,yObj+0.55*size*cosb, xObj,yObj);
    line(xObj,yObj+size*cosb, xObj,yObj);  
    line(xObj,yObj-size*cosb, xObj,yObj);
    
    textFont(createFont("Arial bold",12), 15);
    text("HEXACOPTER", xObj-220, yObj-190);


    motSliderV0.setPosition(xMot+90,yMot+65);
    motSliderV0.setHeight(50);
    motSliderV0.setCaptionLabel("REAR_R");
    motSliderV0.show();

    motSliderV1.setPosition(xMot+90,yMot-5);
    motSliderV1.setHeight(50);
    motSliderV1.setCaptionLabel("FRONT_R");
    motSliderV1.show();


    motSliderV2.setPosition(xMot+5,yMot+65);
    motSliderV2.setHeight(50);
    motSliderV2.setCaptionLabel("REAR_L");
    motSliderV2.show();


    motSliderV3.setPosition(xMot+5,yMot-5);
    motSliderV3.setHeight(50);
    motSliderV3.setCaptionLabel("FRONT_L");
    motSliderV3.show(); 

    motSliderV4.setPosition(xMot+50,yMot-20);
    motSliderV4.setHeight(50);
    motSliderV4.setCaptionLabel("FRONT");
    motSliderV4.show(); 

    motSliderV5.setPosition(xMot+50,yMot+90);
    motSliderV5.setHeight(50);
    motSliderV5.setCaptionLabel("REAR");
    motSliderV5.show(); 


    servoSliderH1.hide();
    servoSliderH2.hide();
  }


  size = 30.0;

//  if (nunchukPresent==1) {
    line(xObj+30+size*cosa,yObj-165+size*sina, xObj+30-size*cosa,yObj-165-size*sina);
    line(xObj+30,yObj-165,xObj+30+10*sina,yObj-165-10*cosa);
    line(xObj+30+size*cosb,yObj-100+size*sinb, xObj+30-size*cosb,yObj-100-size*sinb); 
  
    line(xObj+30+size*cosb,yObj-100+size*sinb, xObj+30+(size-10)*cos(b+PI/10) ,yObj-100+(size-10)*sin(b+PI/10));  
    line(xObj+30+size*cosb,yObj-100+size*sinb, xObj+30+(size-10)*cos(b-PI/10) ,yObj-100+(size-10)*sin(b-PI/10));  
  
    textFont(createFont("Arial bold",12), 15);
    text("ROLL", xObj-60, yObj-160);
    text("PITCH", xObj-60, yObj-95);
//  }

  strokeWeight(1);
  fill(255, 255, 255);
  g_graph.drawGraphBox();
  
  strokeWeight(1.5);
  stroke(255, 0, 0);
  if (axGraph) g_graph.drawLine(accROLL, -500, +500);
  stroke(0, 255, 0);
  if (ayGraph) g_graph.drawLine(accPITCH, -500, +500);
  stroke(0, 0, 255);
  if (azGraph) {
   if (scaleSlider.value()<2) {
     g_graph.drawLine(accYAW, -500, +500);
   } else {
     g_graph.drawLine(accYAW, 200*scaleSlider.value()-500,200*scaleSlider.value()+500);
   }
  }
  stroke(200, 200, 0);
  if (gxGraph) g_graph.drawLine(gyroROLL, -300, +300);
  stroke(0, 255, 255);
  if (gyGraph) g_graph.drawLine(gyroPITCH, -300, +300);
  stroke(255, 0, 255);
  if (gzGraph) g_graph.drawLine(gyroYAW, -300, +300);
  stroke(125, 125, 125);
  if (baroGraph) g_graph.drawLine(baroData, -300, +300);
  
  strokeWeight(2);
  stroke(255, 0, 0);     line(xGraph+25, yGraph+10, xGraph+60, yGraph+10);
  stroke(0, 255, 0);     line(xGraph+25, yGraph+40, xGraph+60, yGraph+40);
  stroke(0, 0, 255);     line(xGraph+25, yGraph+70, xGraph+60, yGraph+70);
  stroke(200, 200, 0);   line(xGraph+25, yGraph+100, xGraph+60, yGraph+100);
  stroke(0, 255, 255);   line(xGraph+25, yGraph+130, xGraph+60, yGraph+130);
  stroke(255, 0, 255);   line(xGraph+25, yGraph+160, xGraph+60, yGraph+160);
  stroke(125, 125, 125); line(xGraph+25, yGraph+190, xGraph+60, yGraph+190);
  fill(0, 0, 0);

  strokeWeight(3);
  stroke(0, 0, 0);
  rectMode(CORNERS);
  rect(xMot-5,yMot-20, xMot+145, yMot+150);
  rect(xRC-5,yRC-5, xRC+185, yRC+195);
  rect(xParam,yParam, xParam+355, yParam+245);

  int xSens       = xParam + 85;
  int ySens       = yParam + 110;
  stroke(255);
  a=confRC_RATE.value();
  b=confRC_EXPO.value();
  strokeWeight(1);
  line(xSens,ySens,xSens,ySens+50);
  line(xSens,ySens+50,xSens+70,ySens+50);
  strokeWeight(3);
  stroke(30,120,30);
  for(i=0;i<70;i++) {
    inter = 10*i;
    val = a*inter*(1-b+inter*inter*b/490000);
    point(xSens+i,ySens+(70-val/10)*5/7);
  }
}

void ACC_ROLL(boolean theFlag) {axGraph = theFlag;}
void ACC_PITCH(boolean theFlag) {ayGraph = theFlag;}
void ACC_Z(boolean theFlag) {azGraph = theFlag;}
void GYRO_ROLL(boolean theFlag) {gxGraph = theFlag;}
void GYRO_PITCH(boolean theFlag) {gyGraph = theFlag;}
void GYRO_YAW(boolean theFlag) {gzGraph = theFlag;}
void BARO(boolean theFlag) {baroGraph = theFlag;}

// print the name of the control being triggered (for debugging) and see if it was a Listbox event
public void controlEvent(ControlEvent theEvent) {
  // ListBox is if type ControlGroup, you need to check the Event with if (theEvent.isGroup())to avoid an error message from controlP5
  if (theEvent.isGroup()) {
    // an event from a group
    if (theEvent.name()=="portComList") {
      InitSerial(theEvent.group().value()); // initialize the serial port selected
      println("got portComList"+"   value = "+theEvent.group().value()); // for debugging
    }
  }
  else {
    //println(theEvent.controller().name()); // for debugging
  }
}

public void START(int theValue) {
  if(graphEnable == false) {return;}
  graph_on=1;
  readEnable = true;
  buttonREAD.setColorBackground(green_);
}

public void STOP(int theValue) {
  graph_on=0;
}

int byteP_ROLL,byteI_ROLL,byteD_ROLL,
     byteP_PITCH,byteI_PITCH,byteD_PITCH,
     byteP_YAW,byteI_YAW,byteD_YAW,
     byteRC_RATE,byteRC_EXPO,byteACC_STRENGTH,
     byteRollPitchRate,byteYawRate,
     byteDynThrPID;


public void READ(int theValue) {
  if(readEnable == false) {return;}
  confP_ROLL.setValue(byteP_ROLL/10.0);
  confI_ROLL.setValue(byteI_ROLL/1000.0);
  confD_ROLL.setValue(-byteD_ROLL);
  confP_PITCH.setValue(byteP_PITCH/10.0);
  confI_PITCH.setValue(byteI_PITCH/1000.0);
  confD_PITCH.setValue(-byteD_PITCH);
  confP_YAW.setValue(byteP_YAW/10.0);
  confI_YAW.setValue(byteI_YAW/1000.0);
  confD_YAW.setValue(-byteD_YAW);
  confRC_RATE.setValue(byteRC_RATE/100.0);
  confRC_EXPO.setValue(byteRC_EXPO/100.0);
  confACC_STRENGTH.setValue(byteACC_STRENGTH/10.0);
  rollPitchRate.setValue(byteRollPitchRate/100.0);
  yawRate.setValue(byteYawRate/100.0);

  dynamic_THR_PID.setValue(byteDynThrPID/100.0);


  buttonWRITE.setColorBackground(green_);
  confP_ROLL.setColorBackground(green_);
  confI_ROLL.setColorBackground(green_);
  confD_ROLL.setColorBackground(green_);
  confP_PITCH.setColorBackground(green_);
  confI_PITCH.setColorBackground(green_);
  confD_PITCH.setColorBackground(green_);
  confP_YAW.setColorBackground(green_);
  confI_YAW.setColorBackground(green_);
  confD_YAW.setColorBackground(green_);
  confRC_RATE.setColorBackground(green_);
  confRC_EXPO.setColorBackground(green_);
  confACC_STRENGTH.setColorBackground(green_);
  rollPitchRate.setColorBackground(green_);
  yawRate.setColorBackground(green_);
  dynamic_THR_PID.setColorBackground(green_);
  
  writeEnable = true;
}

public void WRITE(int theValue) {
  if(writeEnable == false) {return;}

  byteP_ROLL = (round(confP_ROLL.value()*10));
  byteI_ROLL = (round(confI_ROLL.value()*1000));
  byteD_ROLL = (round(-confD_ROLL.value()));
  byteP_PITCH = (round(confP_PITCH.value()*10));
  byteI_PITCH = (round(confI_PITCH.value()*1000));
  byteD_PITCH = (round(-confD_PITCH.value()));
  byteP_YAW = (round(confP_YAW.value()*10));
  byteI_YAW = (round(confI_YAW.value()*1000));
  byteD_YAW = (round(-confD_YAW.value()));
  byteRC_RATE = (round(confRC_RATE.value()*100));
  byteRC_EXPO = (round(confRC_EXPO.value()*100));
  byteACC_STRENGTH = (round(confACC_STRENGTH.value()*10));
  byteRollPitchRate = (round(rollPitchRate.value()*100));
  byteYawRate = (round(yawRate.value()*100));
  byteDynThrPID = (round(dynamic_THR_PID.value()*100));

  int[] s = new int[32];
   s[0] = 'C';
   s[1] =  byteP_ROLL;
   s[2] =  byteI_ROLL;
   s[3] =  byteD_ROLL;
   s[4] =  byteP_PITCH;
   s[5] =  byteI_PITCH;
   s[6] =  byteD_PITCH;
   s[7] =  byteP_YAW;
   s[8] =  byteI_YAW;
   s[9] =  byteD_YAW;
   s[10] = byteRC_RATE;
   s[11] = byteRC_EXPO;
   s[12] = byteACC_STRENGTH;
   s[13] = byteRollPitchRate;
   s[14] = byteYawRate;
   s[15] = byteDynThrPID;
   for(int i =0;i<16;i++)    g_serial.write(char(s[i]));
}


// initialize the serial port selected in the listBox
void InitSerial(float portValue) {
  println("initializing serial " + int(portValue) + " in serial.list()"); // for debugging
  String portPos = Serial.list()[int(portValue)]; // grab the name of the serial port
  txtlblWhichcom.setValue("COM = " + portPos);
  g_serial = new Serial(this, portPos, 115200); // initialize the port
  // read bytes into a buffer until you get a linefeed (ASCII 10):
  g_serial.bufferUntil('\n');
  println("done init serial");
  init_com=1;
  buttonSTART.setColorBackground(green_);
  buttonSTOP.setColorBackground(green_);
  commListbox.setColorBackground(green_);
  graphEnable = true;
}

int p;
byte[] inBuf = new byte[128];

int read16() {
  return (inBuf[p++]&0xff) + (inBuf[p++]<<8);
}
int read8() {
  return inBuf[p++]&0xff;
}


void processSerialData() {

  if (g_serial.read() == 'A') {

    while (g_serial.available() <65) {}
    g_serial.readBytes(inBuf);
    if (inBuf[64] == 'A') {
      p=0;
      ax = read16();
      ay = read16();
      az = read16();
      gx = read16();
      gy = read16();
      gz = read16();
      baro = read16();
      servo1 = read16();
      servo2 = read16();
      mot0 = read16();
      mot1 = read16();
      mot2 = read16();
      mot3 = read16();
      mot4 = read16();
      mot5 = read16();
      rcRoll = read16();
      rcPitch = read16();
      rcYaw = read16();
      rcThrottle = read16();
      rcAUX1 = read16();
      nunchukPresent = read8();
      levelMode = read8();
      cycleTime = read16();
      angx = read16();
      angy = read16();
      multiType = read8();
      byteP_ROLL = read8();
      byteI_ROLL = read8();
      byteD_ROLL = read8();
      byteP_PITCH = read8();
      byteI_PITCH = read8();
      byteD_PITCH = read8();
      byteP_YAW = read8();
      byteI_YAW = read8();
      byteD_YAW = read8();
      byteRC_RATE = read8();
      byteRC_EXPO = read8();
      byteACC_STRENGTH = read8();
      byteRollPitchRate = read8();
      byteYawRate = read8();
      byteDynThrPID = read8();
    }

    if (levelMode>0) {
      buttonLevel.setCaptionLabel("AutoLevel Enable");
      buttonLevel.setColorBackground(green_);
    } else {
      buttonLevel.setCaptionLabel("AutoLevel Disable");
      buttonLevel.setColorBackground(red_);
    }
    if (nunchukPresent>0) {
      buttonNunchuk.setCaptionLabel("Nunchuk Connected");
      buttonNunchuk.setColorBackground(green_);
    } else {
      buttonNunchuk.setCaptionLabel("Nunchuk Absent");
      buttonNunchuk.setColorBackground(red_);
    }

    accROLL.addVal(ax);
    accPITCH.addVal(ay);
    accYAW.addVal(az);
    gyroROLL.addVal(gx);
    gyroPITCH.addVal(gy);
    gyroYAW.addVal(gz);
    baroData.addVal(baro);
  }
}


//********************************************************
//********************************************************
//********************************************************

class cDataArray
{
  float[] m_data;
  int m_maxSize;
  int m_startIndex = 0;
  int m_endIndex = 0;
  int m_curSize;
  
  cDataArray(int maxSize){
    m_maxSize = maxSize;
    m_data = new float[maxSize];
  }
  void addVal(float val) {
    m_data[m_endIndex] = val;
    m_endIndex = (m_endIndex+1)%m_maxSize;
    if (m_curSize == m_maxSize) {
      m_startIndex = (m_startIndex+1)%m_maxSize;
    } else {
      m_curSize++;
    }
  }
  float getVal(int index) {return m_data[(m_startIndex+index)%m_maxSize];}
  int getCurSize(){return m_curSize;}
  int getMaxSize() {return m_maxSize;}
}

// This class takes the data and helps graph it
class cGraph {
  float m_gWidth, m_gHeight;
  float m_gLeft, m_gBottom, m_gRight, m_gTop;
  
  cGraph(float x, float y, float w, float h) {
    m_gWidth     = w;
    m_gHeight    = h;
    m_gLeft      = x;
    m_gBottom    = y;
    m_gRight     = x + w;
    m_gTop       = y - h;
  }
  
  void drawGraphBox() {
    stroke(0, 0, 0);
    rectMode(CORNERS);
    rect(m_gLeft, m_gBottom, m_gRight, m_gTop);
  }
  
  void drawLine(cDataArray data, float minRange, float maxRange) {
    float graphMultX = m_gWidth/data.getMaxSize();
    float graphMultY = m_gHeight/(maxRange-minRange);
    
    for(int i=0; i<data.getCurSize()-1; ++i) {
      float x0 = i*graphMultX+m_gLeft;
      float y0 = m_gBottom-((data.getVal(i)*scaleSlider.value()-minRange)*graphMultY);
      float x1 = (i+1)*graphMultX+m_gLeft;
      float y1 = m_gBottom-((data.getVal(i+1)*scaleSlider.value()-minRange)*graphMultY);
      line(x0, y0, x1, y1);
    }
  }
} 

