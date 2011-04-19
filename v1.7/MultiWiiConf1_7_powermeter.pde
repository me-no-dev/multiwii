import processing.serial.*; // serial library
import controlP5.*; // controlP5 library
import processing.opengl.*;

Serial g_serial;
ControlP5 controlP5;
Textlabel txtlblWhichcom,version; 
ListBox commListbox;

cGraph g_graph;
int windowsX    = 800;
int windowsY    = 540;
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

int xButton    = 485;
int yButton    = 185;

boolean axGraph =true,ayGraph=true,azGraph=true,gxGraph=true,gyGraph=true,gzGraph=true,baroGraph=true,magGraph=true;

int multiType;  // 1 for tricopter, 2 for quad+, 3 for quadX, ...

cDataArray accPITCH   = new cDataArray(100);
cDataArray accROLL    = new cDataArray(100);
cDataArray accYAW     = new cDataArray(100);
cDataArray gyroPITCH  = new cDataArray(100);
cDataArray gyroROLL   = new cDataArray(100);
cDataArray gyroYAW    = new cDataArray(100);
cDataArray baroData   = new cDataArray(100);
cDataArray magData    = new cDataArray(100);

Numberbox confP_ROLL,confP_PITCH,confP_YAW,confP_LEVEL;
Numberbox confI_ROLL,confI_PITCH,confI_YAW,confI_LEVEL;
Numberbox confD_ROLL,confD_PITCH,confD_YAW;
Numberbox confRC_RATE;
Numberbox confRC_EXPO;
Numberbox rollPitchRate;
Numberbox yawRate;
Numberbox dynamic_THR_PID;

Slider rcStickThrottleSlider,rcStickRollSlider,rcStickPitchSlider,rcStickYawSlider;
Slider rcStickAUX1Slider,rcStickAUX2Slider,rcStickCAM1Slider,rcStickCAM2Slider;

Slider motSliderV0,motSliderV1,motSliderV2,motSliderV3,motSliderV4,motSliderV5;
Slider servoSliderH1,servoSliderH2,servoSliderH3,servoSliderH4;
Slider servoSliderV0,servoSliderV1,servoSliderV2;

Slider axSlider,aySlider,azSlider,gxSlider,gySlider,gzSlider;
Slider baroSlider;
Slider magSlider;

Slider scaleSlider;

Button buttonREAD,buttonWRITE,buttonCALIBRATE,buttonSTART,buttonSTOP;

Button buttonNunchuk,buttonI2cAcc,buttonI2cBaro,buttonI2cMagneto;
Button buttonI2cAccActive,buttonI2cBaroActive,buttonI2cMagnetoActive;

color yellow_ = color(200, 200, 20);
color green_ = color(30, 120, 30);
color red_ = color(120, 30, 30);
boolean graphEnable = false;boolean readEnable = false;boolean writeEnable = false;boolean calibrateEnable = false;

float gx,gy,gz;
float ax,ay,az;
float baro = 0;
float mag = 0;
float angx,angy = 0;
float r;
int init_com = 0;
int graph_on = 0;

int pMeterSum = 0, intPowerTrigger = 0, bytevbat = 0;
Numberbox confPowerTrigger;

float mot0=1000,mot1=1000,mot2=1000,mot3=1000,mot4=1000,mot5=1000;
float servo0=1500,servo1=1500,servo2=1500,servo3=1500;
float rcThrottle = 1500,rcRoll = 1500,rcPitch = 1500,rcYaw =1500,
      rcAUX1=1500, rcAUX2=1500, rcCAM1=1500, rcCAM2=1500;
int nunchukPresent,i2cAccPresent,i2cBaroPresent,i2cMagnetoPresent,levelMode;

float time1,time2;
int cycleTime;

int  byteP_ROLL,byteI_ROLL,byteD_ROLL,
     byteP_PITCH,byteI_PITCH,byteD_PITCH,
     byteP_YAW,byteI_YAW,byteD_YAW,
     byteP_LEVEL,byteI_LEVEL,
     byteRC_RATE,byteRC_EXPO,
     byteRollPitchRate,byteYawRate,
     byteDynThrPID;

CheckBox LevelCheckbox,MagCheckbox,BaroCheckbox,CamStabCheckbox,CamTrigCheckbox;


int activationLevelByte,activationMagByte,activationBaroByte,activationCamStabByte,activationCamTrigByte;

PFont font8,font12,font15;

// coded by Eberhard Rensch
// Truncates a long port name for better (readable) display in the GUI
String shortifyPortName(String portName, int maxlen)  {
  String shortName = portName;
  if(shortName.startsWith("/dev/"))
    shortName = shortName.substring(5);  

  if(shortName.startsWith("tty.")) // get rid off leading tty. part of device name
    shortName = shortName.substring(4); 
    
  if(portName.length()>maxlen) {
    shortName = shortName.substring(0,(maxlen-1)/2) + "~" +shortName.substring(shortName.length()-(maxlen-(maxlen-1)/2));
  }
  if(shortName.startsWith("cu.")) // only collect the corresponding tty. devices
    shortName = "";
  return shortName;
}


void setup() {
  size(windowsX,windowsY,OPENGL);
  frameRate(15); 

  font8 = createFont("Arial bold",8,false);
  font12 = createFont("Arial bold",12,false);
  font15 = createFont("Arial bold",15,false);
  
  controlP5 = new ControlP5(this); // initialize the GUI controls
  controlP5.setControlFont(font12);

  g_graph  = new cGraph(120,540, 480, 250);
  // make a listbox and populate it with the available comm ports
  commListbox = controlP5.addListBox("portComList",5,65,110,240); //addListBox(name,x,y,width,height)
  commListbox.captionLabel().set("PORT COM");
  commListbox.setColorBackground(red_);
  for(int i=0;i<Serial.list().length;i++) {
    String pn = shortifyPortName(Serial.list()[i], 13);
    if (pn.length() >0 ) commListbox.addItem(pn,i); // addItem(name,value)
  }

  // text label for which comm port selected
  txtlblWhichcom = controlP5.addTextlabel("txtlblWhichcom","No Port Selected",5,42); // textlabel(name,text,x,y)
    
  buttonSTART = controlP5.addButton("START",1,xGraph+110,yGraph-30,40,19); buttonSTART.setColorBackground(red_);
  buttonSTOP = controlP5.addButton("STOP",1,xGraph+160,yGraph-30,40,19); buttonSTOP.setColorBackground(red_);

  buttonNunchuk = controlP5.addButton("NUNCHUK",1,xButton,yButton,70,15);buttonNunchuk.setColorBackground(red_);
  buttonI2cAcc = controlP5.addButton("ACC",1,xButton,yButton+17,70,15); buttonI2cAcc.setColorBackground(red_);
  buttonI2cBaro = controlP5.addButton("BARO",1,xButton,yButton+34,70,15); buttonI2cBaro.setColorBackground(red_);
  buttonI2cMagneto = controlP5.addButton("MAG",1,xButton,yButton+51,70,15); buttonI2cMagneto.setColorBackground(red_);

  
  buttonI2cAccActive = controlP5.addButton("OFF",1,xButton+75,yButton,70,32);buttonI2cAccActive.setColorBackground(red_);
  buttonI2cBaroActive = controlP5.addButton("OFF",1,xButton+75,yButton+34,70,15);buttonI2cBaroActive.setColorBackground(red_);
  buttonI2cMagnetoActive = controlP5.addButton("OFF",1,xButton+75,yButton+51,70,15);buttonI2cMagnetoActive.setColorBackground(red_);


  controlP5.addToggle("ACC_ROLL",true,xGraph-7,yGraph,20,15);
  controlP5.addToggle("ACC_PITCH",true,xGraph-7,yGraph+30,20,15);
  controlP5.addToggle("ACC_Z",true,xGraph-7,yGraph+60,20,15);
  controlP5.addToggle("GYRO_ROLL",true,xGraph-7,yGraph+90,20,15);
  controlP5.addToggle("GYRO_PITCH",true,xGraph-7,yGraph+120,20,15);
  controlP5.addToggle("GYRO_YAW",true,xGraph-7,yGraph+150,20,15);
  controlP5.addToggle("BARO",true,xGraph-7,yGraph+180,20,15);
  controlP5.addToggle("MAG",true,xGraph-7,yGraph+210,20,15);

  axSlider   = controlP5.addSlider("1",-400,+400,0,xGraph+60,yGraph+10,50,10);axSlider.setDecimalPrecision(0);
  aySlider   = controlP5.addSlider("2",-400,+400,0,xGraph+60,yGraph+40,50,10);aySlider.setDecimalPrecision(0);
  azSlider   = controlP5.addSlider("3",-400,+400,0,xGraph+60,yGraph+70,50,10);azSlider.setDecimalPrecision(0);
  gxSlider   = controlP5.addSlider("4",-500,+500,0,xGraph+60,yGraph+100,50,10);gxSlider.setDecimalPrecision(0);
  gySlider   = controlP5.addSlider("5",-500,+500,0,xGraph+60,yGraph+130,50,10);gySlider.setDecimalPrecision(0);
  gzSlider   = controlP5.addSlider("6",-500,+500,0,xGraph+60,yGraph+160,50,10);gzSlider.setDecimalPrecision(0);
  baroSlider = controlP5.addSlider("7",-300,+300,0,xGraph+60,yGraph+190,50,10);baroSlider.setDecimalPrecision(0);
  magSlider  = controlP5.addSlider("8",-200,+200,0,xGraph+60,yGraph+220,50,10);magSlider.setDecimalPrecision(0);

  confP_ROLL = controlP5.addNumberbox("",0,xParam+40,yParam+20,30,14);confP_ROLL.setDecimalPrecision(1);confP_ROLL.setMultiplier(0.1);
  confP_ROLL.setDirection(Controller.HORIZONTAL);confP_ROLL.setMin(0);confP_ROLL.setMax(20);confP_ROLL.setColorBackground(red_);
  confI_ROLL = controlP5.addNumberbox("",0,xParam+75,yParam+20,40,14);confI_ROLL.setDecimalPrecision(3);confI_ROLL.setMultiplier(0.001);
  confI_ROLL.setDirection(Controller.HORIZONTAL);confI_ROLL.setMin(0);confI_ROLL.setMax(0.250);confI_ROLL.setColorBackground(red_);
  confD_ROLL = controlP5.addNumberbox("",0,xParam+120,yParam+20,30,14);confD_ROLL.setDecimalPrecision(0);confD_ROLL.setMultiplier(1);
  confD_ROLL.setDirection(Controller.HORIZONTAL);confD_ROLL.setMin(0);confD_ROLL.setMax(50);confD_ROLL.setColorBackground(red_);
  confP_PITCH = controlP5.addNumberbox("",0,xParam+40,yParam+40,30,14);confP_PITCH.setDecimalPrecision(1);confP_PITCH.setMultiplier(0.1);
  confP_PITCH.setDirection(Controller.HORIZONTAL);confP_PITCH.setMin(0);confP_PITCH.setMax(20);confP_PITCH.setColorBackground(red_);
  confI_PITCH = controlP5.addNumberbox("",0,xParam+75,yParam+40,40,14);confI_PITCH.setDecimalPrecision(3);confI_PITCH.setMultiplier(0.001);
  confI_PITCH.setDirection(Controller.HORIZONTAL);confI_PITCH.setMin(0);confI_PITCH.setMax(0.250);confI_PITCH.setColorBackground(red_);
  confD_PITCH = controlP5.addNumberbox("",0,xParam+120,yParam+40,30,14);confD_PITCH.setDecimalPrecision(0);confD_PITCH.setMultiplier(1);
  confD_PITCH.setDirection(Controller.HORIZONTAL);confD_PITCH.setMin(0);confD_PITCH.setMax(50);confD_PITCH.setColorBackground(red_);
  confP_YAW = controlP5.addNumberbox("",0,xParam+40,yParam+60,30,14);confP_YAW.setDecimalPrecision(1);confP_YAW.setMultiplier(0.1);
  confP_YAW.setDirection(Controller.HORIZONTAL);confP_YAW.setMin(0);confP_YAW.setMax(20);confP_YAW.setColorBackground(red_);
  confI_YAW = controlP5.addNumberbox("",0,xParam+75,yParam+60,40,14);confI_YAW.setDecimalPrecision(3);confI_YAW.setMultiplier(0.001);
  confI_YAW.setDirection(Controller.HORIZONTAL);confI_YAW.setMin(0);confI_YAW.setMax(0.250);confI_YAW.setColorBackground(red_);
  confD_YAW = controlP5.addNumberbox("",0,xParam+120,yParam+60,30,14);confD_YAW.setDecimalPrecision(0);confD_YAW.setMultiplier(1);
  confD_YAW.setDirection(Controller.HORIZONTAL);confD_YAW.setMin(0);confD_YAW.setMax(50);confD_YAW.setColorBackground(red_);
  confP_LEVEL = controlP5.addNumberbox("",0,xParam+40,yParam+83,30,14);confP_LEVEL.setDecimalPrecision(1);confP_LEVEL.setMultiplier(0.1);
  confP_LEVEL.setDirection(Controller.HORIZONTAL);confP_LEVEL.setMin(0);confP_LEVEL.setMax(25);confP_LEVEL.setColorBackground(red_);
  confI_LEVEL = controlP5.addNumberbox("",0,xParam+75,yParam+83,40,14);confI_LEVEL.setDecimalPrecision(3);confI_LEVEL.setMultiplier(0.001);
  confI_LEVEL.setDirection(Controller.HORIZONTAL);confI_LEVEL.setMin(0);confI_LEVEL.setMax(0.250);confI_LEVEL.setColorBackground(red_);
  rollPitchRate = controlP5.addNumberbox("",0,xParam+160,yParam+30,30,14);rollPitchRate.setDecimalPrecision(2);rollPitchRate.setMultiplier(0.01);
  rollPitchRate.setDirection(Controller.HORIZONTAL);rollPitchRate.setMin(0);rollPitchRate.setMax(1);rollPitchRate.setColorBackground(red_);
  yawRate = controlP5.addNumberbox("",0,xParam+160,yParam+60,30,14);yawRate.setDecimalPrecision(2);yawRate.setMultiplier(0.01);
  yawRate.setDirection(Controller.HORIZONTAL);yawRate.setMin(0);yawRate.setMax(1);yawRate.setColorBackground(red_); 
  dynamic_THR_PID = controlP5.addNumberbox("",0,xParam+300,yParam+12,30,14);dynamic_THR_PID.setDecimalPrecision(2);dynamic_THR_PID.setMultiplier(0.01);
  dynamic_THR_PID.setDirection(Controller.HORIZONTAL);dynamic_THR_PID.setMin(0);dynamic_THR_PID.setMax(1);dynamic_THR_PID.setColorBackground(red_);

  confRC_RATE = controlP5.addNumberbox("RC RATE",1,xParam+5,yParam+115,40,14);confRC_RATE.setDecimalPrecision(2);confRC_RATE.setMultiplier(0.02);
  confRC_RATE.setDirection(Controller.HORIZONTAL);confRC_RATE.setMin(0);confRC_RATE.setMax(5);confRC_RATE.setColorBackground(red_);
  confRC_EXPO = controlP5.addNumberbox("RC EXPO",0,xParam+5,yParam+145,40,14);confRC_EXPO.setDecimalPrecision(2);confRC_EXPO.setMultiplier(0.01);
  confRC_EXPO.setDirection(Controller.HORIZONTAL);confRC_EXPO.setMin(0);confRC_EXPO.setMax(1);confRC_EXPO.setColorBackground(red_);



  LevelCheckbox = controlP5.addCheckBox("",xParam+220,yParam+120);
  LevelCheckbox.setColorActive(color(255));LevelCheckbox.setColorBackground(color(120));
  LevelCheckbox.setItemsPerRow(6);LevelCheckbox.setSpacingColumn(10);
  LevelCheckbox.addItem("",1);LevelCheckbox.addItem("",2);LevelCheckbox.addItem("",3);
  LevelCheckbox.addItem("",4);LevelCheckbox.addItem("",5);LevelCheckbox.addItem("",6);

  BaroCheckbox = controlP5.addCheckBox("",xParam+220,yParam+135);
  BaroCheckbox.setColorActive(color(255));BaroCheckbox.setColorBackground(color(120));
  BaroCheckbox.setItemsPerRow(6);BaroCheckbox.setSpacingColumn(10);
  BaroCheckbox.addItem("",1);BaroCheckbox.addItem("",2);BaroCheckbox.addItem("",3);
  BaroCheckbox.addItem("",4);BaroCheckbox.addItem("",5);BaroCheckbox.addItem("",6);

  MagCheckbox = controlP5.addCheckBox("",xParam+220,yParam+150);
  MagCheckbox.setColorActive(color(255));MagCheckbox.setColorBackground(color(120));
  MagCheckbox.setItemsPerRow(6);MagCheckbox.setSpacingColumn(10);
  MagCheckbox.addItem("",1);MagCheckbox.addItem("",2);MagCheckbox.addItem("",3);
  MagCheckbox.addItem("",4);MagCheckbox.addItem("",5);MagCheckbox.addItem("",6);

  CamStabCheckbox = controlP5.addCheckBox("",xParam+220,yParam+165);
  CamStabCheckbox.setColorActive(color(255));CamStabCheckbox.setColorBackground(color(120));
  CamStabCheckbox.setItemsPerRow(6);CamStabCheckbox.setSpacingColumn(10);
  CamStabCheckbox.addItem("",1);CamStabCheckbox.addItem("",2);CamStabCheckbox.addItem("",3);
  CamStabCheckbox.addItem("",4);CamStabCheckbox.addItem("",5);CamStabCheckbox.addItem("",6);

  CamTrigCheckbox = controlP5.addCheckBox("",xParam+220,yParam+180);
  CamTrigCheckbox.setColorActive(color(255));CamTrigCheckbox.setColorBackground(color(120));
  CamTrigCheckbox.setItemsPerRow(6);CamTrigCheckbox.setSpacingColumn(10);
  CamTrigCheckbox.addItem("",1);CamTrigCheckbox.addItem("",2);CamTrigCheckbox.addItem("",3);
  CamTrigCheckbox.addItem("",4);CamTrigCheckbox.addItem("",5);CamTrigCheckbox.addItem("",6);


  buttonREAD =      controlP5.addButton("READ",1,xParam+5,yParam+225,60,16);buttonREAD.setColorBackground(red_);
  buttonWRITE =     controlP5.addButton("WRITE",1,xParam+290,yParam+225,60,16);buttonWRITE.setColorBackground(red_);
  buttonCALIBRATE = controlP5.addButton("CALIBRATE",1,xParam+210,yParam+225,70,16);buttonCALIBRATE.setColorBackground(red_);

  rcStickThrottleSlider = controlP5.addSlider("Throttle",900,2100,1500,xRC,yRC,10,100);rcStickThrottleSlider.setDecimalPrecision(0);
  rcStickPitchSlider =    controlP5.addSlider("Pitch",900,2100,1500,xRC+80,yRC,10,100);rcStickPitchSlider.setDecimalPrecision(0);
  rcStickRollSlider =     controlP5.addSlider("Roll",900,2100,1500,xRC,yRC+120,100,10);rcStickRollSlider.setDecimalPrecision(0);
  rcStickYawSlider  =     controlP5.addSlider("Yaw",900,2100,1500,xRC,yRC+140,100,10);rcStickYawSlider.setDecimalPrecision(0);
  rcStickAUX1Slider =     controlP5.addSlider("AUX1",900,2100,1500,xRC,yRC+160,100,10);rcStickAUX1Slider.setDecimalPrecision(0);
  rcStickAUX2Slider =     controlP5.addSlider("AUX2",900,2100,1500,xRC,yRC+180,100,10);rcStickAUX2Slider.setDecimalPrecision(0);
  rcStickCAM1Slider =     controlP5.addSlider("CAM1",900,2100,1500,xRC,yRC+200,100,10);rcStickCAM1Slider.setDecimalPrecision(0);
  rcStickCAM2Slider =     controlP5.addSlider("CAM2",900,2100,1500,xRC,yRC+220,100,10);rcStickCAM2Slider.setDecimalPrecision(0);

  motSliderV0  = controlP5.addSlider("motSliderV0",1000,2000,1500,xMot+50,yMot+15,10,100);motSliderV0.setDecimalPrecision(0);motSliderV0.hide();
  motSliderV1  = controlP5.addSlider("motSliderV1",1000,2000,1500,xMot+100,yMot-15,10,100);motSliderV1.setDecimalPrecision(0);motSliderV1.hide();
  motSliderV2  = controlP5.addSlider("motSliderV2",1000,2000,1500,xMot,yMot-15,10,100);motSliderV2.setDecimalPrecision(0);motSliderV2.hide();
  motSliderV3  = controlP5.addSlider("motSliderV3",1000,2000,1500,xMot+50,yMot+15,10,100);motSliderV3.setDecimalPrecision(0);motSliderV3.hide();
  motSliderV4  = controlP5.addSlider("motSliderV4",1000,2000,1500,xMot+100,yMot-15,10,100);motSliderV4.setDecimalPrecision(0);motSliderV4.hide();
  motSliderV5  = controlP5.addSlider("motSliderV5",1000,2000,1500,xMot,yMot-15,10,100);motSliderV5.setDecimalPrecision(0);motSliderV5.hide();

  servoSliderH1  = controlP5.addSlider("Servo0",1000,2000,1500,xMot,yMot+135,100,10);servoSliderH1.setDecimalPrecision(0);servoSliderH1.hide();
  servoSliderH2 = controlP5.addSlider("Servo1",1000,2000,1500,xMot,yMot-15,100,10);servoSliderH2.setDecimalPrecision(0);servoSliderH2.hide();
  servoSliderH3 = controlP5.addSlider("Servo2",1000,2000,1500,xMot,yMot-15,100,10);servoSliderH3.setDecimalPrecision(0);servoSliderH3.hide();
  servoSliderH4 = controlP5.addSlider("Servo3",1000,2000,1500,xMot,yMot-15,100,10);servoSliderH4.setDecimalPrecision(0);servoSliderH4.hide();
  servoSliderV0  = controlP5.addSlider("Servov0",1000,2000,1500,xMot,yMot+135,10,100);servoSliderV0.setDecimalPrecision(0);servoSliderV0.hide();
  servoSliderV1  = controlP5.addSlider("Servov1",1000,2000,1500,xMot,yMot+135,10,100);servoSliderV1.setDecimalPrecision(0);servoSliderV1.hide();
  servoSliderV2 = controlP5.addSlider("Servov2",1000,2000,1500,xMot,yMot-15,10,100);servoSliderV2.setDecimalPrecision(0);servoSliderV2.hide();

  scaleSlider = controlP5.addSlider("SCALE",0,10,1,xGraph+400,yGraph-30,150,20);
  
  confPowerTrigger = controlP5.addNumberbox("",0,xGraph+50,yGraph-29,40,14);confPowerTrigger.setDecimalPrecision(0);confPowerTrigger.setMultiplier(10);
  confPowerTrigger.setDirection(Controller.HORIZONTAL);confPowerTrigger.setMin(0);confPowerTrigger.setMax(65535);confPowerTrigger.setColorBackground(red_);
}

void draw() {
  int i;
  float val,inter,a,b;
 
  background(80);
  textFont(font15);
  text("MultiWii conf",0,16);text("v1.7", 0, 32);
  text("Cycle Time:",230,285);text(cycleTime, 330, 285);

  textFont(font12);
  text("Power:",xGraph-5,yGraph-30); text(pMeterSum,xGraph+50,yGraph-30);
  text("pAlarm:",xGraph-5,yGraph-15);  //text(intPowerTrigger,xGraph+50,yGraph-15);
  text("Volt:",xGraph-5,yGraph-2);  text(bytevbat/10.0,xGraph+50,yGraph-2);

  time1=millis();
  if (init_com==1) {
    if  (g_serial.available() >100) g_serial.clear();
    while (g_serial.available() >73) processSerialData();
    if ((time1-time2)>100 && graph_on==1 && g_serial.available()<60) {
      g_serial.write('M'); // request Multiwii @ arduino to send all data to GUI
      time2=time1;
    }
  }
  
  axSlider.setValue(ax);aySlider.setValue(ay);azSlider.setValue(az);
  gxSlider.setValue(gx);gySlider.setValue(gy);gzSlider.setValue(gz);
  baroSlider.setValue(baro);
  magSlider.setValue(mag);

  motSliderV0.setValue(mot0);motSliderV1.setValue(mot1);motSliderV2.setValue(mot2);
  motSliderV3.setValue(mot3);motSliderV4.setValue(mot4);motSliderV5.setValue(mot5);

  servoSliderH1.setValue(servo0);servoSliderH2.setValue(servo1);servoSliderH3.setValue(servo2);servoSliderH4.setValue(servo3);
  servoSliderV0.setValue(servo0);servoSliderV1.setValue(servo1);servoSliderV2.setValue(servo2);

  rcStickThrottleSlider.setValue(rcThrottle);rcStickRollSlider.setValue(rcRoll);rcStickPitchSlider.setValue(rcPitch);rcStickYawSlider.setValue(rcYaw);
  rcStickAUX1Slider.setValue(rcAUX1);rcStickAUX2Slider.setValue(rcAUX2);rcStickCAM1Slider.setValue(rcCAM1);rcStickCAM2Slider.setValue(rcCAM2);

  stroke(255); 
  a=radians(angx);
  b=radians(angy);

  float size = 30.0;

  pushMatrix();
  camera(xObj,yObj,300/tan(PI*60.0/360.0),xObj/2+30,yObj/2-40,0,0,1,0);
  translate(xObj,yObj);
  directionalLight(200,200,200, 0, 0, -1);
  rotateX(b);rotateY(a);
  stroke(150,255,150);
  strokeWeight(0);sphere(size/3);strokeWeight(3);
  line(0,0, 10,0,-size-5,10);line(0,-size-5,10,+size/4,-size/2,10); line(0,-size-5,10,-size/4,-size/2,10);
  stroke(255);
  
  if (multiType == 1) { //TRI
    ellipse(-size, -size, size, size);
    ellipse(+size, -size, size, size);
    ellipse(0,  +size,size, size);
    line(-size,-size, 0,0);
    line(+size,-size, 0,0);  
    line(0,+size, 0,0);
    noLights();
    textFont(font12);
    text(" TRICOPTER", -40,-50);camera();popMatrix();
 
    motSliderV0.setPosition(xMot+50,yMot+15);motSliderV0.setHeight(100);motSliderV0.setCaptionLabel("REAR");motSliderV0.show();
    motSliderV1.setPosition(xMot+100,yMot-15);motSliderV1.setHeight(100);motSliderV1.setCaptionLabel("RIGHT");motSliderV1.show();
    motSliderV2.setPosition(xMot,yMot-15);motSliderV2.setHeight(100);motSliderV2.setCaptionLabel("LEFT");motSliderV2.show();
    servoSliderH1.setPosition(xMot,yMot+135);servoSliderH1.setCaptionLabel("SERVO");servoSliderH1.show(); 

    motSliderV3.hide();motSliderV4.hide();motSliderV5.hide();
    servoSliderH2.hide();servoSliderH3.hide();servoSliderH4.hide();
    servoSliderV0.hide();servoSliderV1.hide();servoSliderV2.hide();

  } else if (multiType == 2) { //QUAD+
    ellipse(0,  -size,   size,size);
    ellipse(0,  +size,   size, size);
    ellipse(+size, 0,    size , size );
    ellipse(-size, 0,    size , size );
    line(-size,0, +size,0);
    line(0,-size, 0,+size);
    noLights();
    textFont(font12);
    text("QUADRICOPTER +", -40,-50);camera();popMatrix();
    
    motSliderV0.setPosition(xMot+50,yMot+75);motSliderV0.setHeight(60);motSliderV0.setCaptionLabel("REAR");motSliderV0.show();
    motSliderV1.setPosition(xMot+100,yMot+35);motSliderV1.setHeight(60);motSliderV1.setCaptionLabel("RIGHT");motSliderV1.show();
    motSliderV2.setPosition(xMot,yMot+35);motSliderV2.setHeight(60);motSliderV2.setCaptionLabel("LEFT");motSliderV2.show();
    motSliderV3.setPosition(xMot+50,yMot-15);motSliderV3.setHeight(60);motSliderV3.setCaptionLabel("FRONT");motSliderV3.show();
    
    motSliderV4.hide();motSliderV5.hide();
    servoSliderH1.hide();servoSliderH2.hide();servoSliderH3.hide();servoSliderH4.hide();
    servoSliderV0.hide();servoSliderV1.hide();servoSliderV2.hide();
  } else if (multiType == 3) { //QUAD X
    ellipse(-size,  -size, size, size);
    ellipse(+size,  -size, size, size);
    ellipse(-size,  +size, size, size);
    ellipse(+size,  +size, size, size);
    line(-size,-size, 0,0);
    line(+size,-size, 0,0);
    line(-size,+size, 0,0);
    line(+size,+size, 0,0);
    noLights();
    textFont(font12);
    text("QUADRICOPTER X", -40,-50);camera();popMatrix();
    
    motSliderV0.setPosition(xMot+90,yMot+75);motSliderV0.setHeight(60);motSliderV0.setCaptionLabel("REAR_R");motSliderV0.show();
    motSliderV1.setPosition(xMot+90,yMot-15);motSliderV1.setHeight(60);motSliderV1.setCaptionLabel("FRONT_R");motSliderV1.show();
    motSliderV2.setPosition(xMot+10,yMot+75);motSliderV2.setHeight(60);motSliderV2.setCaptionLabel("REAR_L");motSliderV2.show();
    motSliderV3.setPosition(xMot+10,yMot-15);motSliderV3.setHeight(60);motSliderV3.setCaptionLabel("FRONT_L");motSliderV3.show(); 
    
    motSliderV4.hide();motSliderV5.hide();
    servoSliderH1.hide();servoSliderH2.hide();servoSliderH3.hide();servoSliderH4.hide();
    servoSliderV0.hide();servoSliderV1.hide();servoSliderV2.hide();
  } else if (multiType == 4) { //BI
    ellipse(0-size,  0,   size, size);
    ellipse(0+size,  0,   size, size);
    line(0-size,0, 0,0);  
    line(0+size,0, 0,0);
    line(0,size*1.5, 0,0);
    noLights();
    textFont(font12);
    text("BICOPTER", -30,-20);camera();popMatrix();
   
    motSliderV0.setPosition(xMot,yMot+30);motSliderV0.setHeight(55);motSliderV0.setCaptionLabel("");motSliderV0.show();
    motSliderV1.setPosition(xMot+100,yMot+30);motSliderV1.setHeight(55);motSliderV1.setCaptionLabel("");motSliderV1.show();
    servoSliderH1.setPosition(xMot,yMot+100);servoSliderH1.setWidth(60);servoSliderH1.setCaptionLabel("");servoSliderH1.show();
    servoSliderH2.setPosition(xMot+80,yMot+100);servoSliderH2.setWidth(60);servoSliderH2.setCaptionLabel("");servoSliderH2.show();
  } else if (multiType == 5) { //GIMBAL
    noLights();
    textFont(font12);
    text("GIMBAL", -20,-10);camera();popMatrix();
  
    textFont(font12);
    text("GIMBAL", xMot,yMot+25);
 
    servoSliderH3.setPosition(xMot,yMot+75);servoSliderH3.setCaptionLabel("ROLL");servoSliderH3.show();
    servoSliderH2.setPosition(xMot,yMot+35);servoSliderH2.setCaptionLabel("PITCH");servoSliderH2.show();

    motSliderV0.hide();motSliderV1.hide();motSliderV2.hide();motSliderV3.hide();motSliderV4.hide();motSliderV5.hide();
    servoSliderH1.hide();servoSliderH4.hide();
    servoSliderV0.hide();servoSliderV1.hide();servoSliderV2.hide();
  } else if (multiType == 6) { //Y6
    ellipse(-size,-size,size,size);ellipse(size,-size,size,size);ellipse(0,-2+size,size,size);
    translate(0,0,7);
    ellipse(-5-size,-5-size,size,size);ellipse(5+size,-5-size,size,size);ellipse(0,3+size,size,size);
    line(-size,-size,0,0);line(+size,-size, 0,0);line(0,+size, 0,0);
    noLights();
    textFont(font12);
    text("TRICOPTER Y6", -40,-55);camera();popMatrix();

    motSliderV0.setPosition(xMot+50,yMot+23);motSliderV0.setHeight(50);motSliderV0.setCaptionLabel("REAR");motSliderV0.show();
    motSliderV1.setPosition(xMot+100,yMot-18);motSliderV1.setHeight(50);motSliderV1.setCaptionLabel("RIGHT");motSliderV1.show();
    motSliderV2.setPosition(xMot,yMot-18);motSliderV2.setHeight(50);motSliderV2.setCaptionLabel("LEFT");motSliderV2.show();
    motSliderV3.setPosition(xMot+50,yMot+87);motSliderV3.setHeight(50);motSliderV3.setCaptionLabel("U_REAR");motSliderV3.show();
    motSliderV4.setPosition(xMot+100,yMot+48);motSliderV4.setHeight(50);motSliderV4.setCaptionLabel("U_RIGHT");motSliderV4.show();
    motSliderV5.setPosition(xMot,yMot+48);motSliderV5.setHeight(50);motSliderV5.setCaptionLabel("U_LEFT");motSliderV5.show();

    servoSliderH1.hide();servoSliderH2.hide();servoSliderH3.hide();servoSliderH4.hide();
    servoSliderV0.hide();servoSliderV1.hide();servoSliderV2.hide();
  } else if (multiType == 7) { //HEX6
    ellipse(-size,-0.55*size,size,size);ellipse(size,-0.55*size,size,size);ellipse(-size,+0.55*size,size,size);
    ellipse(size,+0.55*size,size,size);ellipse(0,-size,size,size);ellipse(0,+size,size,size);
    line(-size,-0.55*size,0,0);line(size,-0.55*size,0,0);line(-size,+0.55*size,0,0);
    line(size,+0.55*size,0,0);line(0,+size,0,0);  line(0,-size,0,0);
    noLights();
    textFont(font12);
    text("HEXACOPTER", -40,-50);camera();popMatrix();

    motSliderV0.setPosition(xMot+90,yMot+65);motSliderV0.setHeight(50);motSliderV0.setCaptionLabel("REAR_R");motSliderV0.show();
    motSliderV1.setPosition(xMot+90,yMot-5);motSliderV1.setHeight(50);motSliderV1.setCaptionLabel("FRONT_R");motSliderV1.show();
    motSliderV2.setPosition(xMot+5,yMot+65);motSliderV2.setHeight(50);motSliderV2.setCaptionLabel("REAR_L");motSliderV2.show();
    motSliderV3.setPosition(xMot+5,yMot-5);motSliderV3.setHeight(50);motSliderV3.setCaptionLabel("FRONT_L");motSliderV3.show(); 
    motSliderV4.setPosition(xMot+50,yMot-20);motSliderV4.setHeight(50);motSliderV4.setCaptionLabel("FRONT");motSliderV4.show(); 
    motSliderV5.setPosition(xMot+50,yMot+90);motSliderV5.setHeight(50);motSliderV5.setCaptionLabel("REAR");motSliderV5.show(); 

    servoSliderH1.hide();servoSliderH2.hide();servoSliderH3.hide();servoSliderH4.hide();
    servoSliderV0.hide();servoSliderV1.hide();servoSliderV2.hide();

  } else if (multiType == 8) { //FLYING_WING
    line(0,0, 1.8*size,size);line(1.8*size,size,1.8*size,size-30);  line(1.8*size,size-30,0,-1.5*size);
    line(0,0, -1.8*size,+size);line(-1.8*size,size,-1.8*size,+size-30);    line(-1.8*size,size-30,0,-1.5*size);
    noLights();
    textFont(font12);
    text("FLYING WING", -40,-50);camera();popMatrix();

    servoSliderV1.setPosition(xMot+10,yMot+10);servoSliderV1.setCaptionLabel("LEFT");servoSliderV1.show(); 
    servoSliderV2.setPosition(xMot+90,yMot+10);servoSliderV2.setCaptionLabel("RIGHT");servoSliderV2.show(); 

    motSliderV0.hide();motSliderV1.hide();motSliderV2.hide();motSliderV3.hide();motSliderV4.hide();motSliderV5.hide();
    servoSliderH1.hide();servoSliderH2.hide();servoSliderH3.hide();servoSliderH4.hide();
  } else if (multiType == 9) { //Y4
    ellipse(-size,  -size, size, size);
    ellipse(+size,  -size, size, size);
    ellipse(0,  +size, size+2, size+2);
    line(-size,-size, 0,0);
    line(+size,-size, 0,0);
    line(0,+size, 0,0);
    translate(0,0,7);
    ellipse(0,  +size, size, size);

    noLights();
    textFont(font12);
    text("Y4", -5,-50);camera();popMatrix();
    
    motSliderV0.setPosition(xMot+80,yMot+75);motSliderV0.setHeight(60);motSliderV0.setCaptionLabel("REAR_1");motSliderV0.show();
    motSliderV1.setPosition(xMot+90,yMot-15);motSliderV1.setHeight(60);motSliderV1.setCaptionLabel("FRONT_R");motSliderV1.show();
    motSliderV2.setPosition(xMot+30,yMot+75);motSliderV2.setHeight(60);motSliderV2.setCaptionLabel("REAR_2");motSliderV2.show();
    motSliderV3.setPosition(xMot+10,yMot-15);motSliderV3.setHeight(60);motSliderV3.setCaptionLabel("FRONT_L");motSliderV3.show(); 
    
    motSliderV4.hide();motSliderV5.hide();
    servoSliderH1.hide();servoSliderH2.hide();servoSliderH3.hide();servoSliderH4.hide();
    servoSliderV0.hide();servoSliderV1.hide();servoSliderV2.hide();
  } else if (multiType == 10) { //HEX6 X
    ellipse(-0.55*size,-size,size,size);ellipse(-0.55*size,size,size,size);ellipse(+0.55*size,-size,size,size);
    ellipse(+0.55*size,size,size,size);ellipse(-size,0,size,size);ellipse(+size,0,size,size);
    line(-0.55*size,-size,0,0);line(-0.55*size,size,0,0);line(+0.55*size,-size,0,0);
    line(+0.55*size,size,0,0);line(+size,0,0,0);  line(-size,0,0,0);
    noLights();
    textFont(font12);
    text("HEXACOPTER X", -45,-50);camera();popMatrix();

    motSliderV0.setPosition(xMot+80,yMot+90);motSliderV0.setHeight(45);motSliderV0.setCaptionLabel("REAR_R");motSliderV0.show();
    motSliderV1.setPosition(xMot+80,yMot-20);motSliderV1.setHeight(45);motSliderV1.setCaptionLabel("FRONT_R");motSliderV1.show();
    motSliderV2.setPosition(xMot+25,yMot+90);motSliderV2.setHeight(45);motSliderV2.setCaptionLabel("REAR_L");motSliderV2.show();
    motSliderV3.setPosition(xMot+25,yMot-20);motSliderV3.setHeight(45);motSliderV3.setCaptionLabel("FRONT_L");motSliderV3.show(); 
    motSliderV4.setPosition(xMot+90,yMot+35);motSliderV4.setHeight(45);motSliderV4.setCaptionLabel("RIGHT");motSliderV4.show(); 
    motSliderV5.setPosition(xMot+5,yMot+35);motSliderV5.setHeight(45);motSliderV5.setCaptionLabel("LEFT");motSliderV5.show(); 

    servoSliderH1.hide();servoSliderH2.hide();servoSliderH3.hide();servoSliderH4.hide();
    servoSliderV0.hide();servoSliderV1.hide();servoSliderV2.hide();
  } else {
    noLights();
    camera();
    popMatrix();
  }
  
  pushMatrix();
  translate(xObj+30,yObj-165);
  textFont(font15);text("ROLL", -90, 5);
  rotate(a);
  line(-30,0,+30,0);line(0,0,0,-10);
  popMatrix();
  
  pushMatrix();
  translate(xObj+30,yObj-100);
  textFont(font15);text("PITCH", -90, 5);
  rotate(b);
  line(-30,0,30,0);line(+30,0,30-size/2 ,size/3);  line(+30,0,30-size/2 ,-size/3);  
  popMatrix();
 
  pushMatrix();
  translate(xObj-40,yObj-133);
  size=15;
  strokeWeight(1.5);
  fill(0);stroke(0);
  ellipse(0,  0,   2*size+7, 2*size+7);
  stroke(255);
  float head= mag*PI/180;
  rotate(head);
  line(0,size, 0,-size);   
  line(0,-size, -5 ,-size+10);  
  line(0,-size, +5 ,-size+10);  
  popMatrix();
    
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
   if (scaleSlider.value()<2) g_graph.drawLine(accYAW, -500, +500);
   else g_graph.drawLine(accYAW, 200*scaleSlider.value()-500,200*scaleSlider.value()+500);
  }
  stroke(200, 200, 0);  if (gxGraph)   g_graph.drawLine(gyroROLL, -300, +300);
  stroke(0, 255, 255);  if (gyGraph)   g_graph.drawLine(gyroPITCH, -300, +300);
  stroke(255, 0, 255);  if (gzGraph)   g_graph.drawLine(gyroYAW, -300, +300);
  stroke(125, 125, 125);if (baroGraph) g_graph.drawLine(baroData, -300, +300);
  stroke(225, 225, 125);if (magGraph)  g_graph.drawLine(magData, -370, +370);
  
  strokeWeight(2);
  stroke(255, 0, 0);     line(xGraph+25, yGraph+10, xGraph+60, yGraph+10);
  stroke(0, 255, 0);     line(xGraph+25, yGraph+40, xGraph+60, yGraph+40);
  stroke(0, 0, 255);     line(xGraph+25, yGraph+70, xGraph+60, yGraph+70);
  stroke(200, 200, 0);   line(xGraph+25, yGraph+100, xGraph+60, yGraph+100);
  stroke(0, 255, 255);   line(xGraph+25, yGraph+130, xGraph+60, yGraph+130);
  stroke(255, 0, 255);   line(xGraph+25, yGraph+160, xGraph+60, yGraph+160);
  stroke(125, 125, 125); line(xGraph+25, yGraph+190, xGraph+60, yGraph+190);
  stroke(225, 225, 125); line(xGraph+25, yGraph+220, xGraph+60, yGraph+220);
  fill(0, 0, 0);

  strokeWeight(3);
  stroke(0);
  rectMode(CORNERS);
  rect(xMot-5,yMot-20, xMot+145, yMot+150);
  rect(xRC-5,yRC-5, xRC+185, yRC+235);
  rect(xParam,yParam, xParam+355, yParam+245);

  int xSens       = xParam + 70;
  int ySens       = yParam + 120;
  stroke(255);
  a=min(confRC_RATE.value(),1);
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
  if (confRC_RATE.value()>1) { 
    stroke(220,100,100);
    ellipse(xSens+70, ySens, 7, 7);
  }


  fill(255);
  textFont(font15);
  text("P",xParam+45,yParam+15);text("I",xParam+90,yParam+15);text("D",xParam+130,yParam+15);
  textFont(font12);
  text("RATE",xParam+160,yParam+15);
  text("ROLL",xParam+1,yParam+32);text("PITCH",xParam+3,yParam+52);text("YAW",xParam+3,yParam+72);text("LEVEL",xParam+1,yParam+95); 
  text("Throttle PID",xParam+220,yParam+15);text("attenuation",xParam+220,yParam+30);

  text("AUX1",xParam+235,yParam+100);text("AUX2",xParam+295,yParam+100);
  text("LEVEL",xParam+180,yParam+130);
  text("BARO",xParam+180,yParam+145);
  text("MAG",xParam+180,yParam+160);
  textFont(font8);
  text("CAMSTAB",xParam+175,yParam+175);
  text("CAMTRIG",xParam+175,yParam+190);
  
  textFont(font8); 
  text("LOW",xParam+217,yParam+110);text("MID",xParam+237,yParam+110);text("HIGH",xParam+254,yParam+110);
  text("LOW",xParam+280,yParam+110);text("MID",xParam+301,yParam+110);text("HIGH",xParam+318,yParam+110);
}

void ACC_ROLL(boolean theFlag) {axGraph = theFlag;}
void ACC_PITCH(boolean theFlag) {ayGraph = theFlag;}
void ACC_Z(boolean theFlag) {azGraph = theFlag;}
void GYRO_ROLL(boolean theFlag) {gxGraph = theFlag;}
void GYRO_PITCH(boolean theFlag) {gyGraph = theFlag;}
void GYRO_YAW(boolean theFlag) {gzGraph = theFlag;}
void BARO(boolean theFlag) {baroGraph = theFlag;}
void MAG(boolean theFlag) {magGraph = theFlag;}

public void controlEvent(ControlEvent theEvent) {
  if (theEvent.isGroup())
    if (theEvent.name()=="portComList") InitSerial(theEvent.group().value()); // initialize the serial port selected
}

public void START() {
  if(graphEnable == false) {return;}
  graph_on=1;
  readEnable = true;calibrateEnable = true;
  buttonREAD.setColorBackground(green_);
  buttonCALIBRATE.setColorBackground(green_);
  g_serial.clear();
}

public void STOP() {
  graph_on=0;
}

public void READ() {
  if(readEnable == false) {return;}
  confP_ROLL.setValue(byteP_ROLL/10.0);confI_ROLL.setValue(byteI_ROLL/1000.0);confD_ROLL.setValue(byteD_ROLL);
  confP_PITCH.setValue(byteP_PITCH/10.0);confI_PITCH.setValue(byteI_PITCH/1000.0);confD_PITCH.setValue(byteD_PITCH);
  confP_YAW.setValue(byteP_YAW/10.0);confI_YAW.setValue(byteI_YAW/1000.0);confD_YAW.setValue(byteD_YAW);
  confP_LEVEL.setValue(byteP_LEVEL/10.0);confI_LEVEL.setValue(byteI_LEVEL/1000.0);
  confRC_RATE.setValue(byteRC_RATE/50.0);
  confRC_EXPO.setValue(byteRC_EXPO/100.0);
  rollPitchRate.setValue(byteRollPitchRate/100.0);
  yawRate.setValue(byteYawRate/100.0);

  dynamic_THR_PID.setValue(byteDynThrPID/100.0);

  buttonWRITE.setColorBackground(green_);
  confP_ROLL.setColorBackground(green_);confI_ROLL.setColorBackground(green_);confD_ROLL.setColorBackground(green_);
  confP_PITCH.setColorBackground(green_);confI_PITCH.setColorBackground(green_);confD_PITCH.setColorBackground(green_);
  confP_YAW.setColorBackground(green_);confI_YAW.setColorBackground(green_);confD_YAW.setColorBackground(green_);
  confP_LEVEL.setColorBackground(green_);confI_LEVEL.setColorBackground(green_);
  confRC_RATE.setColorBackground(green_);
  confRC_EXPO.setColorBackground(green_);
  rollPitchRate.setColorBackground(green_);
  yawRate.setColorBackground(green_);
  dynamic_THR_PID.setColorBackground(green_);

  if ((byte(activationLevelByte)&32) >0) LevelCheckbox.activate(5); else LevelCheckbox.deactivate(5);if ((byte(activationLevelByte)&16) >0) LevelCheckbox.activate(4); else LevelCheckbox.deactivate(4);
  if ((byte(activationLevelByte)&8) >0) LevelCheckbox.activate(3); else LevelCheckbox.deactivate(3);if ((byte(activationLevelByte)&4) >0) LevelCheckbox.activate(2); else LevelCheckbox.deactivate(2);
  if ((byte(activationLevelByte)&2) >0) LevelCheckbox.activate(1); else LevelCheckbox.deactivate(1);if ((byte(activationLevelByte)&1) >0) LevelCheckbox.activate(0); else LevelCheckbox.deactivate(0);
  if ((byte(activationBaroByte)&32) >0) BaroCheckbox.activate(5); else BaroCheckbox.deactivate(5);if ((byte(activationBaroByte)&16) >0) BaroCheckbox.activate(4); else BaroCheckbox.deactivate(4);
  if ((byte(activationBaroByte)&8) >0) BaroCheckbox.activate(3); else BaroCheckbox.deactivate(3);if ((byte(activationBaroByte)&4) >0) BaroCheckbox.activate(2); else BaroCheckbox.deactivate(2);
  if ((byte(activationBaroByte)&2) >0) BaroCheckbox.activate(1); else BaroCheckbox.deactivate(1);if ((byte(activationBaroByte)&1) >0) BaroCheckbox.activate(0); else BaroCheckbox.deactivate(0);
  if ((byte(activationMagByte)&32) >0) MagCheckbox.activate(5); else MagCheckbox.deactivate(5);if ((byte(activationMagByte)&16) >0) MagCheckbox.activate(4); else MagCheckbox.deactivate(4);
  if ((byte(activationMagByte)&8) >0) MagCheckbox.activate(3); else MagCheckbox.deactivate(3);if ((byte(activationMagByte)&4) >0) MagCheckbox.activate(2); else MagCheckbox.deactivate(2);
  if ((byte(activationMagByte)&2) >0) MagCheckbox.activate(1); else MagCheckbox.deactivate(1);if ((byte(activationMagByte)&1) >0) MagCheckbox.activate(0); else MagCheckbox.deactivate(0);

  if ((byte(activationCamStabByte)&32) >0) CamStabCheckbox.activate(5); else CamStabCheckbox.deactivate(5);if ((byte(activationCamStabByte)&16) >0) CamStabCheckbox.activate(4); else CamStabCheckbox.deactivate(4);
  if ((byte(activationCamStabByte)&8) >0) CamStabCheckbox.activate(3); else CamStabCheckbox.deactivate(3);if ((byte(activationCamStabByte)&4) >0) CamStabCheckbox.activate(2); else CamStabCheckbox.deactivate(2);
  if ((byte(activationCamStabByte)&2) >0) CamStabCheckbox.activate(1); else CamStabCheckbox.deactivate(1);if ((byte(activationCamStabByte)&1) >0) CamStabCheckbox.activate(0); else CamStabCheckbox.deactivate(0);

  if ((byte(activationCamTrigByte)&32) >0) CamTrigCheckbox.activate(5); else CamTrigCheckbox.deactivate(5);if ((byte(activationCamTrigByte)&16) >0) CamTrigCheckbox.activate(4); else CamTrigCheckbox.deactivate(4);
  if ((byte(activationCamTrigByte)&8) >0) CamTrigCheckbox.activate(3); else CamTrigCheckbox.deactivate(3);if ((byte(activationCamTrigByte)&4) >0) CamTrigCheckbox.activate(2); else CamTrigCheckbox.deactivate(2);
  if ((byte(activationCamTrigByte)&2) >0) CamTrigCheckbox.activate(1); else CamTrigCheckbox.deactivate(1);if ((byte(activationCamTrigByte)&1) >0) CamTrigCheckbox.activate(0); else CamTrigCheckbox.deactivate(0);

  confPowerTrigger.setValue(intPowerTrigger);

  writeEnable = true;
  
}

public void WRITE() {
  if(writeEnable == false) {return;}

  byteP_ROLL = (round(confP_ROLL.value()*10));   byteI_ROLL = (round(confI_ROLL.value()*1000));   byteD_ROLL = (round(confD_ROLL.value()));
  byteP_PITCH = (round(confP_PITCH.value()*10)); byteI_PITCH = (round(confI_PITCH.value()*1000)); byteD_PITCH = (round(confD_PITCH.value()));
  byteP_YAW = (round(confP_YAW.value()*10));     byteI_YAW = (round(confI_YAW.value()*1000));     byteD_YAW = (round(confD_YAW.value()));
  byteP_LEVEL = (round(confP_LEVEL.value()*10)); byteI_LEVEL = (round(confI_LEVEL.value()*1000));
  byteRC_RATE = (round(confRC_RATE.value()*50));
  byteRC_EXPO = (round(confRC_EXPO.value()*100));
  byteRollPitchRate = (round(rollPitchRate.value()*100));
  byteYawRate = (round(yawRate.value()*100));
  byteDynThrPID = (round(dynamic_THR_PID.value()*100));
  
  activationLevelByte = (int)(LevelCheckbox.arrayValue()[0]+LevelCheckbox.arrayValue()[1]*2+LevelCheckbox.arrayValue()[2]*4
                             +LevelCheckbox.arrayValue()[3]*8+LevelCheckbox.arrayValue()[4]*16+LevelCheckbox.arrayValue()[5]*32);
  activationBaroByte = (int)(BaroCheckbox.arrayValue()[0]+BaroCheckbox.arrayValue()[1]*2+BaroCheckbox.arrayValue()[2]*4
                            +BaroCheckbox.arrayValue()[3]*8+BaroCheckbox.arrayValue()[4]*16+BaroCheckbox.arrayValue()[5]*32);
  activationMagByte = (int)(MagCheckbox.arrayValue()[0]+MagCheckbox.arrayValue()[1]*2+MagCheckbox.arrayValue()[2]*4
                           +MagCheckbox.arrayValue()[3]*8+MagCheckbox.arrayValue()[4]*16+MagCheckbox.arrayValue()[5]*32);
  activationCamStabByte = (int)(CamStabCheckbox.arrayValue()[0]+CamStabCheckbox.arrayValue()[1]*2+CamStabCheckbox.arrayValue()[2]*4
                           +CamStabCheckbox.arrayValue()[3]*8+CamStabCheckbox.arrayValue()[4]*16+CamStabCheckbox.arrayValue()[5]*32);
  activationCamTrigByte = (int)(CamTrigCheckbox.arrayValue()[0]+CamTrigCheckbox.arrayValue()[1]*2+CamTrigCheckbox.arrayValue()[2]*4
                           +CamTrigCheckbox.arrayValue()[3]*8+CamTrigCheckbox.arrayValue()[4]*16+CamTrigCheckbox.arrayValue()[5]*32);

  intPowerTrigger = (round(confPowerTrigger.value()));

  int[] s = new int[32];
   s[0] = 'E'; // write to Eeprom @ arduino
   s[1] = byteP_ROLL;  s[2] = byteI_ROLL;  s[3] =  byteD_ROLL;
   s[4] = byteP_PITCH; s[5] = byteI_PITCH; s[6] =  byteD_PITCH;
   s[7] = byteP_YAW;   s[8] = byteI_YAW;   s[9] =  byteD_YAW;
   s[10] = byteP_LEVEL; s[11] = byteI_LEVEL;
   s[12] = byteRC_RATE; s[13] = byteRC_EXPO;
   s[14] = byteRollPitchRate;
   s[15] = byteYawRate;
   s[16] = byteDynThrPID;
   s[17] = activationLevelByte;
   s[18] = activationBaroByte;
   s[19] = activationMagByte;
   s[20] = activationCamStabByte;
   s[21] = activationCamTrigByte;
   s[22] = intPowerTrigger ;
   s[23] = intPowerTrigger >>8 &0xff;
   for(int i =0;i<24;i++)    g_serial.write(char(s[i]));
}

public void CALIBRATE() {
  if(calibrateEnable == false) {return;}
  g_serial.write('S'); // Sensor calibration request
}

// initialize the serial port selected in the listBox
void InitSerial(float portValue) {
  String portPos = Serial.list()[int(portValue)];
  txtlblWhichcom.setValue("COM = " + shortifyPortName(portPos, 8));
  g_serial = new Serial(this, portPos, 115200);
  init_com=1;
  buttonSTART.setColorBackground(green_);buttonSTOP.setColorBackground(green_);commListbox.setColorBackground(green_);
  graphEnable = true;
}

int p;
byte[] inBuf = new byte[128];

int read16() {return (inBuf[p++]&0xff) + (inBuf[p++]<<8);}
int read8()  {return inBuf[p++]&0xff;}

void processSerialData() {
  int frame_size = 88;
  int present=0,mode=0;
  
  if (g_serial.read() == 'M') { // Multiwii @ arduino send all data to GUI

    while (g_serial.available() <frame_size) {}
    g_serial.readBytes(inBuf);
    if (inBuf[frame_size-1] == 'M') {
      p=0;
      ax = read16();ay = read16();az = read16();
      gx = read16();gy = read16();gz = read16();
      baro = read16();
      mag = read16();
      servo0 = read16();servo1 = read16();servo2 = read16();servo3 = read16();
      mot0 = read16();mot1 = read16();mot2 = read16();
      mot3 = read16();mot4 = read16();mot5 = read16(); //36
      rcRoll = read16();rcPitch = read16();rcYaw = read16();rcThrottle = read16();
      rcAUX1 = read16();rcAUX2 = read16();rcCAM1 = read16();rcCAM2 = read16(); //52
      present = read8();
      mode = read8();
      cycleTime = read16();
      angx = read16();angy = read16();
      multiType = read8(); //61
      
      byteP_ROLL = read8();byteI_ROLL = read8();byteD_ROLL = read8();
      byteP_PITCH = read8();byteI_PITCH = read8();byteD_PITCH = read8();
      byteP_YAW = read8();byteI_YAW = read8();byteD_YAW = read8(); //70
      byteP_LEVEL = read8();byteI_LEVEL = read8();
      byteRC_RATE = read8();
      byteRC_EXPO = read8();
      byteRollPitchRate = read8();
      byteYawRate = read8();
      byteDynThrPID = read8();
      activationLevelByte = read8();
      activationBaroByte = read8();
      activationMagByte = read8(); //80
      activationCamStabByte = read8(); //81
      activationCamTrigByte = read8(); //82
      pMeterSum = read16(); // 84
      intPowerTrigger = read16(); // 86
      bytevbat = read8(); //87
      
      if ((present&1) >0) nunchukPresent = 1; else  nunchukPresent = 0;
      if ((present&2) >0) i2cAccPresent = 1; else  i2cAccPresent = 0;
      if ((present&4) >0) i2cBaroPresent = 1; else  i2cBaroPresent = 0;
      if ((present&8) >0) i2cMagnetoPresent = 1; else  i2cMagnetoPresent = 0;

      if ((mode&1) >0) {buttonI2cAccActive.setCaptionLabel("ACTIVE");buttonI2cAccActive.setColorBackground(green_);}
      else {buttonI2cAccActive.setCaptionLabel("OFF");buttonI2cAccActive.setColorBackground(red_);}
 
      if ((mode&2) >0) {buttonI2cBaroActive.setCaptionLabel("ACTIVE");buttonI2cBaroActive.setColorBackground(green_);}
      else {buttonI2cBaroActive.setCaptionLabel("OFF");buttonI2cBaroActive.setColorBackground(red_);}

      if ((mode&4) >0) {buttonI2cMagnetoActive.setCaptionLabel("ACTIVE");buttonI2cMagnetoActive.setColorBackground(green_);}
      else {buttonI2cMagnetoActive.setCaptionLabel("OFF");buttonI2cMagnetoActive.setColorBackground(red_);}

      if (nunchukPresent>0) {buttonNunchuk.setColorBackground(green_);} else {buttonNunchuk.setColorBackground(red_);}
      if (i2cAccPresent>0) {buttonI2cAcc.setColorBackground(green_);} else {buttonI2cAcc.setColorBackground(red_);}
      if (i2cBaroPresent>0) {buttonI2cBaro.setColorBackground(green_);} else {buttonI2cBaro.setColorBackground(red_);}
      if (i2cMagnetoPresent>0) {buttonI2cMagneto.setColorBackground(green_);} else {buttonI2cMagneto.setColorBackground(red_);}
  
      accROLL.addVal(ax);accPITCH.addVal(ay);accYAW.addVal(az);
      gyroROLL.addVal(gx);gyroPITCH.addVal(gy);gyroYAW.addVal(gz);
      baroData.addVal(baro);
      magData.addVal(mag);
    }
  }
}


//********************************************************
//********************************************************
//********************************************************

class cDataArray {
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



