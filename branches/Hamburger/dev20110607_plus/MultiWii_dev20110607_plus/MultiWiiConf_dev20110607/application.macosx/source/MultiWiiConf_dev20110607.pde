/*
 GUI understands following command strings:
 M  Multiwii @ arduino send all data to GUI (former A)
 W  write to Eeprom @ arduino (former C)
 S  acc Sensor calibration request(former D)
 E  mag Sensor calibration request
*/
import processing.serial.*; // serial library
import controlP5.*; // controlP5 library
import processing.opengl.*;

Serial g_serial;
ControlP5 controlP5;
Textlabel txtlblWhichcom,version; 
ListBox commListbox;

cGraph g_graph;
int windowsX    = 800; int windowsY    = 540;
int xGraph      = 10;  int yGraph      = 300;
int xObj        = 700; int yObj        = 450;
int xParam      = 120; int yParam      = 10;
int xRC         = 650; int yRC         = 15;
int xMot        = 490; int yMot        = 30;

int xButton    = 485; int yButton    = 185;

boolean axGraph =true,ayGraph=true,azGraph=true,gxGraph=true,gyGraph=true,gzGraph=true,baroGraph=true,magGraph=true;
boolean magxGraph =true,magyGraph=true,magzGraph=true;

int multiType;  // 1 for tricopter, 2 for quad+, 3 for quadX, ...

cDataArray accPITCH   = new cDataArray(100), accROLL    = new cDataArray(100), accYAW     = new cDataArray(100);
cDataArray gyroPITCH  = new cDataArray(100), gyroROLL   = new cDataArray(100), gyroYAW    = new cDataArray(100);
cDataArray magxData   = new cDataArray(100), magyData   = new cDataArray(100), magzData   = new cDataArray(100);
cDataArray baroData   = new cDataArray(100);
cDataArray magData    = new cDataArray(100);

private static final int ROLL = 0;
private static final int PITCH = 1;
private static final int YAW = 2;
private static final int ALT = 3;
private static final int LEVEL = 4;
private static final int MAG = 5;

Numberbox confP[] = new Numberbox[6], confI[] = new Numberbox[6], confD[] = new Numberbox[6];
Numberbox confRC_RATE, confRC_EXPO;
Numberbox rollPitchRate,yawRate;
Numberbox dynamic_THR_PID;

int byteP[] = new int[6], byteI[] = new int[6],byteD[] = new int[6];

int  byteRC_RATE,byteRC_EXPO,
     byteRollPitchRate,byteYawRate,
     byteDynThrPID;

Slider rcStickThrottleSlider,rcStickRollSlider,rcStickPitchSlider,rcStickYawSlider;
Slider rcStickAUX1Slider,rcStickAUX2Slider,rcStickCAM1Slider,rcStickCAM2Slider;

Slider motSliderV0,motSliderV1,motSliderV2,motSliderV3,motSliderV4,motSliderV5;
Slider servoSliderH1,servoSliderH2,servoSliderH3,servoSliderH4;
Slider servoSliderV0,servoSliderV1,servoSliderV2;

Slider axSlider,aySlider,azSlider,gxSlider,gySlider,gzSlider;
Slider magxSlider,magySlider,magzSlider;
Slider baroSlider;
Slider magSlider;

Slider scaleSlider;

Button buttonREAD,buttonWRITE,buttonCALIBRATE_ACC,buttonCALIBRATE_MAG,buttonSTART,buttonSTOP;

Button buttonNunchuk,buttonI2cAcc,buttonI2cBaro,buttonI2cMagneto;
Button buttonI2cAccActive,buttonI2cBaroActive,buttonI2cMagnetoActive;

color yellow_ = color(200, 200, 20), green_ = color(30, 120, 30), red_ = color(120, 30, 30);
boolean graphEnable = false;boolean readEnable = false;boolean writeEnable = false;boolean calibrateEnable = false;

float gx,gy,gz,ax,ay,az,magx,magy,magz;
float baro = 0;
float mag = 0;
float angx,angy = 0;
float r;
int init_com = 0, graph_on = 0;

int pMeterSum = 0, intPowerTrigger = 0, bytevbat = 0;
Numberbox confPowerTrigger;

float mot[] = new float[8];

float servo0=1500,servo1=1500,servo2=1500,servo3=1500;
float rcThrottle = 1500,rcRoll = 1500,rcPitch = 1500,rcYaw =1500,
      rcAUX1=1500, rcAUX2=1500, rcCAM1=1500, rcCAM2=1500;
int nunchukPresent,i2cAccPresent,i2cBaroPresent,i2cMagnetoPresent,levelMode;

float time1,time2;
int cycleTime;


private static final int BOXACC = 0;
private static final int BOXBARO = 1;
private static final int BOXMAG = 2;
private static final int BOXCAMSTAB = 3;
private static final int BOXCAMTRIG = 4;
private static final int BOXARM = 5;

CheckBox checkbox[] = new CheckBox[6];
int activation[] = new int[6];

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

controlP5.Controller hideLabel(controlP5.Controller c) {
  c.setLabel("");
  c.setLabelVisible(false);
  return c;
}

void setup() {
  size(windowsX,windowsY,OPENGL);
  frameRate(15); 

  font8 = createFont("Arial bold",8,false);font12 = createFont("Arial bold",12,false);font15 = createFont("Arial bold",15,false);
  
  controlP5 = new ControlP5(this); // initialize the GUI controls
  controlP5.setControlFont(font12);

  g_graph  = new cGraph(120,510, 480, 210);
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
    
  buttonSTART = controlP5.addButton("bSTART",1,xGraph+110,yGraph-30,40,19); buttonSTART.setLabel("START"); buttonSTART.setColorBackground(red_);
  buttonSTOP = controlP5.addButton("bSTOP",1,xGraph+160,yGraph-30,40,19); buttonSTOP.setLabel("STOP"); buttonSTOP.setColorBackground(red_);

  buttonNunchuk = controlP5.addButton("bNUNCHUK",1,xButton,yButton,70,15);buttonNunchuk.setColorBackground(red_);buttonNunchuk.setLabel("NUNCHUK");
  buttonI2cAcc = controlP5.addButton("bACC",1,xButton,yButton+17,70,15); buttonI2cAcc.setColorBackground(red_);buttonI2cAcc.setLabel("ACC");
  buttonI2cBaro = controlP5.addButton("bBARO",1,xButton,yButton+34,70,15); buttonI2cBaro.setColorBackground(red_);buttonI2cBaro.setLabel("BARO");
  buttonI2cMagneto = controlP5.addButton("bMAG",1,xButton,yButton+51,70,15); buttonI2cMagneto.setColorBackground(red_);buttonI2cMagneto.setLabel("MAG");

  
  buttonI2cAccActive = controlP5.addButton("accOFF",1,xButton+75,yButton,70,32);buttonI2cAccActive.setColorBackground(red_);buttonI2cAccActive.setLabel("OFF");
  buttonI2cBaroActive = controlP5.addButton("baroOFF",1,xButton+75,yButton+34,70,15);buttonI2cBaroActive.setColorBackground(red_);buttonI2cBaroActive.setLabel("OFF");
  buttonI2cMagnetoActive = controlP5.addButton("magnetoOFF",1,xButton+75,yButton+51,70,15);buttonI2cMagnetoActive.setColorBackground(red_);buttonI2cMagnetoActive.setLabel("OFF");


  controlP5.addToggle("ACC_ROLL",true,xGraph-7,yGraph,20,15);
  controlP5.addToggle("ACC_PITCH",true,xGraph-7,yGraph+30,20,15);
  controlP5.addToggle("ACC_Z",true,xGraph-7,yGraph+60,20,15);
  controlP5.addToggle("GYRO_ROLL",true,xGraph-7,yGraph+90,20,15);
  controlP5.addToggle("GYRO_PITCH",true,xGraph-7,yGraph+120,20,15);
  controlP5.addToggle("GYRO_YAW",true,xGraph-7,yGraph+150,20,15);
  controlP5.addToggle("BARO",true,xGraph-7,yGraph+180,20,15);
  controlP5.addToggle("HEAD",true,xGraph-7,yGraph+210,20,15);
  controlP5.addToggle("MAGX",true,xGraph+127,yGraph+210,20,15);
  controlP5.addToggle("MAGY",true,xGraph+257,yGraph+210,20,15);
  controlP5.addToggle("MAGZ",true,xGraph+387,yGraph+210,20,15);


  axSlider   = controlP5.addSlider("axSlider",-400,+400,0,xGraph+60,yGraph+10,50,10);axSlider.setDecimalPrecision(0);axSlider.setLabel("");
  aySlider   = controlP5.addSlider("aySlider",-400,+400,0,xGraph+60,yGraph+40,50,10);aySlider.setDecimalPrecision(0);aySlider.setLabel("");
  azSlider   = controlP5.addSlider("azSlider",-400,+400,0,xGraph+60,yGraph+70,50,10);azSlider.setDecimalPrecision(0);azSlider.setLabel("");
  gxSlider   = controlP5.addSlider("gxSlider",-500,+500,0,xGraph+60,yGraph+100,50,10);gxSlider.setDecimalPrecision(0);gxSlider.setLabel("");
  gySlider   = controlP5.addSlider("gySlider",-500,+500,0,xGraph+60,yGraph+130,50,10);gySlider.setDecimalPrecision(0);gySlider.setLabel("");
  gzSlider   = controlP5.addSlider("gzSlider",-500,+500,0,xGraph+60,yGraph+160,50,10);gzSlider.setDecimalPrecision(0);gzSlider.setLabel("");
  baroSlider = controlP5.addSlider("baroSlider",-300,+300,0,xGraph+60,yGraph+190,50,10);baroSlider.setDecimalPrecision(0);baroSlider.setLabel("");
  magSlider  = controlP5.addSlider("magSlider",-200,+200,0,xGraph+60,yGraph+220,50,10);magSlider.setDecimalPrecision(0);magSlider.setLabel("");
  magxSlider  = controlP5.addSlider("magxSlider",-500,+500,0,xGraph+190,yGraph+220,50,10);magxSlider.setDecimalPrecision(0);magxSlider.setLabel("");
  magySlider  = controlP5.addSlider("magySlider",-500,+500,0,xGraph+320,yGraph+220,50,10);magySlider.setDecimalPrecision(0);magySlider.setLabel("");
  magzSlider  = controlP5.addSlider("magzSlider",-500,+500,0,xGraph+450,yGraph+220,50,10);magzSlider.setDecimalPrecision(0);magzSlider.setLabel("");

  confP[ROLL] = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("confP_ROLL",0,xParam+40,yParam+20,30,14));confP[ROLL].setDecimalPrecision(1);confP[ROLL].setMultiplier(0.1);confP[ROLL].setMax(20);
  confI[ROLL] = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("confI_ROLL",0,xParam+75,yParam+20,40,14));confI[ROLL].setDecimalPrecision(3);confI[ROLL].setMultiplier(0.001);confI[ROLL].setMax(0.250);
  confD[ROLL] = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("confD_ROLL",0,xParam+120,yParam+20,30,14));confD[ROLL].setDecimalPrecision(0);confD[ROLL].setMultiplier(1);confD[ROLL].setMax(50);

  confP[PITCH] = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("confP_PITCH",0,xParam+40,yParam+40,30,14));confP[PITCH].setDecimalPrecision(1);confP[PITCH].setMultiplier(0.1);confP[PITCH].setMax(20);
  confI[PITCH] = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("confI_PITCH",0,xParam+75,yParam+40,40,14));confI[PITCH].setDecimalPrecision(3);confI[PITCH].setMultiplier(0.001);confI[PITCH].setMax(0.250);
  confD[PITCH] = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("confD_PITCH",0,xParam+120,yParam+40,30,14));confD[PITCH].setDecimalPrecision(0);confD[PITCH].setMultiplier(1);confD[PITCH].setMax(50);

  confP[YAW] = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("confP_YAW",0,xParam+40,yParam+60,30,14));confP[YAW].setDecimalPrecision(1);confP[YAW].setMultiplier(0.1);confP[YAW].setMax(20);
  confI[YAW] = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("confI_YAW",0,xParam+75,yParam+60,40,14));confI[YAW].setDecimalPrecision(3);confI[YAW].setMultiplier(0.001);confI[YAW].setMax(0.250);
  confD[YAW] = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("confD_YAW",0,xParam+120,yParam+60,30,14));confD[YAW].setDecimalPrecision(0);confD[YAW].setMultiplier(1);confD[YAW].setMax(50);

  confP[ALT] = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("confP_ALT",0,xParam+40,yParam+80,30,14));confP[ALT].setDecimalPrecision(1);confP[ALT].setMultiplier(0.1);confP[ALT].setMax(20);
  confI[ALT] = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("confI_ALT",0,xParam+75,yParam+80,40,14));confI[ALT].setDecimalPrecision(3);confI[ALT].setMultiplier(0.001);confI[ALT].setMax(0.250);
  confD[ALT] = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("confD_ALT",0,xParam+120,yParam+80,30,14));confD[ALT].setDecimalPrecision(0);confD[ALT].setMultiplier(1);confD[ALT].setMax(50);

  confP[LEVEL] = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("confP_LEVEL",0,xParam+40,yParam+100,30,14));confP[LEVEL].setDecimalPrecision(1);confP[LEVEL].setMultiplier(0.1);confP[LEVEL].setMax(25);
  confI[LEVEL] = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("confI_LEVEL",0,xParam+75,yParam+100,40,14));confI[LEVEL].setDecimalPrecision(3);confI[LEVEL].setMultiplier(0.001);confI[LEVEL].setMax(0.250);

  confP[MAG] = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("confP_MAG",0,xParam+40,yParam+120,30,14));confP[MAG].setDecimalPrecision(1);confP[MAG].setMultiplier(0.1);confP[MAG].setMax(8);

  for(int i=0;i<6;i++) {confP[i].setColorBackground(red_);confP[i].setMin(0);confP[i].setDirection(Controller.HORIZONTAL);}
  for(int i=0;i<5;i++) {confI[i].setColorBackground(red_);confI[i].setMin(0);confI[i].setDirection(Controller.HORIZONTAL);}
  for(int i=0;i<4;i++) {confD[i].setColorBackground(red_);confD[i].setMin(0);confD[i].setDirection(Controller.HORIZONTAL);}

  rollPitchRate = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("rollPitchRate",0,xParam+160,yParam+30,30,14));rollPitchRate.setDecimalPrecision(2);rollPitchRate.setMultiplier(0.01);
  rollPitchRate.setDirection(Controller.HORIZONTAL);rollPitchRate.setMin(0);rollPitchRate.setMax(1);rollPitchRate.setColorBackground(red_);
  yawRate = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("yawRate",0,xParam+160,yParam+60,30,14));yawRate.setDecimalPrecision(2);yawRate.setMultiplier(0.01);
  yawRate.setDirection(Controller.HORIZONTAL);yawRate.setMin(0);yawRate.setMax(1);yawRate.setColorBackground(red_); 
  dynamic_THR_PID = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("dynamic_THR_PID",0,xParam+300,yParam+12,30,14));dynamic_THR_PID.setDecimalPrecision(2);dynamic_THR_PID.setMultiplier(0.01);
  dynamic_THR_PID.setDirection(Controller.HORIZONTAL);dynamic_THR_PID.setMin(0);dynamic_THR_PID.setMax(1);dynamic_THR_PID.setColorBackground(red_);

  confRC_RATE = controlP5.addNumberbox("RC RATE",1,xParam+5,yParam+155,40,14);confRC_RATE.setDecimalPrecision(2);confRC_RATE.setMultiplier(0.02);
  confRC_RATE.setDirection(Controller.HORIZONTAL);confRC_RATE.setMin(0);confRC_RATE.setMax(5);confRC_RATE.setColorBackground(red_);
  confRC_EXPO = controlP5.addNumberbox("RC EXPO",0,xParam+5,yParam+185,40,14);confRC_EXPO.setDecimalPrecision(2);confRC_EXPO.setMultiplier(0.01);
  confRC_EXPO.setDirection(Controller.HORIZONTAL);confRC_EXPO.setMin(0);confRC_EXPO.setMax(1);confRC_EXPO.setColorBackground(red_);

  checkbox[BOXACC] = controlP5.addCheckBox("cbBOXACC",xParam+220,yParam+120);
  checkbox[BOXBARO] = controlP5.addCheckBox("cbBOXARO",xParam+220,yParam+135);
  checkbox[BOXMAG] = controlP5.addCheckBox("cbBOXMAG",xParam+220,yParam+150);
  checkbox[BOXCAMSTAB] = controlP5.addCheckBox("cbBOXCAMSTAB",xParam+220,yParam+165);
  checkbox[BOXCAMTRIG] = controlP5.addCheckBox("cbBOXCAMTRIG",xParam+220,yParam+180);
  checkbox[BOXARM] = controlP5.addCheckBox("cbBOXARM",xParam+220,yParam+195);

  for(int i=0;i<6;i++) {
    checkbox[i].setColorActive(color(255));checkbox[i].setColorBackground(color(120));
    checkbox[i].setItemsPerRow(6);checkbox[i].setSpacingColumn(10);
    checkbox[i].setLabel("");
    hideLabel(checkbox[i].addItem(i + "1",1));hideLabel(checkbox[i].addItem(i + "2",2));hideLabel(checkbox[i].addItem(i + "3",3));
    hideLabel(checkbox[i].addItem(i + "4",4));hideLabel(checkbox[i].addItem(i + "5",5));hideLabel(checkbox[i].addItem(i + "6",6));
  }


  buttonREAD =      controlP5.addButton("READ",1,xParam+5,yParam+225,60,16);buttonREAD.setColorBackground(red_);
  buttonWRITE =     controlP5.addButton("WRITE",1,xParam+290,yParam+225,60,16);buttonWRITE.setColorBackground(red_);
  buttonCALIBRATE_ACC = controlP5.addButton("CALIB_ACC",1,xParam+210,yParam+225,70,16);buttonCALIBRATE_ACC.setColorBackground(red_);
  buttonCALIBRATE_MAG = controlP5.addButton("CALIB_MAG",1,xParam+130,yParam+225,70,16);buttonCALIBRATE_MAG.setColorBackground(red_);

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
  float val,inter,a,b,h;
 
  background(80);
  textFont(font15);
  text("MultiWii conf",0,16);text("v1.dev", 0, 32);
  text("Cycle Time:",230,285);text(cycleTime, 330, 285);

  textFont(font12);
  text("Power:",xGraph-5,yGraph-30); text(pMeterSum,xGraph+50,yGraph-30);
  text("pAlarm:",xGraph-5,yGraph-15);  //text(intPowerTrigger,xGraph+50,yGraph-15);
  text("Volt:",xGraph-5,yGraph-2);  text(bytevbat/10.0,xGraph+50,yGraph-2);

  time1=millis();
  if (init_com==1) {
    if  (g_serial.available() >120) g_serial.clear(); // was 100, why?
    while (g_serial.available() >95) processSerialData(); // why 73?
    if ((time1-time2)>100 && graph_on==1 && g_serial.available()<90) { // why 60? - new values as suggessted by ziss_dm
      g_serial.write('M');
      time2=time1;
    }
  }
  
  axSlider.setValue(ax);aySlider.setValue(ay);azSlider.setValue(az);
  gxSlider.setValue(gx);gySlider.setValue(gy);gzSlider.setValue(gz);
  baroSlider.setValue(baro);
  magSlider.setValue(mag);
  magxSlider.setValue(magx);magySlider.setValue(magy);magzSlider.setValue(magz);

  motSliderV0.setValue(mot[0]);motSliderV1.setValue(mot[1]);motSliderV2.setValue(mot[2]);
  motSliderV3.setValue(mot[3]);motSliderV4.setValue(mot[4]);motSliderV5.setValue(mot[5]);

  servoSliderH1.setValue(servo0);servoSliderH2.setValue(servo1);servoSliderH3.setValue(servo2);servoSliderH4.setValue(servo3);
  servoSliderV0.setValue(servo0);servoSliderV1.setValue(servo1);servoSliderV2.setValue(servo2);

  rcStickThrottleSlider.setValue(rcThrottle);rcStickRollSlider.setValue(rcRoll);rcStickPitchSlider.setValue(rcPitch);rcStickYawSlider.setValue(rcYaw);
  rcStickAUX1Slider.setValue(rcAUX1);rcStickAUX2Slider.setValue(rcAUX2);rcStickCAM1Slider.setValue(rcCAM1);rcStickCAM2Slider.setValue(rcCAM2);

  stroke(255); 
  a=radians(angx);
  b=radians(angy);
  h=radians(mag);

  float size = 30.0;

  pushMatrix();
  camera(xObj,yObj,300/tan(PI*60.0/360.0),xObj/2+30,yObj/2-40,0,0,1,0);
  translate(xObj,yObj);
  directionalLight(200,200,200, 0, 0, -1);
  rotateZ(h);rotateX(b);rotateY(a);
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
  } else if (multiType == 11) { //OCTOX8
    noLights();
    textFont(font12);
    text("OCTOCOPTER X8", -45,-50);camera();popMatrix();

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
  stroke(50, 100, 150); if (magxGraph)  g_graph.drawLine(magxData, -500, +500);
  stroke(100, 50, 150); if (magyGraph)  g_graph.drawLine(magyData, -500, +500);
  stroke(150, 100, 50); if (magzGraph)  g_graph.drawLine(magzData, -500, +500);

  strokeWeight(2);
  stroke(255, 0, 0);     line(xGraph+25, yGraph+10, xGraph+60, yGraph+10);
  stroke(0, 255, 0);     line(xGraph+25, yGraph+40, xGraph+60, yGraph+40);
  stroke(0, 0, 255);     line(xGraph+25, yGraph+70, xGraph+60, yGraph+70);
  stroke(200, 200, 0);   line(xGraph+25, yGraph+100, xGraph+60, yGraph+100);
  stroke(0, 255, 255);   line(xGraph+25, yGraph+130, xGraph+60, yGraph+130);
  stroke(255, 0, 255);   line(xGraph+25, yGraph+160, xGraph+60, yGraph+160);
  stroke(125, 125, 125); line(xGraph+25, yGraph+190, xGraph+60, yGraph+190);
  stroke(225, 225, 125); line(xGraph+25, yGraph+220, xGraph+60, yGraph+220);
  stroke(50, 100, 150);  line(xGraph+155, yGraph+220, xGraph+190, yGraph+220);
  stroke(100, 50, 150);  line(xGraph+285, yGraph+220, xGraph+320, yGraph+220);
  stroke(150, 100, 50);  line(xGraph+415, yGraph+220, xGraph+450, yGraph+220);
  fill(0, 0, 0);

  strokeWeight(3);
  stroke(0);
  rectMode(CORNERS);
  rect(xMot-5,yMot-20, xMot+145, yMot+150);
  rect(xRC-5,yRC-5, xRC+185, yRC+235);
  rect(xParam,yParam, xParam+355, yParam+245);

  int xSens       = xParam + 70;
  int ySens       = yParam + 160;
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
  text("ROLL",xParam+3,yParam+32);text("PITCH",xParam+3,yParam+52);text("YAW",xParam+3,yParam+72);
  text("ALT",xParam+3,yParam+92);
  text("LEVEL",xParam+1,yParam+112);
  text("MAG",xParam+3,yParam+132); 
  text("Throttle PID",xParam+220,yParam+15);text("attenuation",xParam+220,yParam+30);

  text("AUX1",xParam+235,yParam+100);text("AUX2",xParam+295,yParam+100);
  text("LEVEL",xParam+180,yParam+130);
  text("BARO",xParam+180,yParam+145);
  text("MAG",xParam+180,yParam+160);
  text("ARM",xParam+180,yParam+205);
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
void HEAD(boolean theFlag) {magGraph = theFlag;}
void MAGX(boolean theFlag) {magxGraph = theFlag;}
void MAGY(boolean theFlag) {magyGraph = theFlag;}
void MAGZ(boolean theFlag) {magzGraph = theFlag;}

public void controlEvent(ControlEvent theEvent) {
  if (theEvent.isGroup())
    if (theEvent.name()=="portComList") InitSerial(theEvent.group().value()); // initialize the serial port selected
}

public void bSTART() {
  if(graphEnable == false) {return;}
  graph_on=1;
  readEnable = true;calibrateEnable = true;
  buttonREAD.setColorBackground(green_);
  buttonCALIBRATE_ACC.setColorBackground(green_);
  buttonCALIBRATE_MAG.setColorBackground(green_);
  g_serial.clear();
}

public void bSTOP() {
  graph_on=0;
}

public void READ() {
  if(readEnable == false) {return;}
  for(int i=0;i<4;i++) {
    confP[i].setValue(byteP[i]/10.0);confI[i].setValue(byteI[i]/1000.0);confD[i].setValue(byteD[i]);
  }
  confP[LEVEL].setValue(byteP[LEVEL]/10.0);confI[LEVEL].setValue(byteI[LEVEL]/1000.0);
  confP[MAG].setValue(byteP[MAG]/10.0);
  confRC_RATE.setValue(byteRC_RATE/50.0);
  confRC_EXPO.setValue(byteRC_EXPO/100.0);
  rollPitchRate.setValue(byteRollPitchRate/100.0);
  yawRate.setValue(byteYawRate/100.0);

  dynamic_THR_PID.setValue(byteDynThrPID/100.0);

  buttonWRITE.setColorBackground(green_);

  for(int i=0;i<6;i++) {confP[i].setColorBackground(green_);}
  for(int i=0;i<5;i++) {confI[i].setColorBackground(green_);}
  for(int i=0;i<4;i++) {confD[i].setColorBackground(green_);}
  
  confRC_RATE.setColorBackground(green_);
  confRC_EXPO.setColorBackground(green_);
  rollPitchRate.setColorBackground(green_);
  yawRate.setColorBackground(green_);
  dynamic_THR_PID.setColorBackground(green_);

  for(int i=0;i<6;i++) {
    if ((byte(activation[i])&32) >0) checkbox[i].activate(5); else checkbox[i].deactivate(5);if ((byte(activation[i])&16) >0) checkbox[i].activate(4); else checkbox[i].deactivate(4);
    if ((byte(activation[i])&8) >0) checkbox[i].activate(3); else checkbox[i].deactivate(3);if ((byte(activation[i])&4) >0) checkbox[i].activate(2); else checkbox[i].deactivate(2);
    if ((byte(activation[i])&2) >0) checkbox[i].activate(1); else checkbox[i].deactivate(1);if ((byte(activation[i])&1) >0) checkbox[i].activate(0); else checkbox[i].deactivate(0);
  }
  
  confPowerTrigger.setValue(intPowerTrigger);
  writeEnable = true;
  
}

public void WRITE() {
  if(writeEnable == false) {return;}

  for(int i=0;i<6;i++) {byteP[i] = (round(confP[i].value()*10));}
  for(int i=0;i<5;i++) {byteI[i] = (round(confI[i].value()*1000));}
  for(int i=0;i<4;i++) {byteD[i] = (round(confD[i].value()));}

  byteRC_RATE = (round(confRC_RATE.value()*50));
  byteRC_EXPO = (round(confRC_EXPO.value()*100));
  byteRollPitchRate = (round(rollPitchRate.value()*100));
  byteYawRate = (round(yawRate.value()*100));
  byteDynThrPID = (round(dynamic_THR_PID.value()*100));

  for(int i=0;i<6;i++) {
    activation[i] = (int)(checkbox[i].arrayValue()[0]+checkbox[i].arrayValue()[1]*2+checkbox[i].arrayValue()[2]*4
                              +checkbox[i].arrayValue()[3]*8+checkbox[i].arrayValue()[4]*16+checkbox[i].arrayValue()[5]*32);
  }

  intPowerTrigger = (round(confPowerTrigger.value()));

  int[] s = new int[32];
  int p = 0;
   s[p++] = 'W'; //0 write to Eeprom @ arduino
   for(int i=0;i<4;i++) {s[p++] = byteP[i];  s[p++] = byteI[i];  s[p++] =  byteD[i];}
   s[p++] = byteP[LEVEL]; s[p++] = byteI[LEVEL]; //14
   s[p++] = byteP[MAG]; //15
   s[p++] = byteRC_RATE; s[p++] = byteRC_EXPO; //17
   s[p++] = byteRollPitchRate; //18
   s[p++] = byteYawRate;
   s[p++] = byteDynThrPID;
   for(int i=0;i<6;i++) s[p++] = activation[i]; //26
   s[p++] = intPowerTrigger ;
   s[p++] = intPowerTrigger >>8 &0xff;
   for(int i =0;i<29;i++)    g_serial.write(char(s[i]));
}

public void CALIB_ACC() {
  if(calibrateEnable == false) {return;}
  g_serial.write('S'); // acc Sensor calibration request
}
public void CALIB_MAG() {
  if(calibrateEnable == false) {return;}
  g_serial.write('E'); // mag Sensor calibration request
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
  int frame_size = 100;
  int present=0,mode=0;
  
  if (g_serial.read() == 'M') {
    g_serial.readBytes(inBuf);
    if (inBuf[frame_size-1] == 'M') {  // Multiwii @ arduino send all data to GUI
      p=0;
      read8(); //version
      ax = read16();ay = read16();az = read16();
      gx = read16();gy = read16();gz = read16();
      magx = read16();magy = read16();magz = read16();
      baro = read16();
      mag = read16();
      servo0 = read16();servo1 = read16();servo2 = read16();servo3 = read16();
      for(int i=0;i<6;i++) mot[i] = read16();
      rcRoll = read16();rcPitch = read16();rcYaw = read16();rcThrottle = read16();
      rcAUX1 = read16();rcAUX2 = read16();rcCAM1 = read16();rcCAM2 = read16();
      present = read8();
      mode = read8();
      cycleTime = read16();
      angx = read16();angy = read16();
      multiType = read8();
      
      for(int i=0;i<4;i++) {byteP[i] = read8();byteI[i] = read8();byteD[i] = read8();}
      byteP[LEVEL] = read8();byteI[LEVEL] = read8();
      byteP[MAG] = read8();
      byteRC_RATE = read8();
      byteRC_EXPO = read8();
      byteRollPitchRate = read8();
      byteYawRate = read8();
      byteDynThrPID = read8();
      for(int i=0;i<6;i++) activation[i] = read8();

      pMeterSum = read16();
      intPowerTrigger = read16();
      bytevbat = read8();
      
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
      magxData.addVal(magx);
      magyData.addVal(magy);
      magzData.addVal(magz);
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



