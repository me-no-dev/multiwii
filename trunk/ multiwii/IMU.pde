uint8_t rawIMU(uint8_t withACC) { //if the WMP or NK are oriented differently, it can be changed here
  if (withACC) {
    #if defined(I2C_ACC)
      i2c_ACC_getADC();
    #endif
    #if defined(ADCACC)
      adc_ACC_getRawADC();
    #endif  
  }
  #if defined(ITG3200) || defined(L3G4200D)
    i2c_Gyro_getADC();
    return 1;
  #else
    i2c_WMP_getRawADC();
    if ( (rawADC_WMP[5]&0x02) == 0x02 && (rawADC_WMP[5]&0x01) == 0 ) {// motion plus data
      gyroADC[ROLL]   = - ( ((rawADC_WMP[5]>>2)<<8) + rawADC_WMP[2] );
      gyroADC[PITCH]  = - ( ((rawADC_WMP[4]>>2)<<8) + rawADC_WMP[1] );
      gyroADC[YAW]    = - ( ((rawADC_WMP[3]>>2)<<8) + rawADC_WMP[0] );
      return 1;
    } else if ( (rawADC_WMP[5]&0x02) == 0 && (rawADC_WMP[5]&0x01) == 0) { //nunchuk data
      #if defined(I2C_ACC) || defined(ADCACC)
        return 2;
      #else
        accADC[ROLL]  =   ( (rawADC_WMP[3]<<2)        + ((rawADC_WMP[5]>>4)&0x2) );
        accADC[PITCH] = - ( (rawADC_WMP[2]<<2)        + ((rawADC_WMP[5]>>3)&0x2) );
        accADC[YAW]   = - ( ((rawADC_WMP[4]&0xFE)<<2) + ((rawADC_WMP[5]>>5)&0x6) );
        return 0;
      #endif
    } else
      return 2;
  #endif
}

uint8_t updateIMU(uint8_t withACC) {
  static int32_t g[3];
  static int32_t a[3];
  uint8_t axis;
  static int16_t previousGyroADC[3] = {0,0,0};
  uint8_t r;
  r=rawIMU(withACC);
  
  if (currentTime < (neutralizeTime + NEUTRALIZE_DELAY)) {//we neutralize data in case of blocking+hard reset state
    for (axis = 0; axis < 3; axis++) {gyroADC[axis]=0;accADC[axis]=0;}
    accADC[YAW] = acc_1G;
  } else {
    if (r == 1) { //gyro
      if (calibratingG>0) {
        for (axis = 0; axis < 3; axis++) {
          if (calibratingG>1) {
            if (calibratingG == 400) g[axis]=0;
            g[axis] +=gyroADC[axis];
            gyroADC[axis]=0;
          } else {
            gyroZero[axis]=(g[axis]+200)/399;
            blinkLED(10,15,1+3*nunchukPresent);
          }
        }
        calibratingG--;
      }
      #if defined(ITG3200) || defined(L3G4200D)
        gyroADC[ROLL]  = gyroADC[ROLL]  - gyroZero[ROLL];
        gyroADC[PITCH] = gyroADC[PITCH] - gyroZero[PITCH];
        gyroADC[YAW]   = gyroADC[YAW]   - gyroZero[YAW];
      #else
        gyroADC[ROLL]  = gyroADC[ROLL]  - gyroZero[ROLL];
        gyroADC[PITCH] = gyroADC[PITCH] - gyroZero[PITCH];
        gyroADC[YAW]   = gyroADC[YAW]   - gyroZero[YAW];
        gyroADC[ROLL]  = (rawADC_WMP[3]&0x01)     ? gyroADC[ROLL]/5  : gyroADC[ROLL] ;   //the ratio 1/5 is not exactly the IDG600 or ISZ650 specification 
        gyroADC[PITCH] = (rawADC_WMP[4]&0x02)>>1  ? gyroADC[PITCH]/5 : gyroADC[PITCH] ;  //we detect here the slow of fast mode WMP gyros values (see wiibrew for more details)
        gyroADC[YAW]   = (rawADC_WMP[3]&0x02)>>1  ? gyroADC[YAW]/5   : gyroADC[YAW] ;
      #endif
      //anti gyro glitch, limit the variation between two consecutive readings
      for (axis = 0; axis < 3; axis++) {
        gyroADC[axis] = constrain(gyroADC[axis],previousGyroADC[axis]-100,previousGyroADC[axis]+100);
        previousGyroADC[axis] = gyroADC[axis];
      }
    }
    if (r == 0 || ( (accPresent == 1) && (withACC == 1) ) ) { //nunchuk or i2c ACC
      if (calibratingA>0) {
        if (calibratingA>1) {
          for (uint8_t axis = 0; axis < 3; axis++) {
            if (calibratingA == 400) a[axis]=0;
            a[axis] +=accADC[axis];
            accADC[axis]=0;
          }
        } else {
          accZero[ROLL]  = (a[ROLL]+200)/399;
          accZero[PITCH] = (a[PITCH]+200)/399;
          accZero[YAW]   = (a[YAW]+200)/399+acc_1G; // for nunchuk 200=1G
          writeParams(); // write accZero in EEPROM
        }
        calibratingA--;
      } else {
        accADC[ROLL]  =    accADC[ROLL]  - accZero[ROLL] ;
        accADC[PITCH] =    accADC[PITCH] - accZero[PITCH];
        accADC[YAW]   = - (accADC[YAW]   - accZero[YAW]) ;
      }
    }
  }  
  return r;
}

void computeIMU () {
  uint8_t axis;
  static int16_t gyroADCprevious[3] = {0,0,0};
  static int16_t gyroADCp[3] = {0,0,0};
  int16_t gyroADCinter[3];
  static int16_t lastAccADC[3] = {0,0,0};
  static int16_t similarNumberAccData[3];
  static int16_t gyroDeviation[3];
  static uint32_t timeInterleave;
  static int16_t gyroYawSmooth = 0;

  //we separate the 2 situations because reading gyro values with a gyro only setup can be acchieved at a higher rate
  //gyro+nunchuk: we must wait for a quite high delay betwwen 2 reads to get both WM+ and Nunchuk data. It works with 3ms
  //gyro only: the delay to read 2 consecutive values can be reduced to only 0.65ms
  if (nunchukPresent) {
    annexCode();
    while((micros()-timeInterleave)<INTERLEAVING_DELAY) ; //interleaving delay between 2 consecutive reads
    timeInterleave=micros();
    updateIMU(0);
    getEstimatedAttitude(); // computation time must last less than one interleaving delay
    while((micros()-timeInterleave)<INTERLEAVING_DELAY) ; //interleaving delay between 2 consecutive reads
    timeInterleave=micros();
    while(updateIMU(0) != 1) ; // For this interleaving reading, we must have a gyro update at this point (less delay)

    for (axis = 0; axis < 3; axis++) {
      // empirical, we take a weighted value of the current and the previous values
      gyroData[axis] = (gyroADC[axis]*3+gyroADCprevious[axis]+16)/4/8; // /4 is to average 4 values ; /8 is to reduce the sensibility of gyro
      gyroADCprevious[axis] = gyroADC[axis];
    }
  } else {
    #if defined(I2C_ACC) || defined(ADCACC)
      getEstimatedAttitude();
      updateIMU(1); //with I2C or ADC ACC
    #else
      updateIMU(0); //without ACC
    #endif
    for (axis = 0; axis < 3; axis++)
      gyroADCp[axis] =  gyroADC[axis];
    timeInterleave=micros();
    annexCode();
    while((micros()-timeInterleave)<650) ; //empirical, interleaving delay between 2 consecutive reads
    updateIMU(0); //without ACC
    for (axis = 0; axis < 3; axis++) {
      gyroADCinter[axis] =  gyroADC[axis]+gyroADCp[axis];
      // empirical, we take a weighted value of the current and the previous values
      gyroData[axis] = (gyroADCinter[axis]+gyroADCprevious[axis]+12)/3/8; // /3 is to average 3 values ; /8 is to reduce the sensibility of gyro
      gyroADCprevious[axis] = gyroADCinter[axis]/2;
      #if not defined (I2C_ACC) && not defined (ADCACC)
        accADC[axis]=0;
      #endif
    }
  }
  #if defined(TRI)
    gyroData[YAW] = (gyroYawSmooth*2+gyroData[YAW]+1)/3;
    gyroYawSmooth = gyroData[YAW];
  #endif
}


// **************************************************
// Simplified IMU based on "Complementary Filter"
// Inspired by http://starlino.com/imu_guide.html
//
// The following ideas was used in this project:
// 1) Rotation matrix: http://en.wikipedia.org/wiki/Rotation_matrix
// 2) Small-angle approximation: http://en.wikipedia.org/wiki/Small-angle_approximation
// 3) C. Hastings approximation for atan2()
// 4) Optimization tricks: http://www.hackersdelight.org/
//
// Currently Magnetometer uses separate CF which is used only
// for heading approximation.
//
// Last Modified: 19/06/2011
// Version: V1.1   
// **************************************************

//******  advanced users settings *******************
/* Set the Low Pass Filter factor for ACC */
/* Increasing this value would reduce ACC noise (visible in GUI), but would increase ACC lag time*/
/* Comment this if  you do not want filter at all.*/
/* Default WMC value: 8*/
#define ACC_LPF_FACTOR 8

/* Set the Low Pass Filter factor for Magnetometer */
/* Increasing this value would reduce Magnetometer noise (not visible in GUI), but would increase Magnetometer lag time*/
/* Comment this if  you do not want filter at all.*/
/* Default WMC value: n/a*/
//#define MG_LPF_FACTOR 4

/* Set the Gyro Weight for Gyro/Acc complementary filter */
/* Increasing this value would reduce and delay Acc influence on the output of the filter*/
/* Default WMC value: 300*/
#define GYR_CMPF_FACTOR 310.0f

/* Set the Gyro Weight for Gyro/Magnetometer complementary filter */
/* Increasing this value would reduce and delay Magnetometer influence on the output of the filter*/
/* Default WMC value: n/a*/
#define GYR_CMPFM_FACTOR 200.0f

//****** end of advanced users settings *************

#define INV_GYR_CMPF_FACTOR   (1.0f / (GYR_CMPF_FACTOR  + 1.0f))
#define INV_GYR_CMPFM_FACTOR  (1.0f / (GYR_CMPFM_FACTOR + 1.0f))
#if defined(ITG3200) || defined(L3G4200D)
  //#define GYRO_SCALE ((2000.0f * PI)/((32767.0f / 4.0f ) * 180.0f * 1000000.0f) * 1.155f)  
  // +-2000/sec deg scale
  #define GYRO_SCALE ((200.0f * PI)/((32768.0f / 5.0f / 4.0f ) * 180.0f * 1000000.0f) * 1.5f)     
  // +- 200/sec deg scale
  // 1.5 is emperical, not sure what it means
  // should be in rad/sec
#else
  #define GYRO_SCALE (1.0f/200e6f)
  // empirical, depends on WMP on IDG datasheet, tied of deg/ms sensibility
  // !!!!should be adjusted to the rad/sec
#endif 
// Small angle approximation
#define ssin(val) (val)
#define scos(val) 1.0f

typedef struct {
  float X;
  float Y;
  float Z;
} t_fp_vector_def;

typedef union {
  float   A[3];
  t_fp_vector_def V;
} t_fp_vector;

int16_t _atan2(float y, float x){
  #define fp_is_neg(val) ((((byte*)&val)[3] & 0x80) != 0)
  float z = y / x;
  int16_t zi = abs(int16_t(z * 100)); 
  int8_t y_neg = fp_is_neg(y);
  if ( zi < 100 ){
    if (zi > 10) 
     z = z / (1.0f + 0.28f * z * z);
   if (fp_is_neg(x)) {
     if (y_neg) z -= PI;
     else z += PI;
   }
  } else {
   z = (PI / 2.0f) - z / (z * z + 0.28f);
   if (y_neg) z -= PI;
  }
  z *= (180.0f / PI * 10); 
  return z;
}

void getEstimatedAttitude(){
  uint8_t axis;
  int16_t  AccMag = 0;
  static t_fp_vector GEstG = {0,0,200};
  t_fp_vector EstG = GEstG;
  static t_fp_vector EstM = {10,10,200};
  float deltaGyroAngle;
  static uint16_t PreviousTime;
  static int16_t mgSmooth[3];  //projection of smoothed and normalized magnetic vector on x/y/z axis, as measured by magnetometer
  uint16_t CurrentTime  = micros();
  float deltaTime = (CurrentTime - PreviousTime) * GYRO_SCALE;
  PreviousTime = CurrentTime;
  // Initialization
  for (axis = 0; axis < 3; axis++) {
    #if defined(ACC_LPF_FACTOR)
      // LPF for ACC values
      accSmooth[axis] = (accSmooth[axis] * (ACC_LPF_FACTOR - 1) + accADC[axis]) / ACC_LPF_FACTOR;
      AccMag += (accSmooth[axis] * 10 / acc_1G) * (accSmooth[axis] * 10 / acc_1G);
      #define ACC_VALUE accSmooth[axis]
    #else  
      accSmooth[axis] = accADC[axis];
      AccMag += (accADC[axis] * 10 / acc_1G) * (accADC[axis] * 10 / acc_1G);
      #define ACC_VALUE accADC[axis]
    #endif  
    #if defined(I2C_MAG) & defined(MG_LPF_FACTOR)
      // LPF for Magnetometer values
      mgSmooth[axis] = (mgSmooth[axis] * (MG_LPF_FACTOR - 1) + magADC[axis]) / MG_LPF_FACTOR;
      #define MAG_VALUE mgSmooth[axis]
    #else  
      #define MAG_VALUE magADC[axis]
    #endif 
  }
  // Rotate Estimated vector(s), ROLL
  deltaGyroAngle  = gyroADC[ROLL] * deltaTime;
  EstG.V.Z =  scos(deltaGyroAngle) * EstG.V.Z - ssin(deltaGyroAngle) * EstG.V.X;
  EstG.V.X =  ssin(deltaGyroAngle) * EstG.V.Z + scos(deltaGyroAngle) * EstG.V.X;
  #if defined(I2C_MAG)
    EstM.V.Z =  scos(deltaGyroAngle) * EstM.V.Z - ssin(deltaGyroAngle) * EstM.V.X;
    EstM.V.X =  ssin(deltaGyroAngle) * EstM.V.Z + scos(deltaGyroAngle) * EstM.V.X;
  #endif 
  // Rotate Estimated vector(s), PITCH
  deltaGyroAngle  = gyroADC[PITCH] * deltaTime;
  EstG.V.Y =  scos(deltaGyroAngle) * EstG.V.Y + ssin(deltaGyroAngle) * EstG.V.Z;
  EstG.V.Z = -ssin(deltaGyroAngle) * EstG.V.Y + scos(deltaGyroAngle) * EstG.V.Z;
  #if defined(I2C_MAG)
    EstM.V.Y =  scos(deltaGyroAngle) * EstM.V.Y + ssin(deltaGyroAngle) * EstM.V.Z;
    EstM.V.Z = -ssin(deltaGyroAngle) * EstM.V.Y + scos(deltaGyroAngle) * EstM.V.Z;
  #endif 
  // Rotate Estimated vector(s), YAW
  deltaGyroAngle  = gyroADC[YAW] * deltaTime;
  EstG.V.X =  scos(deltaGyroAngle) * EstG.V.X - ssin(deltaGyroAngle) * EstG.V.Y;
  EstG.V.Y =  ssin(deltaGyroAngle) * EstG.V.X + scos(deltaGyroAngle) * EstG.V.Y;
  #if defined(I2C_MAG)
    EstM.V.X =  scos(deltaGyroAngle) * EstM.V.X - ssin(deltaGyroAngle) * EstM.V.Y;
    EstM.V.Y =  ssin(deltaGyroAngle) * EstM.V.X + scos(deltaGyroAngle) * EstM.V.Y;
  #endif 
  // Apply complimentary filter (Gyro drift correction)
  // If accel magnitude >1.4G or <0.6G => we neutralize the effect of accelerometers in the angle estimation.
  // To do that, we just skip filter, as EstV already rotated by Gyro
  if (!((36 > AccMag) or (AccMag > 196))) {
    for (axis = 0; axis < 3; axis++)
      EstG.A[axis] = (EstG.A[axis] * GYR_CMPF_FACTOR + ACC_VALUE) * INV_GYR_CMPF_FACTOR;
  }
  // Attitude of the estimated vector  
  angle[ROLL]  =  _atan2(EstG.V.X, EstG.V.Z);
  angle[PITCH] =  _atan2(EstG.V.Y, EstG.V.Z);
  GEstG = EstG;
  #if defined(I2C_MAG)
    // Apply complimentary filter (Gyro drift correction)
    for (axis = 0; axis < 3; axis++)
      EstM.A[axis] = (EstM.A[axis] * GYR_CMPFM_FACTOR + MAG_VALUE) * INV_GYR_CMPFM_FACTOR;
    // Attitude of the cross product vector GxM
    heading = _atan2(EstG.V.Z * EstM.V.X - EstG.V.X * EstM.V.Z, EstG.V.Y * EstM.V.Z - EstG.V.Z * EstM.V.Y) / 10;
  #endif 
}


