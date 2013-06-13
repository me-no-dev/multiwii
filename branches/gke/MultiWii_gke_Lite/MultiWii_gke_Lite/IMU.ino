

// **************************************************
// Simplified IMU based on "Complementary Filter"
// Inspired by http://starlino.com/imu_guide.html
//
// adapted by ziss_dm : http://www.multiwii.com/forum/viewtopic.php?f=8&t=198
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
// **************************************************

//******  advanced users settings *******************
/* Set the Low Pass Filter factor for ACC
 Increasing this value would reduce ACC noise (visible in GUI), but would increase ACC lag time
 Comment this if  you do not want filter at all.
 unit = n power of 2 */
// this one is also used for ALT HOLD calculation, should not be changed
#if !defined(ACC_LPF_FACTOR)
//#define ACC_LPF_FACTOR 4 // that means a LPF of 16
#endif

/* Set the Gyro Weight for Gyro/Acc complementary filter
 Increasing this value would reduce and delay Acc influence on the output of the filter*/
#ifndef GYR_CMPF_FACTOR
#define GYR_CMPF_FACTOR 600
#endif

/* Set the Gyro Weight for Gyro/Magnetometer complementary filter
 Increasing this value would reduce and delay Magnetometer influence on the output of the filter*/
#define GYR_CMPFM_FACTOR 250

//****** end of advanced users settings *************
#define INV_GYR_CMPF_FACTOR   (1.0f / (GYR_CMPF_FACTOR  + 1.0f))
#define INV_GYR_CMPFM_FACTOR  (1.0f / (GYR_CMPFM_FACTOR + 1.0f))

#define GYRO_SCALE ((1998 * PI)/((32767.0f / 4.0f ) * 180.0f * 1000000.0f)) //MPU6050

// +-2000/sec deg scale
// !!!!should be adjusted to the rad/sec and be part defined in each gyro sensor

typedef struct fp_vector {		
  float X,Y,Z;		
} 
t_fp_vector_def;

typedef union {		
  float A[3];		
  t_fp_vector_def V;		
} 
t_fp_vector;

typedef struct int32_t_vector {
  int32_t X,Y,Z;
} 
t_int32_t_vector_def;

typedef union {
  int32_t A[3];
  t_int32_t_vector_def V;
} 
t_int32_t_vector;

int16_t _atan2(int32_t y, int32_t x){
  float z = (float)y / x;
  int16_t a;
  if ( abs(y) < abs(x) ){
    a = 573 * z / (1.0f + 0.28f * z * z);
    if (x<0) 
      if (y<0) a -= 1800;
      else a += 1800;
  } 
  else {
    a = 900 - 573 * z / (z * z + 0.28f);
    if (y<0) a -= 1800;
  }
  return a;
} // atan2

float invSqrt (float x){ 
  union{  
    int32_t i;  
    float   f; 
  } 
  conv; 
  conv.f = x; 
  conv.i = 0x5f3759df - (conv.i >> 1); 
  return 0.5f * conv.f * (3.0f - x * conv.f * conv.f);
} // invSqrt

// Rotate Estimated vector(s) with small angle approximation, according to the gyro data
void rotateV(struct fp_vector *v, float* delta) {
  fp_vector v_tmp = *v;
  v->Z -= delta[ROLL] * v_tmp.X + delta[PITCH] * v_tmp.Y;
  v->X += delta[ROLL] * v_tmp.Z - delta[YAW] * v_tmp.Y;
  v->Y += delta[PITCH] * v_tmp.Z + delta[YAW] * v_tmp.X;
} // rotateV

static float invG; // 1/|G|

static t_fp_vector EstG;
static t_int32_t_vector EstG32;

int16_t inline accFilter(uint8_t axis, int32_t a) {
#if defined(ACC_LPF_FACTOR)
  static int32_t F[3] = {
    0,0,512  };

  F[axis] -= F[axis] >> ACC_LPF_FACTOR;
  F[axis] += a;

  return ( F[axis] >> ACC_LPF_FACTOR); 
#else
  return (a);
#endif
} // AccFilter

void getEstimatedAttitude(void){
  uint8_t axis;
  int32_t accMagSq;
  float scale, deltaGyroAngle[3];
  int32_t sqGZ, sqGX, sqGY, sqGX_sqGZ;
  float invmagXZ;
  static uint32_t PrevuS;
  uint32_t NowuS = micros();

  scale = (NowuS - PrevuS) * GYRO_SCALE;
  PrevuS = NowuS;

  accMagSq = 0;
  for (axis = 0; axis < 3; axis++) {
    deltaGyroAngle[axis] = (float)gyroADC[axis] * scale; 
    accSmooth[axis] = accFilter(axis, accADC[axis]);
    accMagSq += sq((int32_t)accSmooth[axis]);
  }
  accMagSq = (accMagSq * 100) / sq((int32_t)acc_1G);

  rotateV(&EstG.V, deltaGyroAngle);

  f.SMALL_ANGLES_25 = (abs(accSmooth[ROLL])<acc_25deg) && (abs(accSmooth[PITCH])<acc_25deg) && (accSmooth[YAW] > 0);

  // To do that, we just skip filter, as EstV already rotated by Gyro
  if ((accMagSq > 72) && (accMagSq < 133)) // >1.15G or <0.85G 
    for (axis = 0; axis < 3; axis++) 
      EstG.A[axis] = (EstG.A[axis] * GYR_CMPF_FACTOR + accSmooth[axis]) * INV_GYR_CMPF_FACTOR;

  for (axis = 0; axis < 3; axis++)
    EstG32.A[axis] = EstG.A[axis]; //int32_t cross calculation is a little bit faster than float	

  // Attitude of the estimated vector
  sqGZ = sq(EstG32.V.Z);
  sqGX = sq(EstG32.V.X);
  sqGY = sq(EstG32.V.Y);
  sqGX_sqGZ = sqGX + sqGZ;
  invmagXZ  = invSqrt(sqGX_sqGZ);
  invG = invSqrt(sqGX_sqGZ + sqGY);

  angle[ROLL]  = _atan2(EstG32.V.X , EstG32.V.Z);
  angle[PITCH] = _atan2(EstG32.V.Y , invmagXZ * sqGX_sqGZ);

} // getEstimatedAttitude


