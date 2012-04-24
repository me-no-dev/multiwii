
/********************************************************************/
/************************* MPU6050 Settings *************************/
/********************************************************************/

#define MPU6050_ADDRESS 0xD0 // default
//#define MPU6050_ADDRESS 0xD2


/* MPU6050 Low pass filter setting. In case you cannot eliminate all vibrations to the Gyro, you can try
   to decrease the LPF frequency, only one step per try. As soon as twitching gone, stick with that setting.
   It will not help on feedback wobbles, so change only when copter is randomly twiching and all dampening and
   balancing options ran out. Uncomment only one option!
   IMPORTANT! Change low pass filter setting changes PID behaviour, so retune your PID's after changing LPF.*/
#define MPU6050_LPF_256HZ
//#define MPU6050_LPF_188HZ
//#define MPU6050_LPF_98HZ
//#define MPU6050_LPF_42HZ
//#define MPU6050_LPF_20HZ
//#define MPU6050_LPF_10HZ      // Use this only in extreme cases, rather change motors and/or props


// for the MPU6050 ES (samples) with acc 1g 255
//#define MPU6050ES







/********************************************************************/
/********************* MPU6050 defines ******************************/
/********************************************************************/

#if defined(MPU6050_LPF_256HZ) || defined(MPU6050_LPF_188HZ) || defined(MPU6050_LPF_98HZ) || defined(MPU6050_LPF_42HZ) || defined(MPU6050_LPF_20HZ) || defined(MPU6050_LPF_10HZ)
  #if defined(MPU6050_LPF_256HZ)
    #define MPU6050_SMPLRT_DIV 0  //8000Hz
    #define MPU6050_DLPF_CFG   0
  #endif
  #if defined(MPU6050_LPF_188HZ)
    #define MPU6050_SMPLRT_DIV 0  //1000Hz
    #define MPU6050_DLPF_CFG   1
  #endif
  #if defined(MPU6050_LPF_98HZ)
    #define MPU6050_SMPLRT_DIV 0
    #define MPU6050_DLPF_CFG   2
  #endif
  #if defined(MPU6050_LPF_42HZ)
    #define MPU6050_SMPLRT_DIV 0
    #define MPU6050_DLPF_CFG   3
  #endif
  #if defined(MPU6050_LPF_20HZ)
    #define MPU6050_SMPLRT_DIV 0
    #define MPU6050_DLPF_CFG   4
  #endif
  #if defined(MPU6050_LPF_10HZ)
    #define MPU6050_SMPLRT_DIV 0
    #define MPU6050_DLPF_CFG   5
  #endif
#endif

#define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -Y; accADC[PITCH]  =  X; accADC[YAW]  =  Z;}
#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] = -X; gyroADC[PITCH] = -Y; gyroADC[YAW] = -Z;}
#undef INTERNAL_I2C_PULLUPS
#define ACC 1
#define GYRO 1
