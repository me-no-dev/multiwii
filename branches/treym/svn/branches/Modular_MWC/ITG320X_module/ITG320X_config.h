/********************************************************************/
/****              ITG3200 & ITG3205 config                      ****/
/********************************************************************/

// I2C Adress
#define ITG3200_ADDRESS 0XD0 // default
//#define ITG3200_ADDRESS 0XD2





/* ITG3200 & ITG3205 Low pass filter setting. In case you cannot eliminate all vibrations to the Gyro, you can try
   to decrease the LPF frequency, only one step per try. As soon as twitching gone, stick with that setting.
   It will not help on feedback wobbles, so change only when copter is randomly twiching and all dampening and
   balancing options ran out. Uncomment only one option!
   IMPORTANT! Change low pass filter setting changes PID behaviour, so retune your PID's after changing LPF.*/
#define ITG3200_LPF_256HZ     // default
//#define ITG3200_LPF_188HZ
//#define ITG3200_LPF_98HZ
//#define ITG3200_LPF_42HZ
//#define ITG3200_LPF_20HZ
//#define ITG3200_LPF_10HZ      // Use this only in extreme cases, rather change motors and/or props














/********************************************************************/
/****              ITG3200 & ITG3205 defines                     ****/
/********************************************************************/

#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] = X; gyroADC[PITCH] = Y; gyroADC[YAW] = Z;}

#define GYRO 1

#if defined(ITG3200_LPF_256HZ)
  #define ITG3200_SMPLRT_DIV 0  //8000Hz
  #define ITG3200_DLPF_CFG   0
#endif
#if defined(ITG3200_LPF_188HZ)
  #define ITG3200_SMPLRT_DIV 0  //1000Hz
  #define ITG3200_DLPF_CFG   1
#endif
#if defined(ITG3200_LPF_98HZ)
  #define ITG3200_SMPLRT_DIV 0
  #define ITG3200_DLPF_CFG   2
#endif
#if defined(ITG3200_LPF_42HZ)
  #define ITG3200_SMPLRT_DIV 0
  #define ITG3200_DLPF_CFG   3
#endif
#if defined(ITG3200_LPF_20HZ)
  #define ITG3200_SMPLRT_DIV 0
  #define ITG3200_DLPF_CFG   4
#endif
#if defined(ITG3200_LPF_10HZ)
  #define ITG3200_SMPLRT_DIV 0
  #define ITG3200_DLPF_CFG   5
#endif
