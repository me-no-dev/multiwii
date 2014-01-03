#include "def.h"

int16_t motor[NUMBER_MOTOR];
#if defined(SERVO)
int16_t servo[8] = {1500,1500,1500,1500,1500,1500,1500,1500};

#if defined(AIRPLANE)|| defined(HELICOPTER)
// To prevent motor to start at reset. atomicServo[7]=5 or 249 if reversed servo
volatile uint8_t atomicServo[8] = {125,125,125,125,125,125,125,5};
#else
volatile uint8_t atomicServo[8] = {125,125,125,125,125,125,125,125};
#endif

#endif

#if defined(HARMONISE)
uint8_t const DM[] = { 3, 2, 1, 0, 6, 7, 8, 9, 4, 5 }; // 4 & 5 always servo rate
#else
uint8_t const DM[] = { 0, 1, 2, 3, 6, 7, 8, 9, 4, 5 }; // 4 & 5 always servo rate
#endif // HARMONISE

uint8_t const SM[] = { 1, 2, 1, 4, 5, 2, 3, 0 };

int32_t map(float x, float in_min, float in_max, float out_min,
		float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void pwmSetPeriod(uint8 channel, uint32 period) {

	// This works by "timer" NOT timer channel so
	// the period of ALL pins using that timer are changed
	if (channel < CurrMaxPWMOutputs) {
		PWMPins[channel].Period = period;
		PWMPins[channel].Timer.Tim->ARR = PWMPins[channel].Period - 1;
	}
} // pwmSetPeriod

void pwmWrite(uint8 channel, uint16 v) {

	if (channel <= CurrMaxPWMOutputs) {
		v = Limit(v, PWMPins[channel].MinPW, PWMPins[channel].MaxPW);
		*PWMPins[channel].Timer.CCR = v;
	}
} // pwmWrite

void writeMotors(void) {
#if defined(FLYING_WING)
	pwmWrite(0, motor[0]);
#else
	uint8_t s;

	for (s = 0; s < NUMBER_MOTOR; s++)
		pwmWrite(DM[s], motor[s]);
#endif
}

void writeServos(void) {
#if defined(SERVO)
	uint8_t s;

	for (s = (PRI_SERVO_FROM-1); s < PRI_SERVO_TO; s++)
		pwmWrite(SM[s], servo[s]);
#endif
}

void writeAllMotors(uint16_t m) {
#if defined(FLYING_WING)
	pwmWrite(0, m);
#else
	uint8_t i;

	for (i = 0; i < NUMBER_MOTOR; i++)
		pwmWrite(DM[i], m);
#endif
}

void initializeServo(void) {
#if defined(SERVO)
	uint8_t s;

	for (s = (PRI_SERVO_FROM-1); s < PRI_SERVO_TO; s++)
		pwmWrite(SM[s], 1500);
#endif
}

void initOutput(void) {
	uint8_t i;
	//PWM0-3, 4-5, 6-10

	if (NUMBER_MOTOR <= 1)
		for (i = 0; i < MAX_PWM_OUTPUTS; i++)
			pwmSetPeriod(DM[i], PWM_PERIOD_SERVO);
	else if (NUMBER_MOTOR <= 4) {
		for (i = 0; i < 4; i++)
			pwmSetPeriod(DM[i], PWM_PERIOD);
		for (i = 4; i < MAX_PWM_OUTPUTS; i++)
			pwmSetPeriod(DM[i], PWM_PERIOD_SERVO);
	} else {
		for (i = 0; i < 8; i++)
			pwmSetPeriod(DM[i], PWM_PERIOD);
		for (i = 8; i < MAX_PWM_OUTPUTS; i++)
			pwmSetPeriod(DM[i], PWM_PERIOD_SERVO);
	}

	/********  special version of MultiWii to calibrate all attached ESCs ************/
#if defined(ESC_CALIB_CANNOT_FLY)
	writeAllMotors(ESC_CALIB_HIGH);
	delay(3000);
	writeAllMotors(ESC_CALIB_LOW);
	delay(500);
	while (1) {
		delay(5000);
		blinkLED(2,20, 2);
	}
#endif

	writeAllMotors(MINCOMMAND);
	delay(300);
#if defined(SERVO)
	initializeServo();
#endif
}



/**************************************************************************************/
/********** Mixes the Computed stabilize values to the Motors & Servos  ***************/
/**************************************************************************************/
void mixTable() {
	int16_t maxMotor;
	uint8_t i;

#define PIDMIX(X,Y,Z) rcCommand[THROTTLE] + axisPID[ROLL]*X + axisPID[PITCH]*Y + YAW_DIRECTION * axisPID[YAW]*Z

#if NUMBER_MOTOR > 3
	//prevent "yaw jump" during yaw correction
	axisPID[YAW]
			= constrain(axisPID[YAW],-100-abs(rcCommand[YAW]),+100+abs(rcCommand[YAW]));
#endif
	/****************                   main Mix Table                ******************/
#ifdef BI
	motor[0] = PIDMIX(+1, 0, 0); //LEFT
	motor[1] = PIDMIX(-1, 0, 0); //RIGHT
	servo[4] = constrain(1500 + (YAW_DIRECTION * axisPID[YAW]) + axisPID[PITCH], 1020, 2000); //LEFT
	servo[5] = constrain(1500 + (YAW_DIRECTION * axisPID[YAW]) - axisPID[PITCH], 1020, 2000); //RIGHT
#endif
#ifdef TRI
	motor[0] = PIDMIX( 0,+4/3, 0); //REAR
	motor[1] = PIDMIX(-1,-2/3, 0); //RIGHT
	motor[2] = PIDMIX(+1,-2/3, 0); //LEFT
	servo[5] = constrain(conf.tri_yaw_middle + YAW_DIRECTION * axisPID[YAW], TRI_YAW_CONSTRAINT_MIN, TRI_YAW_CONSTRAINT_MAX); //REAR
#endif
#ifdef QUADP
	motor[0] = PIDMIX( 0,+1,-1); //REAR
	motor[1] = PIDMIX(-1, 0,+1); //RIGHT
	motor[2] = PIDMIX(+1, 0,+1); //LEFT
	motor[3] = PIDMIX( 0,-1,-1); //FRONT
#endif
#ifdef QUADX
	motor[0] = PIDMIX(-1,+1,-1); //REAR_R
	motor[1] = PIDMIX(-1,-1,+1); //FRONT_R
	motor[2] = PIDMIX(+1,+1,+1); //REAR_L
	motor[3] = PIDMIX(+1,-1,-1); //FRONT_L
#endif
#ifdef Y4
	motor[0] = PIDMIX(+0,+1,-1); //REAR_1 CW
	motor[1] = PIDMIX(-1,-1, 0); //FRONT_R CCW
	motor[2] = PIDMIX(+0,+1,+1); //REAR_2 CCW
	motor[3] = PIDMIX(+1,-1, 0); //FRONT_L CW
#endif
#ifdef Y6
	motor[0] = PIDMIX(+0,+4/3,+1); //REAR
	motor[1] = PIDMIX(-1,-2/3,-1); //RIGHT
	motor[2] = PIDMIX(+1,-2/3,-1); //LEFT
	motor[3] = PIDMIX(+0,+4/3,-1); //UNDER_REAR
	motor[4] = PIDMIX(-1,-2/3,+1); //UNDER_RIGHT
	motor[5] = PIDMIX(+1,-2/3,+1); //UNDER_LEFT
#endif
#ifdef HEX6
	motor[0] = PIDMIX(-7/8,+1/2,+1); //REAR_R
	motor[1] = PIDMIX(-7/8,-1/2,-1); //FRONT_R
	motor[2] = PIDMIX(+7/8,+1/2,+1); //REAR_L
	motor[3] = PIDMIX(+7/8,-1/2,-1); //FRONT_L
	motor[4] = PIDMIX(+0 ,-1 ,+1); //FRONT
	motor[5] = PIDMIX(+0 ,+1 ,-1); //REAR
#endif
#ifdef HEX6X
	motor[0] = PIDMIX(-1/2,+7/8,+1); //REAR_R
	motor[1] = PIDMIX(-1/2,-7/8,+1); //FRONT_R
	motor[2] = PIDMIX(+1/2,+7/8,-1); //REAR_L
	motor[3] = PIDMIX(+1/2,-7/8,-1); //FRONT_L
	motor[4] = PIDMIX(-1 ,+0 ,-1); //RIGHT
	motor[5] = PIDMIX(+1 ,+0 ,+1); //LEFT
#endif
#ifdef OCTOX8
	motor[0] = PIDMIX(-1,+1,-1); //REAR_R
	motor[1] = PIDMIX(-1,-1,+1); //FRONT_R
	motor[2] = PIDMIX(+1,+1,+1); //REAR_L
	motor[3] = PIDMIX(+1,-1,-1); //FRONT_L
	motor[4] = PIDMIX(-1,+1,+1); //UNDER_REAR_R
	motor[5] = PIDMIX(-1,-1,-1); //UNDER_FRONT_R
	motor[6] = PIDMIX(+1,+1,-1); //UNDER_REAR_L
	motor[7] = PIDMIX(+1,-1,+1); //UNDER_FRONT_L
#endif
#ifdef OCTOFLATP
	motor[0] = PIDMIX(+7/10,-7/10,+1); //FRONT_L
	motor[1] = PIDMIX(-7/10,-7/10,+1); //FRONT_R
	motor[2] = PIDMIX(-7/10,+7/10,+1); //REAR_R
	motor[3] = PIDMIX(+7/10,+7/10,+1); //REAR_L
	motor[4] = PIDMIX(+0 ,-1 ,-1); //FRONT
	motor[5] = PIDMIX(-1 ,+0 ,-1); //RIGHT
	motor[6] = PIDMIX(+0 ,+1 ,-1); //REAR
	motor[7] = PIDMIX(+1 ,+0 ,-1); //LEFT
#endif
#ifdef OCTOFLATX
	motor[0] = PIDMIX(+1 ,-1/2,+1); //MIDFRONT_L
	motor[1] = PIDMIX(-1/2,-1 ,+1); //FRONT_R
	motor[2] = PIDMIX(-1 ,+1/2,+1); //MIDREAR_R
	motor[3] = PIDMIX(+1/2,+1 ,+1); //REAR_L
	motor[4] = PIDMIX(+1/2,-1 ,-1); //FRONT_L
	motor[5] = PIDMIX(-1 ,-1/2,-1); //MIDFRONT_R
	motor[6] = PIDMIX(-1/2,+1 ,-1); //REAR_R
	motor[7] = PIDMIX(+1 ,+1/2,-1); //MIDREAR_L
#endif
#ifdef VTAIL4
	motor[0] = PIDMIX(+0,+1, +1); //REAR_R
	motor[1] = PIDMIX(-1, -1, +0); //FRONT_R
	motor[2] = PIDMIX(+0,+1, -1); //REAR_L
	motor[3] = PIDMIX(+1, -1, -0); //FRONT_L
#endif

	/****************                Cam stabilize Sevos             ******************/
#if defined(SERVO_TILT)
#if defined(A0_A1_PIN_HEX) && (NUMBER_MOTOR == 6) && defined(PROMINI)
#define S_PITCH servo[2]
#define S_ROLL  servo[3]
#else
#define S_PITCH servo[0]
#define S_ROLL  servo[1]
#endif
	S_PITCH = TILT_PITCH_MIDDLE + rcData[AUX3]-1500;
	S_ROLL = TILT_ROLL_MIDDLE + rcData[AUX4]-1500;
	if (rcOptions[BOXCAMSTAB]) {
		S_PITCH += TILT_PITCH_PROP * angle[PITCH] /16;
		S_ROLL += TILT_ROLL_PROP * angle[ROLL] /16;
	}
	S_PITCH = constrain(S_PITCH, TILT_PITCH_MIN, TILT_PITCH_MAX);
	S_ROLL = constrain(S_ROLL , TILT_ROLL_MIN, TILT_ROLL_MAX );
#endif

#ifdef GIMBAL
	servo[0] = constrain(TILT_PITCH_MIDDLE + TILT_PITCH_PROP * angle[PITCH] /16 + rcCommand[PITCH], TILT_PITCH_MIN, TILT_PITCH_MAX);
	servo[1] = constrain(TILT_ROLL_MIDDLE + TILT_ROLL_PROP * angle[ROLL] /16 + rcCommand[ROLL], TILT_ROLL_MIN, TILT_ROLL_MAX);
#endif
#if defined(FLYING_WING)
	motor[0] = rcCommand[THROTTLE];
	if (f.PASSTHRU_MODE) {// do not use sensors for correction, simple 2 channel mixing
		servo[0] = PITCH_DIRECTION_L * (rcData[PITCH]-MIDRC) + ROLL_DIRECTION_L * (rcData[ROLL]-MIDRC);
		servo[1] = PITCH_DIRECTION_R * (rcData[PITCH]-MIDRC) + ROLL_DIRECTION_R * (rcData[ROLL]-MIDRC);
	} else { // use sensors to correct (gyro only or gyro+acc according to aux1/aux2 configuration
		servo[0] = PITCH_DIRECTION_L * axisPID[PITCH] + ROLL_DIRECTION_L * axisPID[ROLL];
		servo[1] = PITCH_DIRECTION_R * axisPID[PITCH] + ROLL_DIRECTION_R * axisPID[ROLL];
	}
	servo[0] = constrain(servo[0] + conf.wing_left_mid , WING_LEFT_MIN, WING_LEFT_MAX );
	servo[1] = constrain(servo[1] + conf.wing_right_mid, WING_RIGHT_MIN, WING_RIGHT_MAX);
#endif

	/************************************************************************************************************/
#if defined(AIRPLANE) || defined(SINGLECOPTER) || defined(DUALCOPTER)
	// Common parts for Plane and Heli
	static int16_t servoMid[8]; // Midpoint on servo
	static uint8_t servoTravel[8] = SERVO_RATES; // Rates in 0-100%
	static int8_t servoReverse[8] = SERVO_DIRECTION; // Inverted servos
	static int16_t servoLimit[8][2]; // Holds servoLimit data

	/***************************
	 * servo endpoints Airplane.
	 ***************************/
#define SERVO_MIN 1020           // limit servo travel range must be inside [1020;2000]
#define SERVO_MAX 2000           // limit servo travel range must be inside [1020;2000]
	for(i=0; i<8; i++) { //  Set rates with 0 - 100%.
		servoMid[i] = MIDRC + conf.servoTrim[i];
		servoLimit[i][0]=servoMid[i]-((servoMid[i]-SERVO_MIN) *(servoTravel[i]*0.01));
		servoLimit[i][1]=servoMid[i]+((SERVO_MAX - servoMid[i]) *(servoTravel[i]*0.01));
	}

	// servo[7] is programmed with safety features to avoid motor starts when ardu reset..
	// All other servos go to center at reset..  Half throttle can be dangerous
	// Only use servo[7] as motor control if motor is used in the setup            */
	if (!f.ARMED) {
		servo[7] = MINCOMMAND; // Kill throttle when disarmed
	} else {
		servo[7] = rcData[THROTTLE];
	}

	// Flaperon Control
	int16_t flapperons[2]= {0,0};
#if  defined(FLAPPERONS) && defined(FLAPPERON_EP)
	int8_t flapinv[2] = FLAPPERON_INVERT;
	static int16_t F_Endpoint[2] = FLAPPERON_EP;
	int16_t flap =MIDRC-constrain(rcData[FLAPPERONS],F_Endpoint[1],F_Endpoint[0]);
	static int16_t slowFlaps= flap;
#if defined(FLAPSPEED)
	if (slowFlaps < flap ) {slowFlaps+=FLAPSPEED;} else if(slowFlaps > flap) {slowFlaps-=FLAPSPEED;}
#else
	slowFlaps = flap;
#endif
	flap = MIDRC-(constrain(MIDRC-slowFlaps,F_Endpoint[0],F_Endpoint[1]));
	for(i=0; i<2; i++) {flapperons[i] = flap * flapinv[i];}
#endif

	// Traditional Flaps on A2
#if defined(FLAPS)  && defined(FLAP_EP)
	static int16_t lF_Endpoint[2] = FLAP_EP;
	int16_t lFlap = MIDRC-constrain(rcData[FLAPS],lF_Endpoint[0],lF_Endpoint[1]);
	static int16_t slow_LFlaps= lFlap;
#if defined(FLAPSPEED)
	if (slow_LFlaps < lFlap ) {slow_LFlaps+=FLAPSPEED;} else if(slow_LFlaps > lFlap) {slow_LFlaps-=FLAPSPEED;}
#else
	slow_LFlaps = lFlap;
#endif
	servo[2] = servoMid[2]+(slow_LFlaps *servoReverse[2]);
#endif

#if defined(AIRPLANE)
	if(f.PASSTHRU_MODE) { // Direct passthru from RX
		servo[3] = servoMid[3]+((rcCommand[ROLL] + flapperons[0]) *servoReverse[3]); //   Wing 1
		servo[4] = servoMid[4]+((rcCommand[ROLL] + flapperons[1]) *servoReverse[4]); //   Wing 2
		servo[5] = servoMid[5]+(rcCommand[YAW] *servoReverse[5]); //   Rudder
		servo[6] = servoMid[6]+(rcCommand[PITCH] *servoReverse[6]); //   Elevator
	} else {

		// Assisted modes (gyro only or gyro+acc according to AUX configuration in Gui
		servo[3] =(servoMid[3] + ((axisPID[ROLL] + flapperons[0]) *servoReverse[3])); //   Wing 1
		servo[4] =(servoMid[4] + ((axisPID[ROLL] + flapperons[1]) *servoReverse[4])); //   Wing 2
		servo[5] =(servoMid[5] + (axisPID[YAW] *servoReverse[5])); //   Rudder
		servo[6] =(servoMid[6] + (axisPID[PITCH] *servoReverse[6])); //   Elevator
	}
#endif


	/*************************************************************************************************************************/
	/*************************************************************************************************************************/
	/******************                   Development of single & DualCopter                            **********************/
	/*************************************************************************************************************************/
	/*************************************************************************************************************************/
#if  defined(SINGLECOPTER)
	int8_t yawServo[4] =SINGLECOPTRER_YAW;
	int8_t scServo[4] =SINGLECOPTRER_SERVO;
	// Singlecopter
	// This is a beta requested by  xuant9
	// Assisted modes (gyro only or gyro+acc according to AUX configuration in Gui
	// http://www.kkmulticopter.kr/multicopter/images/blue_single.jpg
	servo[3] = servoMid[3] + (axisPID[YAW]*yawServo[0]) + (axisPID[PITCH]*scServo[0]); //   SideServo  5  D12
	servo[4] = servoMid[4] + (axisPID[YAW]*yawServo[1]) + (axisPID[PITCH]*scServo[1]); //   SideServo  3  D11
	servo[5] = servoMid[5] + (axisPID[YAW]*yawServo[2]) + (axisPID[ROLL] *scServo[2]); //   FrontServo 2  D3
	servo[6] = servoMid[6] + (axisPID[YAW]*yawServo[3]) + (axisPID[ROLL] *scServo[3]); //   RearServo  4  D10
	motor[1] = rcData[THROTTLE]; //  Pin D10
#endif
#if  defined(DUALCOPTER)
	int8_t dcServo[2] =DUALCOPTER_SERVO;
	// Dualcopter
	// This is a beta requested by  xuant9
	// Assisted modes (gyro only or gyro+acc according to AUX configuration in Gui
	//http://www.kkmulticopter.kr/products_pic/index.html?sn=multicopter_v02_du&name=KKMultiCopter%20Flight%20Controller%20Blackboard%3Cbr%3E
	servo[5] = servoMid[5] + (axisPID[PITCH] * dcServo[0]); //  PITCHServo 3  D12
	servo[6] = servoMid[6] + (axisPID[ROLL] * dcServo[1]); //  ROLLServo  4  D11
	motor[0] = PIDMIX(0,0,-1); //  Pin D9
	motor[1] = PIDMIX(0,0,+1); //  Pin D10
#endif

	/*************************************************************************************************************************/
	/*************************************************************************************************************************/
	/*************************************************************************************************************************/
	/*************************************************************************************************************************/
	// ServoRates
	for(i=3;i<8;i++) {
		servo[i] = map(servo[i], SERVO_MIN, SERVO_MAX,servoLimit[i][0],servoLimit[i][1]);
		servo[i] = constrain( servo[i], SERVO_MIN, SERVO_MAX);
	}

#endif

	/************************************************************************************************************/
#ifdef HELICOPTER
	// Common controlls for Helicopters
	int16_t heliRoll,heliNick;
	int16_t collRange[3] = COLLECTIVE_RANGE;
	static int16_t collective;
	static int16_t servoEndpiont[8][2];
	static int16_t servoHigh[8] = SERVO_ENDPOINT_HIGH; // HIGHpoint on servo
	static int16_t servoLow[8] = SERVO_ENDPOINT_LOW; // LOWpoint on servo

	/***************************
	 * servo settings Heli.
	 ***************************/
	for(i=0; i<8; i++) { //  Set rates using endpoints.
		servoEndpiont[i][0] = servoLow[i]; //Min
		servoEndpiont[i][1] = servoHigh[i]; //Max
	}

	// Limit Collective range up/down
	int16_t collect = rcData[COLLECTIVE_PITCH]-collRange[1];
	if (collect>0) {
		collective = collect * (collRange[2]*0.01);
	} else {
		collective = collect * (collRange[0]*0.01);
	}

	if(f.PASSTHRU_MODE) { // Use Rcdata Without sensors
		heliRoll= rcCommand[ROLL];
		heliNick= rcCommand[PITCH];
	} else { // Assisted modes
		heliRoll= axisPID[ROLL];
		heliNick= axisPID[PITCH];
	}

	// Limit Maximum Rates for Heli
	int16_t cRange[2] = CONTROLL_RANGE;
	heliRoll*=cRange[0]*0.01;
	heliNick*=cRange[1]*0.01;

#define HeliXPIDMIX(Z,Y,X) collRange[1]+collective*Z + heliNick*Y +  heliRoll*X

	// Yaw is common for Heli 90 & 120
	uint16_t yawControll = YAW_CENTER + (axisPID[YAW]*YAW_DIRECTION) + conf.servoTrim[5];

	/* Throttle & YAW
	 ********************
	 Handeled in common functions for Heli */
	if (!f.ARMED) {
		servo[7] = 900; // Kill throttle when disarmed
		if (YAWMOTOR) {servo[5] = MINCOMMAND;} else {servo[5] = yawControll;} // Kill YAWMOTOR when disarmed
	} else {
		servo[7] = rcData[THROTTLE]; //   50hz ESC or servo
		if (YAWMOTOR && rcData[THROTTLE] < MINTHROTTLE) {servo[5] = MINCOMMAND;}
		else {servo[5] = yawControll;} // YawSero
	}
#ifndef HELI_USE_SERVO_FOR_THROTTLE
	motor[0] = servo[7]; // use real motor output - ESC capable
#endif

	//              ( Collective, Pitch/Nick, Roll ) Change sign to invert
	/************************************************************************************************************/
#ifdef HELI_120_CCPM
	static int8_t nickMix[3] =SERVO_NICK;
	static int8_t leftMix[3] =SERVO_LEFT;
	static int8_t rightMix[3]=SERVO_RIGHT;

	servo[3] = HeliXPIDMIX( (nickMix[0]*0.1) , nickMix[1]*0.1, nickMix[2]*0.1) +conf.servoTrim[3]; //    NICK  servo
	servo[4] = HeliXPIDMIX( (leftMix[0]*0.1) , leftMix[1]*0.1, leftMix[2]*0.1) +conf.servoTrim[4]; //    LEFT servo
	servo[6] = HeliXPIDMIX( (rightMix[0]*0.1),rightMix[1]*0.1,rightMix[2]*0.1) +conf.servoTrim[6]; //    RIGHT  servo

#endif

	/************************************************************************************************************/
#ifdef HELI_90_DEG
	static int8_t servoDir[3]=SERVO_DIRECTIONS;
	servo[3] = HeliXPIDMIX( +0, servoDir[1], -0)+conf.servoTrim[3]; //     NICK  servo
	servo[4] = HeliXPIDMIX( +0, +0, servoDir[2])+conf.servoTrim[4]; //     ROLL servo
	servo[6] = HeliXPIDMIX( servoDir[0], +0, +0)+conf.servoTrim[6]; //     COLLECTIVE  servo
#endif

	for(i=3;i<8;i++) {
		servo[i] = constrain( servo[i], servoEndpiont[i][0], servoEndpiont[i][1] );
	}

#endif

	// End of PatrikE Experimentals
	/************************************************************************************************************/

	/****************                    Cam trigger Sevo                ******************/
#if defined(CAMTRIG)
	static uint8_t camCycle = 0;
	static uint8_t camState = 0;
	static uint32_t camTime = 0;
	if (camCycle==1) {
		if (camState == 0) {
			servo[2] = CAM_SERVO_HIGH;
			camState = 1;
			camTime = millis();
		} else if (camState == 1) {
			if ( (millis() - camTime) > CAM_TIME_HIGH ) {
				servo[2] = CAM_SERVO_LOW;
				camState = 2;
				camTime = millis();
			}
		} else { //camState ==2
			if ( (millis() - camTime) > CAM_TIME_LOW ) {
				camState = 0;
				camCycle = 0;
			}
		}
	}
	if (rcOptions[BOXCAMTRIG]) camCycle=1;
#endif

	/****************                Filter the Motors values                ******************/
	maxMotor = motor[0];
	for (i = 1; i < NUMBER_MOTOR; i++)
		if (motor[i] > maxMotor)
			maxMotor = motor[i];
	for (i = 0; i < NUMBER_MOTOR; i++) {
		if (maxMotor > MAXTHROTTLE) // this is a way to still have good gyro corrections if at least one motor reaches its max.
			motor[i] -= maxMotor - MAXTHROTTLE;
		motor[i] = constrain(motor[i], MINTHROTTLE, MAXTHROTTLE);
		if ((rcData[THROTTLE]) < MINCHECK)
#ifndef MOTOR_STOP
			motor[i] = MINTHROTTLE;
#else
		motor[i] = MINCOMMAND;
#endif
		if (!f.ARMED)
			motor[i] = MINCOMMAND;
	}
	/****************                      Powermeter Log                    ******************/
#if (LOG_VALUES == 2) || defined(POWERMETER_SOFT)
	uint32_t amp;
	/* true cubic function; when divided by vbat_max=126 (12.6V) for 3 cell battery this gives maximum value of ~ 500 */

	static uint16_t amperes[64] = {0, 2, 6, 15, 30, 52, 82,123,
		175,240,320,415,528,659,811,984,
		1181,1402,1648,1923,2226,2559,2924,3322,
		3755,4224,4730,5276,5861,6489,7160,7875,
		8637 ,9446 ,10304,11213,12173,13187,14256,15381,
		16564,17805,19108,20472,21900,23392,24951,26578,
		28274,30041,31879,33792,35779,37843,39984,42205,
		44507,46890,49358,51910,54549,57276,60093,63000};

	if (vbat) { // by all means - must avoid division by zero
		for (i =0;i<NUMBER_MOTOR;i++) {
			amp = amperes[ ((motor[i] - 1000)>>4) ] / vbat; // range mapped from [1000:2000] => [0:1000]; then break that up into 64 ranges; lookup amp
#if (LOG_VALUES == 2)
			pMeter[i]+= amp; // sum up over time the mapped ESC input
#endif
#if defined(POWERMETER_SOFT)
			pMeter[PMOTOR_SUM]+= amp; // total sum over all motors
#endif
		}
	}
#endif
}

