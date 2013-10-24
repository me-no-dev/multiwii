/*
 * GPS.h
 *
 *  Created on: 20 juin 2013
 *      Author: WEYEE
 */

#ifndef GPS_H_
#define GPS_H_

extern int32_t wrap_18000(int32_t ang);

void FW_NAV(void);
void GPS_set_pids(void);
void GPS_SerialInit(void);
void GPS_NewData(void);
void GPS_reset_home_position(void);
void GPS_set_next_wp(int32_t* lat, int32_t* lon);
void GPS_reset_nav(void);
#if defined(I2C_GPS)
  void GPS_I2C_command(uint8_t command, uint8_t wp);
#endif
/*****************************************/
/*   Settings for FixedWing navigation   */
/*****************************************/

/* ABS Target Alt for RTL over home position. */
// Use  GUI  PID ALT D To set RTH Alt.

/* Correct direction by setting +/- 
   Increase for harder compensating
   Set to Zero to  disable         */
   

// Moved to Gui.
// Use Gui NavR
//#define GPS_RUDDCORR   1    // P for Rudder.
//#define GPS_NAVCORR    3    // P for Roll.

#define GPS_ALTCORR    10   // P for Elevator.

#define GPS_UPD_HZ     2   // Set looptime for NavUpdate

/* Set Maximum Limits for PID-controlls */
#define MAX_I 4000    // Increase steps by 1000 for more controll
#define MAX_D 2000

/* Maximum Limits for controlls */
#define GPS_MAXCORR    15   // Degrees banking aplyed by GPS.
#define GPS_MAXCLIMB   15   // Degrees climbing . To much can stall plane.

//#define SAFETY_SWITCH  AUX4 // Use a Safty switch for AutoThrottle. Must be over 1700ms THROTTLE can also be used
#define CLIMBTHROTTLE  1800 // Max allowed throttle in GPS modes .
#define CRUICETHROTTLE 1500 // Throttle to set in cruice.

// ....!!! New Defs !!!....
#define IDLE_THROTTLE 1200  // Lowest throttleValue during Descend
#define SCALER_THROTTLE 8   // Adjust to Match Power/Weiget ratio of your model
#define RTH_BAILOUT  false  // Forced RTH Climbout with Level Wings
#define FAILSAFE_RTH true   // Enable RTH for failsafe incl Auto DisARM at home


#define ELEVATORCOMPENSATION 100 // Compensate elevator with % of rollAngle
                                 // Elevator compensates up when plane Roll
                                 // Set to negative to reverse.                                 
/*****************************************************/

#endif /* GPS_H_ */
