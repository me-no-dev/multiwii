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
   

// Moved to GUI.
// Use GUI NavR
//#define GPS_NAVCORR    3    // P for Roll.
//#define GPS_RUDDCORR   1    // P for Rudder. common from GUI

// Moved to GUI.
// Use GUI PIDALT P
//#define GPS_ALTCORR    10   // P for Elevator.

#define GPS_UPD_HZ     2   // Set loop time for NavUpdate

/* Set Maximum Limits for PID-controls */
#define MAX_I 200    // Increase steps by 10 for more control (1 degree)
#define MAX_D 200    // 100 = 10 degree

/* Maximum Limits for controls */
#define GPS_MAXCORR    30   // Degrees banking aplyed by GPS.
#define GPS_MAXCLIMB   20   // Degrees climbing . To much can stall plane.


#define CLIMBTHROTTLE  1800 // Max allowed throttle in GPS modes .
#define CRUICETHROTTLE 1500 // Throttle to set in cruise.

#define IDLE_THROTTLE   1200  // Lowest throttleValue during Descend
#define SCALER_THROTTLE  8    // Adjust to Match Power/Weight ratio of your model

#define FAILSAFE_RTH    true  // Enable RTH for failsafe incl Auto DisARM at home for autoland

#define CLIMBOUT          true // Forced RTH Climbout with Level Wings For Auto launch etc.
#define SAFE_NAV_ALT        20 // Safe Altitude during climbouts Wings Level below this Alt. (ex. treetop height..)
#define SAFE_DECSCEND_ZONE  100// Radius around home where descending is OK


#define ELEVATORCOMPENSATION 100 // Compensate elevator with % of rollAngle
                                 // Elevator compensates up when plane Roll
                                 // Set to negative to reverse.                                 
/*****************************************************/

#endif /* GPS_H_ */
