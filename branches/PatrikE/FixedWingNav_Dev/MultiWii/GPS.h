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

// Use the Patched I2C GPS for FixedWing and OSD
//#define I2CPATCH   
/* 
   Values set in GUI.
   Set ABS Target Alt for RTL over home position. 
   RTH_Alt is set with (POSR) => D   

   for Navigation      (NavR) => P,I & D
   for Altitue.        (ALT)  => P, I &D
*/

/* Maximum Limits for controls */
#define GPS_MAXCORR    25     // Degrees banking applied by GPS.
#define GPS_MAXCLIMB   15     // Degrees climbing . To much can stall the plane.
#define GPS_MAXDIVE    15     // Degrees Diving . To much can overspeed the plane.

#define CLIMBTHROTTLE  1900  // Max allowed throttle in GPS modes.
#define CRUICETHROTTLE 1650   // Throttle to set for cruisespeed.

#define IDLE_THROTTLE   1300  // Lowest throttleValue during Descend
#define SCALER_THROTTLE  8    // Adjust to Match Power/Weight ratio of your model

#define FAILSAFE_RTH      true  // Enable RTH for failsafe incl Auto DisARM at home to autoland

#define SAFE_NAV_ALT        20  // Safe Altitude during climbouts Wings Level below this Alt. (ex. trees & buildings..)
#define SAFE_DECSCEND_ZONE  50  // Radius around home where descending is OK


 

#endif /* GPS_H_ */
