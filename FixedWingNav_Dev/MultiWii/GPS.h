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

/* Set ABS Target Alt for RTL over home position. */
// Use  GUI  PID 
//  ALT D To set RTH Alt.
   

// Moved to GUI as PID.
// Use GUI NavR for roll & rudder
// PID for Roll.   P=3 recomended I & D needs testing

// Use GUI PIDALT P for elevator
// Only P used for Elevator.  P=3 recomended

#define GPS_UPD_HZ     5   // Set loop time for NavUpdate

/* Set Maximum Limits for PID-controls */
#define MAX_I 200    // Increase steps by 10 for more control (1 degree)
#define MAX_D 200    

/* Maximum Limits for controls */
#define GPS_MAXCORR    15   // Degrees banking applied by GPS.
#define GPS_MAXCLIMB   15   // Degrees climbing . To much can stall plane.


#define CLIMBTHROTTLE  1800 // Max allowed throttle in GPS modes .
#define CRUICETHROTTLE 1500 // Throttle to set in cruise.

#define IDLE_THROTTLE   1300  // Lowest throttleValue during Descend
#define SCALER_THROTTLE  8    // Adjust to Match Power/Weight ratio of your model

#define FAILSAFE_RTH      true  // Enable RTH for failsafe incl Auto DisARM at home and autoland

#define CLIMBOUT          false // Forced RTH Climbout to min RTH alt with Level Wings
#define SAFE_NAV_ALT        20  // Safe Altitude during climbouts Wings Level below this Alt. (ex. treetop height..) For Auto launch etc.
#define SAFE_DECSCEND_ZONE  50  // Radius around home where descending is OK


#define ELEVATORCOMPENSATION 100 // Compensate elevator with % of rollAngle
                                 // Elevator compensates up when plane Roll
                                 // Set to negative to reverse.                                 
/*****************************************************/

#endif /* GPS_H_ */
