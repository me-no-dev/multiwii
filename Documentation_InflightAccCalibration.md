# Introduction #

You have to calibrate your ACC at least once after you cleared the eeprom. For a correct calibration you need to put the copter on a absolutely flat surface and start the calibration.

Sometimes there is no flat surface or the copter is not built exactly even or you have a flexible landing gear. in these cases a common calibration is not possible and the copter will start to drift. You have the possibility to trim the copter via stick combo by trial and error.
The inflight acc calibration is a way to trim the copter to make it drift free in just a glimpse.
Arm/disarm via Aux switch is mandatory to do this.
You need a buzzer to ehar the beeping signals in the process.

# Table of contents #
  1. Software setup
  1. How it works

# Software #

In config.h you need to alter one of the following lines:

#define INFLIGHT\_ACC\_CALIBRATION

to enable the option.

# How it works #

## Concept ##
  * fly
  * trim/get stable
  * save deviation
  * land
  * finished

One of the following procedures has to be followed to finish the calibration.

## Activation with stick combo ##

Preparation: Plug the power and wait for the init to finish, the ACC has to be calibrated at least once since you cleared the eeporm.

1. do the following stick combination (throttle low)+(yaw left)+(pitch forward)+(roll right)

2. you will hear two beeps that indicate that the function is armed (to disarm it simply repeat the combo - you will hear three beeps)

3. start your copter and get airborne

4a) in acro mode: get your copter level and make sure it does not drift

4b) in level mode: get you copter level using the trims on your tx until it does not drift

5. now disarm the copter with the aux switch - you will hear a beep:the measurement is finished (don't worry, a failsafe feature prevents it from shutting down if throttle is above minthrottle)

6. you may rearm the copter now to prevent a crash while landing

7. Land the copter and disarm it, you will hear a long beep: The values have been stored in eeprom

8. if you have choosen 4 b): TAKE BACK ALL THE TRIMS BEFORE TAKEOFF

9. get airborne again - the copter should now be level without drift

10. repeat if neccesary




## Activation with passthrough switch ##

Preparation: Plug the power and wait for the init to finish, the ACC has to be calibrated at least once since you cleared the eeporm.

1. start your copter and get airborne

2a) in acro mode: get your copter level and make sure it does not drift

2b) in level mode: get you copter level using the trims on your tx until it does not drift

3. Activate "Passhtrough" via Aux switch


4. Land the copter and disarm it

5. deactivate "Passhtrough" - you will hear a long beep: The values have been stored in eeprom

6. if you have choosen 2 b): TAKE BACK ALL THE TRIMS BEFORE TAKEOFF

7. get airborne again - the copter should now be level without drift

8. repeat if neccesary