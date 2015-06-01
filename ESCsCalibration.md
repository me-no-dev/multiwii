# Introduction #

If the ESCs of the copter  need calibration, a special version of MultiWii can be built, that does nothing but trying to calibrate all attached ESCs. This may come in handy when plugging/unplugging of the cables is undesirable or near impossible (soldered cables)

# How To: #
  1. remove props or tie copter down.
  1. setup all options in config.h to whatever suits your copter
  1. activate the _define_ _ESC\_CALIB\_CANNOT\_FLY_, possibly set high and low values for ESC calibration, if you know what you are doing
  1. compile, upload, run --- cannot fly and will use Buzzer/LEDs to indicate finished calibration (after approx 10 seconds)
  1. comment the define again, compile, upload
  1. test carefully with your ESCs calibrated, fly and have fun

# Details #
If neccessary, the low and high values for the ESCs can be tweaked by changing the values for the defines of ESC\_CALIB\_LOW  and ESC\_CALIB\_HIGH 2000

# Warning #
Remove propellers or tie your copter down. Seriously. If for whatever reason this firmware would not make the ESCs enter calibration mode, it would nevertheless send 'full throttle' signals to the ESCs for some time. This is guaranteed to be dangerous.

# !!! #
This page is not maintained any more. The information was moved to the new official multiwii wiki at http://www.multiwii.com/wiki/index.php?title=Extra_features#ESCsCalibration