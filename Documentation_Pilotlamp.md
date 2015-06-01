# Introduction #

The PilotLampis used to give the pilot even more feedback of the functions and errors of the MultiWii system than the buzzer alone.

# Table of contents #
  1. Hardware setup
  1. Software setup
  1. How it works

# Hardware #
You need to connect the pilotlamp to the dedicated buzzer port of your FC and to 5V and ground.

# Software #

In config.h you need to alter the following line:

#define PILOTLAMP

to enable the pilot lamp.

# How it works #

**LEDs**

There are three different colors on the pilot light:red, green and blue.

  * Red: error indicator
  * Green: Flight mode indicator
  * GPS function indicator

  * Armed, acro mode: green LED medium blinking 2x
  * Armed, angle mode: green LED slow blinking 3x
  * Armed, horizon mode: green LED medium blinking 3x

**Blue LED**
  * GPS present: No GPS fix: fast blinking
  * GPS present: Valid Fix: slow short blinking
  * GPS present: RTH/PH active: slow long blinking

**Red LED**
  * Failsafe landing active: fast blinking
  * Failsafe find me: slow blinking

**Beeper**
  * Sequence RED->BLUE->GREEN (repeat)

**I²C**
  * All colors blink at the same time, buzzer soundsBuzzer

There are several mandatory and optional beeping patterns to indicate the different states of the MultWii system. The patternn consist of 3 beeps followed by a pause. The length of the beeps and pauses are defined as follows:
  * N: None (0ms)
  * S: Short (50ms)
  * L: Long (200ms)
  * D: Double (2000ms)

The beeps are priority driven, the first entry has the highest priority:

  1. **Failsafe find me signal:** LNN,D (repeat)
  1. **Failsafe landing active:** SLL,S  (repeat)
  1. **GPS RTH/ PH activated without fix:** SSN,S (repeat)
  1. **Beeper:** SSS,S (repeat)
  1. **Runtime Warning:** SSS,N (repeat)  (#define ARMEDTIMEWARNING)
  1. **Battery voltage warning level 3:** SLL,D (repeat)
  1. **Battery voltage warning level 2:** SSL,D (repeat)
  1. **Battery voltage warning level 1:** SLN,D (repeat)
  1. **RCoptions Toggle:** S   (#define RCOPTIONSBEEP)
Additional beeps
  * **Inflight ACC calibration activated:** SS (#define INFLIGHT\_ACC\_CALIBRATION)
  * **Inflight ACC calibration deactivated:** SSS
  * **Gyro calibration done:** SSS
  * **ACC calibration done:** L
  * **LCD Configuration step:** S