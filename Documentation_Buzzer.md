# Introduction #

The buzzer is used to give the pilot some feedback of the functions and errors of the MultiWii system.

# Table of contents #
  1. Hardware setup
  1. Software setup
  1. How it works

# Hardware #
You need to connect a piezo buzzer or a pilotlamp to the dedicated buzzer port of you FC.

# Software #

In config.h you need to alter one of the following lines:

#define BUZZER

#define PILOTLAMP

to enable the buzzer.

# How it works #

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