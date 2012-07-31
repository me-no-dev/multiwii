/* 
 Multiwiicopter.com LED Ring
  - I2C implementation for Multiwii
  - Standalone implementation 

 by Alexander Dubus
 adapted by Shikra
*/

/* SUPORTED MULTIWII RELEASES - SUPORTED MULTIWII RELEASES  ********************
 - 2.2 dev versions
 - 2.1 with a modified LEDring.ino
 - 2.0 with a modified LEDring.ino 
 
 Note this now requires Arduino 1.0 to compile
*/

 
/* CONNECTING HARDWARE - CONNECTING HARDWARE - CONNECTING HARDWARE  ************
For programming, connect directly to the LEDring 6 pin connector marked gnd/vcc/rx/tx/dtr with a 5v programmer

For connecting to Multiwii after LEDring is programmed: 
Vcc = 5v
SDA = SDA (can be either 5v or 3.3v)
SCL = SDA (can be either 5v or 3.3v)
GND = GND

NOTE! - if LEDring pullups are defined, then only use with 5v SDA/SCL)
*/


 
/* INSTALLATION NOTES -INSTALLATION NOTES -INSTALLATION NOTES ******************  
 
 STEP1 - Program LED board
 STEP2 - If not using a compatible version (2.2) Recompile MultiWii2.0 with revised LED.INO 
 STEP3 - verify all runs OK and no new I2C errors on the GUI.

STEP1 - Program LED board
 Drivers need to be installed for the USB programmer. This is probably not needed if used same pragrammer as for Multiwii board
 LED Ring v3 uses Opti bootloader so Arduino IDE may need to be modified to allow programming. The following should work...
 Use Arduino v1.0 or greater.
 Add the contents of included "Append to Boards.txt" into end of boards.txt which you will find in your arduino folder ...\hardware\arduino\boards.txt
 Restart Arduino IDE and you should see the LED board listed as a board option.
 Note - if you have multiple IDE's installed, need to amend the correct boards.txt file. If board does not show this is probably why.
 Now select the new board listed from Tools-->board-->MultiWiiCopter LEDring v3
 Open the LEDring.PDEsketch and program as normal
 Don't forget to set board back to program Multiwii !!

STEP2 - Program MultiWii
 Multiwii 2.2 dev should be fully compatible with this release. Just enable #define LED_RING in config.h
 For release 2.1/2.0:
 Replace existing LED.ino in with the new one provided
 Enable #define LED_RING in config.h
 Program as normal

STEP3 - Verify correct operation.
 Ensure that all wiring is correct.
 Ensure that LED's flash as expected.
 Veryify no additional I2C errors
 */



/* CONFIGURATION OPTIONS -CONFIGURATION OPTIONS -CONFIGURATION OPTIONS *********  
MultiWii_I2C_v2  - to use Multiwii enhanced LEDring funcionality. (standard in 2.2 dev)
MultiWii_I2C_v1  - to use standard Multiwii 2.0/2.1 LED functionality. 
Standalone       - to use standalone. Select LED flash sequence from switch.

LEDBOARDv2 - pre and early 2011 hardware
LEDBOARDv3 - released late 2011

I2C_address - option to select alternative to avoid clash 
INTERNAL_I2C_PULLUPS - for specific hardware configurations

PWM LOW/MID/HIGH - select LED sequences for switch positions
*/



/* OPERATION MODES - OPERATION MODES - OPERATION MODES - OPERATION MODES -  ****  
2 modes - 
Mode 0 - Standalone - set LED sequence flash mode via Switch 1. 
 - Switch S1 - Toggles between various flashing sequences when run standalone without I2C connection. 
 - Switch S2 - Toggles on/off 
 - Modes / flash sequence remembered between restarts
 Mode 1 - MultiWii - If Autodetect I2C Multiwii command, puts into Multiwii control
 - Switch S2 - Toggles on/off 
Mode 2 - PWM controlled
 - Support for Non Multiwii controllers
 - 3 LED sequences can be controlled by TX 2/3 position switch.
 - Not avaialble in this release  
*/



/* LED DISPLAYS - LED DISPLAYS - LED DISPLAYS - LED DISPLAYS - *****************  
Multiwii connected Mode:
-----------------------
Motors on 
 - ACRO flying - MultiWiicopter Navigation lights / strobe
 - LEVEL flying - Static Red/Green. Position for orientation. 
 - Position hold - Static all Blue
 - RTH - Flashing all Blue
 - Battery level low - Fast flashing all Red   ** UNTESTED **

Motors off  
 - Unstable position warning - Fast Green flash
 - Acc not calibrated - Fast Green flash 
 - Acro mode with BARO/MAG disabled - cool MultiWiicopter Navi lights

Motors off - BARO/MAG enabled
 - ACRO mode - base color = RED
 - LEVEL mode - base color = GREEN
 -->IF MAG = North direction indicated by single Blue
 -->IF BARO = flash every other blue

Motors off - GPS RTH/POSHOLD enabled
 - base color = WHITE
 - no sats = circling red LED (NOT ready to fly)
 - sats < 5 = flashing RED count of numberof sats (NOT ready to fly) 
 - sats 5 pr more = steady RED count of number of sats (ready to fly)

Standalone Mode:
---------------
Sequence 0 - Static Red
Sequence 1 - Static Green
Sequence 2 - Static Blue
Sequence 3 - Flashing Red
Sequence 4 - Flashing Green
Sequence 5 - Flashing Blue
Sequence 6 - Fast Red flash
Sequence 7 - Fast Green flash
Sequence 8 - Fast Blue flash
Sequence 10 - Standard Navigation lights. Red left, Green right, white rear, flashing red flashing white strobe 
Sequence 11 - MultiWiccopter Navigation lights. Red left, Green right, white rear, all flashing red / flashing white strobe 
Sequence 12 - Flashing Red/Green. Red facing forward for orientation
Sequence 13 - Static Red/Green. Red facing forward for orientation 
Sequence 14 - MultiWiiCopter Andromeda. Blue forward/White Sides/Red rear for orientation
Sequence 15 - Circling effect. Red circling LED on white
Sequence 16 - Circling effect. Green circling LED on white
Sequence 17 - Circling effect. Blue circling LED on white
Sequence 20 - Alexander the great effect 1
Sequence 21 - Alexander the great effect 2
Sequence 22 - Random 
*/



