// ===============================================================================================
// =                                UAVX Quadrocopter Controller                                 =
// =                           Copyright (c) 2008 by Prof. Greg Egan                             =
// =                 Original V3.15 Copyright (c) 2007 Ing. Wolfgang Mahringer                   =
// =                     http://code.google.com/p/uavp-mods/ http://uavp.ch                      =
// ===============================================================================================

//    This is part of UAVX.

//    UAVX is free software: you can redistribute it and/or modify it under the terms of the GNU 
//    General Public License as published by the Free Software Foundation, either version 3 of the 
//    License, or (at your option) any later version.

//    UAVX is distributed in the hope that it will be useful,but WITHOUT ANY WARRANTY; without
//    even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
//    See the GNU General Public License for more details.

//    You should have received a copy of the GNU General Public License along with this program.  
//    If not, see http://www.gnu.org/licenses/

#include "harness.h"

uint8 LEDChase[] = { LEDBlueSel, LEDGreenSel, LEDRedSel, LEDYellowSel };
boolean LEDsSaved[4] = { false };
uint8 LEDPattern = 0;

void BeeperOff(void) {
	digitalWrite(&GPIOPins[BeeperSel], 0);
} // BeeperOff

void BeeperOn(void) {
	digitalWrite(&GPIOPins[BeeperSel], 1);
} // BeeperOn

void BeeperToggle(void) {
	digitalToggle(&GPIOPins[BeeperSel]);
} // BeeperToggle

boolean BeeperIsOn(void) {
	return (digitalRead(&GPIOPins[BeeperSel]) != 0);
} // BeeperIsOn

inline void LEDOn(uint8 l) {

	if (l < MAX_LEDS)
		digitalWrite(&LEDPins[l], 0);

} // LEDOn

inline void LEDOff(uint8 l) {
	if (l < MAX_LEDS)
		digitalWrite(&LEDPins[l], 1);
} // LEDOff

inline void LEDToggle(uint8 l) {
	if (l < MAX_LEDS)
		digitalToggle(&LEDPins[l]);
} // LEDToggle

inline boolean LEDIsOn(uint8 l) {
	if (l < MAX_LEDS)
		return (!digitalRead(&LEDPins[l]));
	else
		return (false);
} // LEDIsOn

void LEDsOn(void) {
	uint8 l;

	for (l = 0; l < MAX_LEDS; l++)
		LEDOn(l);
} // LEDsOn

void LEDsOff(void) {
	uint8 l;

	for (l = 0; l < MAX_LEDS; l++)
		LEDOff(l);
} // LEDsOff

void SaveLEDs(void) { // one level only
	uint8 l;

	for (l = 0; l < MAX_LEDS; l++)
		LEDsSaved[l] = LEDIsOn(l);
} // SaveLEDs

void RestoreLEDs(void) {
	uint8 l;

	for (l = 0; l < MAX_LEDS; l++)
		LEDOn(LEDsSaved[l]);
} // RestoreLEDs


void InitLEDs(void) {

	LEDsOff();
	BeeperOff();
} // InitLEDs


