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

// External TWI EEPROM

#include "harness.h"

#ifdef STM32F1
#define NV_FLASH_SIZE 2048
#else
#define NV_FLASH_ORIGIN 0x00004000
#define NV_FLASH_SIZE 16384
#endif

__attribute__((__section__(".scratchflash")))   const int8 FlashNV[NV_FLASH_SIZE];
#define FLASH_SCRATCH_ADDR (0x08000000+ NV_FLASH_ORIGIN)
#define FLASH_SCRATCH_SECTOR	FLASH_Sector_1 // 11
#define HaveEEPROM false

uint32_t EEPROMReady = 0;

boolean WriteBlockEE(uint32 a, uint16 l, int8 *v) {
	boolean r = true;
	uint8 i;
	uint8 b[66]; // I2C implementation restriction
	uint8 bank;
	i8u8u u;

	while (uSClock() < EEPROMReady) {
		// BLOCKING
	};
	bank = 0;// only one chip (a & 0x00070000) >> 15;
	b[0] = (a >> 8) & 0xff;
	b[1] = a & 0xff; // messy - i2c routines use 8 bit register#
	for (i = 0; i < l; i++) {
		u.i8 = v[i];
		b[i + 2] = u.u8;
	}

	r &= i2cWriteBlock(I2CEEPROM, EEPROM_ID | bank, 0xff, 2 + l, b);
	EEPROMReady = uSClock() + 5000;
	return (r);

} // WriteBlockEE

void ReadBlockEE(uint32 a, uint16 l, int8 * v) {
	uint8 bank;
	uint8 b[2];

	while (uSClock() < EEPROMReady) {
		// BLOCKING
	};
	bank = 0; // only one chip (a & 0x00070000) >> 15;
	b[0] = (a >> 8) & 0xff;
	b[1] = a & 0xff; // messy - i2c routines use 8 bit register#
	i2cWriteBlock(I2CEEPROM, EEPROM_ID | bank, 0xff, 2, b);
	i2cRead(I2CEEPROM, EEPROM_ID | bank, 0xff, l, (uint8*) v);

} // ReadBlockEE


void ReadBlockNV(uint32 a, uint16 l, uint8 * v) {
	uint16 i;

	for (i = 0; i < l; i++)
		v[i] = *(uint8 *) (FLASH_SCRATCH_ADDR + a + i);

} // ReadBlockNV


boolean WriteBlockNV(uint32 a, uint16 l, uint8 * v) {
	//zzzzzzzzzzzzzzzzzzzzzzz issues iwth addressing conf??????????????
	uint16 i;
	boolean r = true;

	r = false;

	if (!r) {
		FLASH_Unlock();
#ifdef STM32F1
		FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
		r = FLASH_ErasePage(FLASH_SCRATCH_ADDR) == FLASH_COMPLETE;
#else
		FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR
				| FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
		r = FLASH_EraseSector(FLASH_SCRATCH_SECTOR, VoltageRange_3)
				== FLASH_COMPLETE;
#endif
		if (r) {
			for (i = 0; i < sizeof(conf); i += 4) {
				r = FLASH_ProgramWord(FLASH_SCRATCH_ADDR + i,
						*(uint32 *) ((int8 *)&conf + i)) == FLASH_COMPLETE;
				if (!r)
					break;
			}
		}
		FLASH_Lock();
	}

	return (r);

} // WriteConfNV


