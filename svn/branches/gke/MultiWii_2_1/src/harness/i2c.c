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

//    UAVX is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
//    even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
//    See the GNU General Public License for more details.

//    You should have received a copy of the GNU General Public License along with this program.  
//    If not, see http://www.gnu.org/licenses/

#include "harness.h"

#define I2C_DEFAULT_TIMEOUT 30000

uint32 I2CClockRate = 100000;

volatile i2cStateDef i2cState[MAX_I2C_PORTS] = { { 0 } };

boolean I2CFatal = false;

void Catastrophe(void) {

	while (true) {
		LEDsOn();
		Delay1mS(500);
		LEDsOff();
		Delay1mS(500);
	}
}

void i2cUpdateClockRate(uint32 c) {
	if (c != I2CClockRate) {
		Delay1mS(10);
		I2CClockRate = c;
		InitI2C(1);
	}
} // i2cUpdateClockRate

void i2c_er_handler(uint8 I2CCurr) {
	// Original source unknown but based on those in baseflight by TimeCop
	volatile uint32 SR1Register, SR2Register;
	I2CPortDef * d;

	d = &I2CPorts[I2CCurr];

	SR1Register = d->I2C->SR1;
	if (SR1Register & 0x0f00) { //an error
		// I2C1error.error = ((SR1Register & 0x0f00) >> 8);        //save error
		// I2C1error.job = job;    //the task
	}
	/* If AF, BERR or ARLO, abandon the current job and commence new if there are jobs */
	if (SR1Register & 0x0700) {
		SR2Register = d->I2C->SR2; //read second status register to clear ADDR if it is set (note that BTF will not be set after a NACK)
		I2C_ITConfig(d->I2C, I2C_IT_BUF, DISABLE); //disable the RXNE/TXE interrupt - prevent the ISR tail-chaining onto the ER (hopefully)
		if (!(SR1Register & 0x0200) && !(d->I2C->CR1 & 0x0200)) { //if we don't have an ARLO error, ensure sending of a stop
			if (d->I2C->CR1 & 0x0100) { //We are currently trying to send a start, this is very bad as start,stop will hang the peripheral
				while (d->I2C->CR1 & 0x0100) {//wait for any start to finish sending
				}
				I2C_GenerateSTOP(d->I2C, ENABLE); //send stop to finalise bus transaction
				while (d->I2C->CR1 & 0x0200) {//wait for stop to finish sending
				}
				InitI2C(I2CCurr); //reset and configure the hardware
			} else {
				I2C_GenerateSTOP(d->I2C, ENABLE); //stop to free up the bus
				I2C_ITConfig(d->I2C, I2C_IT_EVT | I2C_IT_ERR, DISABLE); //Disable EVT and ERR interrupts while bus inactive
			}
		}
	}
	d->I2C->SR1 &= ~0x0f00; //reset all the error bits to clear the interrupt
	i2cState[I2CCurr].busy = false;
} // i2c_er_handler

boolean i2cWriteBlock(uint8 I2CCurr, uint8 addr_, uint8 reg_, uint8 len_,
		uint8 *data) {
	// Original source unknown but based on those in baseflight by TimeCop
	uint8 i, my_data[128]; // TODO: magic number
	uint32 timeout = I2C_DEFAULT_TIMEOUT;
	I2CPortDef * d;

	if (len_ > 127)
		return (false);

	d = &I2CPorts[I2CCurr];

	i2cState[I2CCurr].addr = addr_;
	i2cState[I2CCurr].reg = reg_;
	i2cState[I2CCurr].subaddress_sent = false;
	i2cState[I2CCurr].final_stop = false;
	i2cState[I2CCurr].writing = true;
	i2cState[I2CCurr].reading = false;
	i2cState[I2CCurr].write_p = my_data;
	i2cState[I2CCurr].read_p = my_data;
	i2cState[I2CCurr].bytes = len_;
	i2cState[I2CCurr].busy = true;

	for (i = 0; i < len_; i++)
		my_data[i] = data[i];

	if (!(d->I2C->CR2 & I2C_IT_EVT)) { //if we are restarting the driver
		if (!(d->I2C->CR1 & 0x0100)) { // ensure sending a start
			while (d->I2C->CR1 & 0x0200) { //wait for any stop to finish sending
			}
			I2C_GenerateSTART(d->I2C, ENABLE); //send the start for the new job
		}
		I2C_ITConfig(d->I2C, I2C_IT_EVT | I2C_IT_ERR, ENABLE); //allow the interrupts to fire off again
	}

	while (i2cState[I2CCurr].busy && --timeout > 0) {
	}
	if (timeout == 0) {
		i2cState[I2CCurr].i2cErrors++;
		InitI2C(I2CCurr);
		return (false);
	}

	return (true);
} // i2cWriteBlock

inline boolean i2cWrite(uint8 I2CCurr, uint8 addr_, uint8 reg_, uint8 data) {

	return (i2cWriteBlock(I2CCurr, addr_, reg_, 1, &data));
} // i2cWrite

boolean i2cRead(uint8 I2CCurr, uint8 addr_, uint8 reg_, uint8 len, uint8* buf) {
	// Original source unknown but based on those in baseflight by TimeCop
	uint32 timeout = I2C_DEFAULT_TIMEOUT;
	I2CPortDef * d;

	d = &I2CPorts[I2CCurr];

	i2cState[I2CCurr].addr = addr_;
	i2cState[I2CCurr].reg = reg_;
	i2cState[I2CCurr].writing = false;
	i2cState[I2CCurr].reading = true;
	i2cState[I2CCurr].subaddress_sent = false;
	i2cState[I2CCurr].final_stop = false;
	i2cState[I2CCurr].read_p = buf;
	i2cState[I2CCurr].write_p = buf;
	i2cState[I2CCurr].bytes = len;
	i2cState[I2CCurr].busy = true;

	if (!(d->I2C->CR2 & I2C_IT_EVT)) { //if we are restarting the driver
		if (!(d->I2C->CR1 & 0x0100)) { // ensure sending a start
			while (d->I2C->CR1 & 0x0200) { //wait for any stop to finish sending
			}
			I2C_GenerateSTART(d->I2C, ENABLE); //send the start for the new job
		}
		I2C_ITConfig(d->I2C, I2C_IT_EVT | I2C_IT_ERR, ENABLE); //allow the interrupts to fire off again
	}

	while (i2cState[I2CCurr].busy && (--timeout > 0)) {
	}
	if (timeout == 0) {
		i2cState[I2CCurr].i2cErrors++;
		InitI2C(I2CCurr);
		return (false);
	}

	return (true);
} // i2cRead

void i2c_ev_handler(uint8 I2CCurr) {
	// Original source unknown but based on those in baseflight by TimeCop
	static int8 index; //index is signed -1==send the sub-address
	uint8 SReg_1; //read the status register here
	I2CPortDef * d;

	d = &I2CPorts[I2CCurr];

	SReg_1 = d->I2C->SR1;

	if (SReg_1 & 0x0001) { //we just sent a start - EV5 in reference manual
		d->I2C->CR1 &= ~0x0800; //reset the POS bit so ACK/NACK applied to the current byte
		I2C_AcknowledgeConfig(d->I2C, ENABLE); //make sure ACK is on
		index = 0; //reset the index
		if (i2cState[I2CCurr].reading && (i2cState[I2CCurr].subaddress_sent
				|| (0xff == i2cState[I2CCurr].reg))) { //we have sent the sub-address
			i2cState[I2CCurr].subaddress_sent = true; //make sure this is set in case of no sub-address, so following code runs correctly
			if (i2cState[I2CCurr].bytes == 2)
				d->I2C->CR1 |= 0x0800; //set the POS bit so NACK applied to the final byte in the two byte read
			I2C_Send7bitAddress(d->I2C, i2cState[I2CCurr].addr,
					I2C_Direction_Receiver); //send the address and set hardware mode
		} else { //direction is Tx, or we haven't sent the sub and rep start
			I2C_Send7bitAddress(d->I2C, i2cState[I2CCurr].addr,
					I2C_Direction_Transmitter); //send the address and set hardware mode
			if (i2cState[I2CCurr].reg != 0xff) //0xff as sub-address means it will be ignored, in Tx or Rx mode
				index = -1; //send a sub-address
		}
	} else if (SReg_1 & 0x0002) { //we just sent the address - EV6 in ref manual
		//Read SR1,2 to clear ADDR
		volatile uint8 a;
		__DMB(); // memory fence to control hardware
		if (i2cState[I2CCurr].bytes == 1 && i2cState[I2CCurr].reading
				&& i2cState[I2CCurr].subaddress_sent) { //we are receiving 1 byte - EV6_3
			I2C_AcknowledgeConfig(d->I2C, DISABLE); //turn off ACK
			__DMB();
			a = d->I2C->SR2; //clear ADDR after ACK is turned off
			I2C_GenerateSTOP(d->I2C, ENABLE);
			i2cState[I2CCurr].final_stop = true;
			I2C_ITConfig(d->I2C, I2C_IT_BUF, ENABLE); //allow us to have an EV7
		} else { //EV6 and EV6_1
			a = d->I2C->SR2; //clear the ADDR here
			__DMB();
			if ((i2cState[I2CCurr].bytes == 2) && i2cState[I2CCurr].reading
					&& i2cState[I2CCurr].subaddress_sent) { //rx 2 bytes - EV6_1
				I2C_AcknowledgeConfig(d->I2C, DISABLE); //turn off ACK
				I2C_ITConfig(d->I2C, I2C_IT_BUF, DISABLE); //disable TXE to allow the buffer to fill
			} else if ((i2cState[I2CCurr].bytes == 3)
					&& i2cState[I2CCurr].reading
					&& i2cState[I2CCurr].subaddress_sent) //rx 3 bytes
				I2C_ITConfig(d->I2C, I2C_IT_BUF, DISABLE); //make sure RXNE disabled so we get a BTF in two bytes time
			else
				//receiving greater than three bytes, sending sub-address, or transmitting
				I2C_ITConfig(d->I2C, I2C_IT_BUF, ENABLE);
		}
	} else if (SReg_1 & 0x004) { //Byte transfer finished - EV7_2, EV7_3 or EV8_2
		i2cState[I2CCurr].final_stop = true;
		if (i2cState[I2CCurr].reading && i2cState[I2CCurr].subaddress_sent) { //EV7_2, EV7_3
			if (i2cState[I2CCurr].bytes > 2) { //EV7_2
				I2C_AcknowledgeConfig(d->I2C, DISABLE); //turn off ACK
				i2cState[I2CCurr].read_p[index++] = I2C_ReceiveData(d->I2C); //read data N-2
				I2C_GenerateSTOP(d->I2C, ENABLE);
				i2cState[I2CCurr].final_stop = true; //required to fix hardware
				i2cState[I2CCurr].read_p[index++] = I2C_ReceiveData(d->I2C); //read data N-1
				I2C_ITConfig(d->I2C, I2C_IT_BUF, ENABLE); //enable TXE to allow the final EV7
			} else { //EV7_3
				if (i2cState[I2CCurr].final_stop)
					I2C_GenerateSTOP(d->I2C, ENABLE);
				else
					I2C_GenerateSTART(d->I2C, ENABLE); //repeated start
				i2cState[I2CCurr].read_p[index++] = I2C_ReceiveData(d->I2C); //read data N-1
				i2cState[I2CCurr].read_p[index++] = I2C_ReceiveData(d->I2C); //read data N
				index++; //to show job completed
			}
		} else { //EV8_2, which may be due to a sub-address sent or a write completion
			if (i2cState[I2CCurr].subaddress_sent
					|| (i2cState[I2CCurr].writing)) {
				if (i2cState[I2CCurr].final_stop)
					I2C_GenerateSTOP(d->I2C, ENABLE);
				else
					I2C_GenerateSTART(d->I2C, ENABLE); //repeated start
				index++; //to show that the job is complete
			} else { //send a sub-address
				I2C_GenerateSTART(d->I2C, ENABLE); //repeated Start
				i2cState[I2CCurr].subaddress_sent = true; //this is set back to zero upon completion of the current task
			}
		}
		//we must wait for the start to clear, otherwise we get constant BTF
		while (d->I2C->CR1 & 0x0100) {
		}
	} else if (SReg_1 & 0x0040) { //Byte received - EV7
		i2cState[I2CCurr].read_p[index++] = I2C_ReceiveData(d->I2C);
		if (i2cState[I2CCurr].bytes == (index + 3))
			I2C_ITConfig(d->I2C, I2C_IT_BUF, DISABLE); //disable TXE to allow the buffer to flush so we can get an EV7_2
		if (i2cState[I2CCurr].bytes == index) //We have completed a final EV7
			index++; //to show job is complete
	} else if (SReg_1 & 0x0080) { //Byte transmitted -EV8/EV8_1
		if (index != -1) { //we don't have a sub-address to send
			I2C_SendData(d->I2C, i2cState[I2CCurr].write_p[index++]);
			if (i2cState[I2CCurr].bytes == index) //we have sent all the data
				I2C_ITConfig(d->I2C, I2C_IT_BUF, DISABLE); //disable TXE to allow the buffer to flush
		} else {
			index++;
			I2C_SendData(d->I2C, i2cState[I2CCurr].reg); //send the sub-address
			if (i2cState[I2CCurr].reading || (i2cState[I2CCurr].bytes == 0)) //if receiving or sending 0 bytes, flush now
				I2C_ITConfig(d->I2C, I2C_IT_BUF, DISABLE); //disable TXE to allow the buffer to flush
		}
	}
	if (index == i2cState[I2CCurr].bytes + 1) { //we have completed the current job
		//Completion Tasks go here
		//End of completion tasks
		i2cState[I2CCurr].subaddress_sent = false; //reset this here
		// d->I2C->CR1 &= ~0x0800;   //reset the POS bit so NACK applied to the current byte
		if (i2cState[I2CCurr].final_stop) //if there is a final stop and no more jobs, bus is inactive, disable interrupts to prevent BTF
			I2C_ITConfig(d->I2C, I2C_IT_EVT | I2C_IT_ERR, DISABLE); //Disable EVT and ERR interrupts while bus inactive
		i2cState[I2CCurr].busy = false;
	}
} // i2c_ev_handler

uint32 i2cGetErrors(uint8 I2CCurr) {
	return (i2cState[I2CCurr].i2cErrors);
} // i2cGetErrors

boolean i2cBlockread(uint8 I2CCurr, uint8 d, uint8 l, uint8 *S) {
	return (i2cRead(I2CCurr, d, 0, l, S));
} // blockread

//boolean i2cBlockreadataddrXXXXXX(uint8 I2CCurr, uint8 d, uint8 a, uint8 l, uint8 *S) {
//	return (i2cRead(I2CCurr, d & 0xfe, a, l, S));
//} // blockreadataddr

boolean i2cBlockreadi16v(uint8 I2CCurr, uint8 d, uint8 l, int16 *v, boolean h) {
	uint8 ll, b;
	uint8 S[32];
	boolean r;

	ll = l * 2;
	r = i2cRead(I2CCurr, d, 0, ll, S);

	if (h) {
		for (b = 0; b < ll; b += 2)
			v[b >> 1] = ((int16) S[b] << 8) | S[b + 1];
	} else {
		for (b = 0; b < ll; b += 2)
			v[b >> 1] = ((int16) S[b + 1] << 8) | S[b];
	}

	return (r);
} // readi16v

boolean i2cBlockreadi16vataddr(uint8 I2CCurr, uint8 d, uint8 a, uint8 l,
		int16 *v, boolean h) {
	uint8 b, ll;
	uint8 S[32];

	ll = l * 2;
	//	i2cBlockreadataddr(I2CCurr, d, a, ll, S);
	i2cRead(I2CCurr, d & 0xfe, a, ll, S);

	if (h) {
		for (b = 0; b < ll; b += 2)
			v[b >> 1] = ((int16) S[b] << 8) | S[b + 1];
	} else {
		for (b = 0; b < ll; b += 2)
			v[b >> 1] = ((int16) S[b + 1] << 8) | S[b];
	}

	return (true);
} // readi16v


boolean i2cBlockwrite(uint8 I2CCurr, uint8 d, uint8 l, uint8 *S) {
	uint8 i;
	boolean r = true;

	for (i = 0; i < l; i++)
		r &= i2cWrite(I2CCurr, d, 0, *S++);

	return (r);
} // blockwrite


boolean i2cBlockwriteataddr(uint8 I2CCurr, uint8 d, uint8 a, uint8 l, uint8 *S) {
	uint8 i;
	boolean r = true;

	for (i = 0; i < l; i++)
		r &= i2cWrite(I2CCurr, d, a++, *S++);

	return (r);
} // i2cBlockwriteataddr

inline boolean i2cResponse(uint8 I2CCurr, uint8 d) { // returns true unless there is an I2C timeout????
	uint8 v;

	return (i2cRead(I2CCurr, d, 0, 1, &v));

} // response


