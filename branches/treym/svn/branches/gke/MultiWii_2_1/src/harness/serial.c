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

// USART routines

#include "harness.h"

// Rewritten from AQ original Copyright © 2011 Bill Nesbitt

volatile uint8 TxQ[MAX_SERIAL_PORTS][SERIAL_BUFFER_SIZE] __attribute__((aligned(4)));
volatile int16 TxQTail[MAX_SERIAL_PORTS];
volatile int16 TxQHead[MAX_SERIAL_PORTS];
volatile int16 TxQNewHead[MAX_SERIAL_PORTS];

volatile uint8 RxQ[MAX_SERIAL_PORTS][SERIAL_BUFFER_SIZE] __attribute__((aligned(4)));
volatile int16 RxQTail[MAX_SERIAL_PORTS];
volatile int16 RxQHead[MAX_SERIAL_PORTS];
volatile int16 RxQNewHead[MAX_SERIAL_PORTS];

uint8_t TxCheckSum = 0;

void serialTxDMA(uint8 s) {

	SerialPorts[s].TxDMAStream->M0AR = (uint32) &TxQ[s][TxQHead[s]];

	if (TxQTail[s] > TxQHead[s]) { // Tail not wrapped around yet
		DMA_SetCurrDataCounter(SerialPorts[s].TxDMAStream, TxQTail[s]
				- TxQHead[s]);
		TxQNewHead[s] = TxQTail[s];
	} else {// Tail has wrapped do balance from Head to end of Buffer
		DMA_SetCurrDataCounter(SerialPorts[s].TxDMAStream, SERIAL_BUFFER_SIZE
				- TxQHead[s]);
		TxQNewHead[s] = 0;
	}
	DMA_Cmd(SerialPorts[s].TxDMAStream, ENABLE);

} // serialTxDMA

boolean serialAvailable(uint8 s) {
	boolean r;

	if (SerialPorts[s].DMAUsed) {
		RxQTail[s] = SERIAL_BUFFER_SIZE - DMA_GetCurrDataCounter(
				SerialPorts[s].RxDMAStream);
		r = RxQHead[s] != RxQTail[s];
	} else if (SerialPorts[s].InterruptsUsed)
		r = RxQTail[s] != RxQHead[s];
	else {
		r = (USART_GetFlagStatus(SerialPorts[s].USART, USART_FLAG_RXNE) == SET);
	}

	return (r);
} // serialAvailable

uint8 RxChar(uint8 s) {
	uint8 ch;

	if (SerialPorts[s].DMAUsed || SerialPorts[s].InterruptsUsed) {
		ch = RxQ[s][RxQHead[s]];
		RxQHead[s] = (RxQHead[s] + 1) & (SERIAL_BUFFER_SIZE - 1);
	} else
		ch = USART_ReceiveData(SerialPorts[s].USART);

	return (ch);
} // RxChar

uint8 PollRxChar(uint8 s) {
	uint8 ch;

	if (serialAvailable(s)) {
		ch = RxChar(s);
		return (ch);
	} else {
		return (0);
	}
} // PollRxChar

void TxChar(uint8 s, uint8 ch) {
	int16 NewTail;

	TxCheckSum ^= ch;
	if (SerialPorts[s].DMAUsed || SerialPorts[s].InterruptsUsed) {
		NewTail = (TxQTail[s] + 1) & (SERIAL_BUFFER_SIZE - 1);
		while (NewTail == TxQHead[s]) {
		};

		TxQ[s][TxQTail[s]] = ch;
		// tail points to NEXT free slot
		TxQTail[s] = NewTail;

		if (SerialPorts[s].DMAUsed) {
			if (DMA_GetCmdStatus(SerialPorts[s].TxDMAStream) == DISABLE)
				serialTxDMA(s);
		} else {
			// if TXE then interrupt will be pending
			USART_ITConfig(SerialPorts[s].USART, USART_IT_TXE, ENABLE);
		}
	} else {
		while (USART_GetFlagStatus(SerialPorts[s].USART, USART_FLAG_TXE)
				== RESET) {
		};
		USART_SendData(SerialPorts[s].USART, ch);
	}
} // TxChar

void serialISR(uint8 s) {
	uint8 ch;

	if (USART_GetITStatus(SerialPorts[s].USART, USART_IT_RXNE) == SET) {
		ch = USART_ReceiveData(SerialPorts[s].USART);
		RxQ[s][RxQTail[s]] = ch;
		RxQTail[s] = (RxQTail[s] + 1) & (SERIAL_BUFFER_SIZE - 1);

		if (RxQTail[s] == RxQHead[s]) { // full
			//USART_ITConfig(SerialPorts[s].USART, USART_IT_RXNE, DISABLE);
		}
	}

	if (USART_GetITStatus(SerialPorts[s].USART, USART_IT_TXE) == SET) {
		if (TxQTail[s] != TxQHead[s]) {
			ch = TxQ[s][TxQHead[s]];
			USART_SendData(SerialPorts[s].USART, ch);
			TxQHead[s] = (TxQHead[s] + 1) & (SERIAL_BUFFER_SIZE - 1);
		}
		if (TxQHead[s] == TxQTail[s])
			USART_ITConfig(SerialPorts[s].USART, USART_IT_TXE, DISABLE);
	}
} // serialISR

void InitSerial(void) {

	//TODO:	serialPortInit(0, 115200);

} // InitSerial

