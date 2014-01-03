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

inline int16 SRS16(int16 x, uint8 s) {
	return ((x < 0) ? -((-x) >> s) : (x >> s));
} // SRS16

inline int32 SRS32(int32 x, uint8 s) {
	return ((x < 0) ? -((-x) >> s) : (x >> s));
} // SRS32

uint8 CurrMaxPWMOutputs = 6;

inline boolean digitalRead(PinDef * d) {
	return (GPIO_ReadInputDataBit(d->Port, d->Pin));
} // digitalRead

inline void digitalWrite(PinDef * d, uint8 m) {

#ifdef STM32F1
	if (m)
	d->Port->BSRR = d->Pin;
	else
	d->Port->BRR = d->Pin;
#else
	if (m)
		d->Port->BSRRL = d->Pin;
	else
		d->Port->BSRRH = d->Pin;
#endif // STM32F1
} // digitalWrite

inline void digitalToggle(PinDef * d) {
	d->Port->ODR ^= d->Pin;
} // digitalToggle


#ifdef STM32F1

#define AIRCR_VECTKEY_MASK    ((uint32)0x05FA0000)

void systemReset(boolean toBootloader) {
	if (toBootloader) {
		// 1FFFF000 -> 20000200 -> SP
		// 1FFFF004 -> 1FFFF021 -> PC
		*((uint32 *) 0x20004FF0) = 0xDEADBEEF; // 20KB STM32F103
	}

	// Generate system reset
	SCB->AIRCR = AIRCR_VECTKEY_MASK | (uint32) 0x04;
}

#endif // STM32F1
void pinInit(PinDef * d) {
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = d->Pin;
	GPIO_InitStructure.GPIO_Mode = d->Mode;
#ifndef STM32F1
	GPIO_InitStructure.GPIO_OType = d->OType;
	GPIO_InitStructure.GPIO_PuPd = d->PuPd;
#endif
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(d->Port, &GPIO_InitStructure);
} // pinInit

void NVICDisable(IRQn_Type ISR) {
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel = ISR;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
	NVIC_Init(&NVIC_InitStructure);

} // NVICDisable

TIM_ICInitTypeDef TIM_ICInitStructure = { 0, }; // global for rc

void InitRCPin(uint8 PPMInputs) {
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure = { 0, };
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	uint8 i;
	PinDef * u;

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_ICStructInit(&TIM_ICInitStructure);

	// Common initialisation

	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; // JIH
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF; // TODO: 32 bit period?
#ifdef STM32F1
	TIM_TimeBaseStructure.TIM_Prescaler = TIMER_PS - 1;
#else
	TIM_TimeBaseStructure.TIM_Prescaler = (TIMER_PS / 2) - 1;
#endif

	// Pin specific
	for (i = 0; i < PPMInputs; i++) {
		u = &RCPins[i];

		pinInit(u);
#ifndef STM32F1
		GPIO_PinAFConfig(u->Port, u->PinSource, u->Timer.TimAF);
#endif
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_InitStructure.NVIC_IRQChannel = u->PinISR;
		NVIC_Init(&NVIC_InitStructure);

		TIM_TimeBaseInit(u->Timer.Tim, &TIM_TimeBaseStructure);

		TIM_ICStructInit(&TIM_ICInitStructure);
		TIM_ICInitStructure.TIM_Channel = u->Timer.Channel;
		TIM_ICInit(u->Timer.Tim, &TIM_ICInitStructure);

		//u->Timer.Tim->DIER |= u->Timer.CC; // TIM_ITConfig ENABLE Channel
		TIM_ITConfig(u->Timer.Tim, u->Timer.CC, ENABLE);

	}
	for (i = 0; i < PPMInputs; i++)
		TIM_Cmd(RCPins[i].Timer.Tim, ENABLE);

} // InitRCPin


void InitI2C(uint8 I2CCurr) {
	// Original source unknown but based on those in baseflight by TimeCop
	NVIC_InitTypeDef NVIC_InitStructure;
	I2C_InitTypeDef I2C_InitStructure;
	I2CPortDef * d;

	d = &I2CPorts[I2CCurr];

	if (d->Used) {

#ifndef STM32F1
		GPIO_PinAFConfig(d->SCLPort, d->SCLPinSource, d->I2C_AF);
		GPIO_PinAFConfig(d->SDAPort, d->SDAPinSource, d->I2C_AF);
#endif

		i2cUnstick(I2CCurr); // attempt to unfreeze slave(s) - initialises pins
		if (I2CFatal)
			return;

		I2C_DeInit(d->I2C);
		I2C_StructInit(&I2C_InitStructure);

		I2C_ITConfig(d->I2C, I2C_IT_EVT | I2C_IT_ERR, DISABLE); //Enable EVT and ERR interrupts - they are enabled by the first request
		I2C_InitStructure.I2C_ClockSpeed = I2CClockRate;
		I2C_Cmd(d->I2C, ENABLE);
		I2C_Init(d->I2C, &I2C_InitStructure);

		NVIC_PriorityGroupConfig(0x500);

		// I2C ER Interrupt
		NVIC_InitStructure.NVIC_IRQChannel = I2C2_ER_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
		// I2C EV Interrupt
		NVIC_InitStructure.NVIC_IRQChannel = I2C2_EV_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_Init(&NVIC_InitStructure);
	}
} // InitI2C

void i2cUnstick(uint8 I2CCurr) {
	// Original source unknown but based on those in baseflight by TimeCop
	GPIO_InitTypeDef GPIO_InitStructure;
	uint8 i;
	uint16 Timeout;
	I2CPortDef * d;

	uint8 I2CDelayuS;

	if (I2CClockRate == 400000)
		I2CDelayuS = 10;
	else
		I2CDelayuS = 40;

	d = &I2CPorts[I2CCurr];

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = d->SCLPin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
#ifdef STM32F1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
#else
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
#endif
	GPIO_Init(d->SCLPort, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = d->SDAPin;
	GPIO_Init(d->SDAPort, &GPIO_InitStructure);

	GPIO_SetBits(d->SCLPort, d->SCLPin);
	GPIO_SetBits(d->SDAPort, d->SDAPin);

	Timeout = mSClock() + 100;
	for (i = 0; i < 8; i++) { // Wait for any clock stretching to finish
		while (!GPIO_ReadInputDataBit(d->SCLPort, d->SCLPin)) {
			Delay1uS(I2CDelayuS);
			if (mSClock() > Timeout) {
				I2CFatal = true;
				return;
			}
		}

		GPIO_ResetBits(d->SCLPort, d->SCLPin);
		Delay1uS(I2CDelayuS);

		GPIO_SetBits(d->SCLPort, d->SCLPin);
		Delay1uS(I2CDelayuS);
	}

	GPIO_ResetBits(d->SDAPort, d->SDAPin);
	Delay1uS(I2CDelayuS);
	GPIO_ResetBits(d->SCLPort, d->SCLPin);
	Delay1uS(I2CDelayuS);
	GPIO_SetBits(d->SCLPort, d->SCLPin);
	Delay1uS(I2CDelayuS);
	GPIO_SetBits(d->SDAPort, d->SDAPin);

	GPIO_InitStructure.GPIO_Pin = d->SCLPin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
#ifdef STM32F1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
#else
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
#endif
	GPIO_Init(d->SCLPort, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = d->SDAPin;
	GPIO_Init(d->SDAPort, &GPIO_InitStructure);

} // i2cUnstick

void InitPWMPins(uint8 s) {
        TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
        TIM_OCInitTypeDef TIM_OCInitStructure = { 0, };
        uint8 i;
        PinDef * u;

        for (i = 0; i < MAX_PWM_OUTPUTS; i++)
                TIM_DeInit(PWMPins[i].Timer.Tim);

        TIM_OCStructInit(&TIM_OCInitStructure);
        TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
        TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
        TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
        TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

        TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

        for (i = 0; i < s; i++) {

                u = &PWMPins[i];
                pinInit(u);

#ifdef STM32F1
                TIM_TimeBaseStructure.TIM_Prescaler = TIMER_PS - 1; // 1uSec tick
#else
                GPIO_PinAFConfig(u->Port, u->PinSource, u->Timer.TimAF);
                if ((u->Timer.Tim == TIM1) || (u->Timer.Tim == TIM8))
                        TIM_TimeBaseStructure.TIM_Prescaler = TIMER_PS - 1;
                else
                        TIM_TimeBaseStructure.TIM_Prescaler = (TIMER_PS / 2) - 1;
#endif
                TIM_TimeBaseStructure.TIM_Period = u->Period - 1;
                TIM_TimeBaseInit(u->Timer.Tim, &TIM_TimeBaseStructure);

                TIM_OCInitStructure.TIM_Pulse = u->Width;

                switch (u->Timer.Channel) { // TODO: must be a more direct way
                case TIM_Channel_1:
                        TIM_OC1Init(u->Timer.Tim, &TIM_OCInitStructure);
                        TIM_OC1PreloadConfig(u->Timer.Tim, TIM_OCPreload_Enable);
                        break;
                case TIM_Channel_2:
                        TIM_OC2Init(u->Timer.Tim, &TIM_OCInitStructure);
                        TIM_OC2PreloadConfig(u->Timer.Tim, TIM_OCPreload_Enable);
                        break;
                case TIM_Channel_3:
                        TIM_OC3Init(u->Timer.Tim, &TIM_OCInitStructure);
                        TIM_OC3PreloadConfig(u->Timer.Tim, TIM_OCPreload_Enable);
                        break;
                case TIM_Channel_4:
                        TIM_OC4Init(u->Timer.Tim, &TIM_OCInitStructure);
                        TIM_OC4PreloadConfig(u->Timer.Tim, TIM_OCPreload_Enable);
                        break;
                } // switch Timer.Channel
        }

        for (i = 0; i < s; i++)
                TIM_ARRPreloadConfig(PWMPins[i].Timer.Tim, ENABLE);

        for (i = 0; i < s; i++) {
                u = &PWMPins[i];
                TIM_Cmd(u->Timer.Tim, ENABLE);
                TIM_CtrlPWMOutputs(u->Timer.Tim, ENABLE);
        }
} // InitPWMPins

void InitSerialPort(uint8 s) {
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	SerialPortDef * u;

	u = &SerialPorts[s];

	if (u->Used) {

		GPIO_StructInit(&GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
#ifdef STM32F1
		GPIO_InitStructure.GPIO_Pin = u->TxPin;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_Init(u->Port, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin = u->RxPin;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
		GPIO_Init(u->Port, &GPIO_InitStructure);
#else
		GPIO_StructInit(&GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = u->TxPin | u->RxPin;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(u->Port, &GPIO_InitStructure);

		GPIO_PinAFConfig(u->Port, u->TxPinSource, u->USART_AF);
		GPIO_PinAFConfig(u->Port, u->RxPinSource, u->USART_AF);
#endif
		USART_StructInit(&USART_InitStructure);
		//USART_InitStruct->USART_Parity = USART_Parity_No;
		USART_InitStructure.USART_BaudRate = u->Baud;
		USART_Init(u->USART, &USART_InitStructure);

		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

		TxQTail[s] = TxQHead[s] = TxQNewHead[s] = 0;

		if (u->DMAUsed) {
			// Common
			DMA_StructInit(&DMA_InitStructure);
			DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
			DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
			DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32) &u->USART->DR;
			DMA_InitStructure.DMA_BufferSize = SERIAL_BUFFER_SIZE;

			// Receive DMA
			DMA_DeInit(u->RxDMAStream);

			DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32) &u->USART->DR;
			DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
#ifdef STM32F1
			DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
			DMA_InitStructure.DMA_MemoryBaseAddr = (uint32) RxQ[s];
#else
			while (DMA_GetCmdStatus(u->RxDMAStream) != DISABLE) {
			};
			DMA_InitStructure.DMA_Channel = u->DMAChannel;
			DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
			DMA_InitStructure.DMA_Memory0BaseAddr = (uint32) RxQ[s];
#endif
			DMA_Init(u->RxDMAStream, &DMA_InitStructure);
			DMA_Cmd(u->RxDMAStream, ENABLE);
			USART_DMACmd(u->USART, USART_DMAReq_Rx, ENABLE);

			RxQTail[s] = RxQHead[s] = SERIAL_BUFFER_SIZE
					- DMA_GetCurrDataCounter(u->RxDMAStream);

			// Transmit DMA
			NVIC_InitStructure.NVIC_IRQChannel = u->TxDMAISR;
			NVIC_Init(&NVIC_InitStructure);

			DMA_DeInit(u->TxDMAStream);
#ifdef STM32F1
			DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
#else
			while (DMA_GetCmdStatus(u->TxDMAStream) != DISABLE) {
			};
			DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
			//	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32) TxQ[s];
#endif
			DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
			DMA_Init(u->TxDMAStream, &DMA_InitStructure);

			DMA_SetCurrDataCounter(u->TxDMAStream, 0);
			DMA_ITConfig(u->TxDMAStream, DMA_IT_TC, ENABLE);

			USART_DMACmd(u->USART, USART_DMAReq_Tx, ENABLE);
		} else if (u->InterruptsUsed) {
			RxQTail[s] = RxQHead[s] = 0;
			NVIC_InitStructure.NVIC_IRQChannel = u->ISR;
			NVIC_Init(&NVIC_InitStructure);

			USART_ITConfig(u->USART, USART_IT_RXNE, ENABLE);
		}
		USART_Cmd(u->USART, ENABLE);
	}

} // InitSerialPort

void InitAnalogPins(void) {
	uint8 a;
	ADC_InitTypeDef ADC_InitStructure;
#ifndef STM32F1
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
#endif
	DMA_InitTypeDef DMA_InitStructure;
	AnalogPinDef * u, *ux;

	if (ANALOG_CHANNELS > 0) {
		// all pins already configured as AIN

		ux = &AnalogPins[0];

		DMA_DeInit(ux->DMAStream);
#ifndef STM32F1
		while (DMA_GetCmdStatus(ux->DMAStream) != DISABLE) {
		};
#endif
		DMA_StructInit(&DMA_InitStructure);
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32) &ux->ADCx->DR;
		DMA_InitStructure.DMA_BufferSize = ANALOG_CHANNELS;
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_InitStructure.DMA_PeripheralDataSize
				= DMA_PeripheralDataSize_HalfWord;
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
		DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
		DMA_InitStructure.DMA_Priority = DMA_Priority_High;
#ifdef STM32F1
		DMA_InitStructure.DMA_MemoryBaseAddr = (uint32) ADCValues;
#else
		DMA_InitStructure.DMA_Channel = ux->DMAChannel;
		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32) ADCValues;
		//DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
#endif
		DMA_Init(ux->DMAStream, &DMA_InitStructure);
		DMA_Cmd(ux->DMAStream, ENABLE);

#ifndef STM32F1
		ADC_CommonStructInit(&ADC_CommonInitStructure);
		ADC_CommonInit(&ADC_CommonInitStructure);
#endif
		ADC_StructInit(&ADC_InitStructure);
		ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
		ADC_InitStructure.ADC_ScanConvMode = ENABLE;
#ifdef STM32F1
		ADC_InitStructure.ADC_NbrOfChannel = ANALOG_CHANNELS;
#else
		ADC_InitStructure.ADC_NbrOfConversion = ANALOG_CHANNELS;
#endif
		ADC_Init(ux->ADCx, &ADC_InitStructure);

		for (a = 0; a < ANALOG_CHANNELS; a++) {
			u = &AnalogPins[a];
#ifdef STM32F1
			ADC_RegularChannelConfig(u->ADCx, u->ADCChannel, u->Rank,
					ADC_SampleTime_28Cycles5);
#else
			ADC_RegularChannelConfig(u->ADCx, u->ADCChannel, u->Rank,
					ADC_SampleTime_28Cycles);
#endif
		}
#ifndef STM32F1
		ADC_DMARequestAfterLastTransferCmd(ux->ADCx, ENABLE);
#endif

		ADC_DMACmd(ux->ADCx, ENABLE);
		ADC_Cmd(ux->ADCx, ENABLE);

#ifdef STM32F1
		a = 2;
		while (a-- > 0) {
			Delay1mS(5);
			ADC_ResetCalibration(ux->ADCx);
			while (ADC_GetResetCalibrationStatus(ux->ADCx)) {
			}
			ADC_StartCalibration(ux->ADCx);
			while (ADC_GetCalibrationStatus(ux->ADCx)) {
			}
		}
		ADC_SoftwareStartConvCmd(ux->ADCx, ENABLE);
#else
		ADC_SoftwareStartConv(ux->ADCx);
#endif // STM32F1
	}

} // InitAnalogPins

void InitPortCombination(void) {

	if (ParallelPPM) {
		CurrMaxPWMOutputs = 6;
		InitPWMPins(CurrMaxPWMOutputs);
		InitRCPin(8);
	} else {
		CurrMaxPWMOutputs = 10;
		InitPWMPins(CurrMaxPWMOutputs);
		if (CompoundPPM)
			InitRCPin(1);
		else
			InitSerialPort(RCSerial);
	}

} // InitPortCombination

void InitHarness(void) {
	GPIO_InitTypeDef GPIO_InitStructure;
	uint8 i;

	// Using all ports - could be generalised but .... later
#ifdef STM32F1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
#else
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
#endif // STM32F1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);

#ifdef STM32F1
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	//RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);
#else
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
#endif

	USART_DeInit(USART1);
#if (MAX_SERIAL_PORTS >1)
	USART_DeInit(USART2);
#endif

#ifdef STM32F1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
#if (MAX_SERIAL_PORTS >1)
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
#endif
#else
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
#endif
	RCC_ClearFlag();

	// Make all GPIO input by default to save power and reduce noise
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_Init(GPIOC, &GPIO_InitStructure);

#ifdef STM32F1
	// Turn off JTAG port used for LEDs
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
#else

#endif // STM32F1
	NVIC_PriorityGroupConfig(0x500);

	InitI2C(1);

	for (i = 0; i < MAX_LEDS; i++) {
		pinInit(&LEDPins[i]);
		digitalWrite(&LEDPins[i], 1);
	}

	for (i = 0; i < MAX_GPIO_PINS; i++)
		pinInit(&GPIOPins[i]);
	BeeperOff();

	Delay1mS(10);

	InitSerialPort(0);

	for (i = 0; i < MAX_SPIIO_PINS; i++)
		pinInit(&SPIIOPins[i]);
	digitalWrite(&SPIIOPins[SPICSSel], 1);
	digitalWrite(&SPIIOPins[SPISCLSel], 1);
	digitalWrite(&SPIIOPins[SPISDASel], 1); //zzz

	InitAnalogPins();

	Delay1mS(100);

} // InitHarness

