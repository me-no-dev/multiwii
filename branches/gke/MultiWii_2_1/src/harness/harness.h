
#ifndef _harness_h
#define _harness_h

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

#define ADC_RANGE 4096.0f	// 12bit

#include "../def.h"

#ifdef STM32F1

#define TIMER_PS 72
#include "stm32f10x.h"
#include "system_stm32f10x.h"

#else

#define STM32F4
#define TIMER_PS 168
#include "stm32f4xx.h"
#include "system_stm32f4xx.h"

#endif // STM32F1

// Additional Types

typedef unsigned char uint8;
typedef signed char int8;
typedef uint16_t uint16;
typedef short int16;
typedef int int32;
typedef int64_t int64;
typedef uint32_t uint32;
typedef uint8 boolean;
typedef float real32;
typedef double real64;
typedef void (*voidFuncPtr)(void);
typedef void (*i16FuncPtr)(int16 * v);
typedef unsigned char BYTE;
typedef int DWORD;

typedef union {
	int16 i32;
	uint8 u8[4];
} i32u8u;

typedef union {
	int16 i16;
	uint8 u8[2];
} i16u8u;

typedef union {
	int16 i8;
	uint8 u8;
} i8u8u;

typedef union {
	int16 i32;
	int8 i8[4];
} i32i8u;

typedef union {
	int16 i16;
	int8 i8[2];
} i16i8u;

#define true 1
#define false 0

// Macros

#define Abs(i)		(((i)<0) ? -(i) : (i))
#define Sign(i)		(((i)<0) ? -1 : 1)
#define Sqr(r)		( (r) * (r) )

#define Max(i,j) 	((i<j) ? j : i)
#define Min(i,j) 	((i<j) ? i : j )
#define Decay1(i) 	(((i) < 0) ? (i+1) : (((i) > 0) ? (i-1) : 0))

#define Limit(i,l,u) 	(((i) < l) ? l : (((i) > u) ? u : (i)))
#define Limit1(i,l) 	(((i) < -(l)) ? -(l) : (((i) > (l)) ? (l) : (i)))

#define bitSet(S,b) 			((uint8)(S|=(1<<b)))
#define bitClear(S,b) 			((uint8)(S&=(~(1<<b))))
#define bitIsSet(S,b) 			((uint8)((S>>b)&1))
#define bitIsClear(S,b) 		((uint8)(!(S>>b)&1))
#define bitInvert(S,b) 		((uint8)(S^=(1<<b)))

#define ToPercent(n, m) ((100.0f*(real32)n)/m)
#define FromPercent(n, m) (m*(real32)n*0.01f)

//________________________________________________________________________________________

// Useful Constants

#define ASCII_NUL 0
#define ASCII_SOH 1
#define ASCII_EOT 4
#define ASCII_ACK 6
#define ASCII_HT 9
#define ASCII_LF 10
#define ASCII_CR 13
#define ASCII_NAK 21
#define ASCII_ESC 27

#define PI   		3.14159265358979323846f

#define RadiansToDegrees(r) ((r)*(180.0f/PI))
#define DegreesToRadians(r)	((r)*(PI/180.0f))
#define DEG_TO_RAD	(PI/180.0f)
#define RAD_TO_DEG	(180.0f/PI)

#define HALF_PI 		(PI*0.5f)
#define QUARTER_PI 	(PI*0.2f)
#define SIXTH_PI		(PI/6.0f)
#define TWO_PI 		(PI*2.0f)

//______________________________________________________________________________________________

// analog.c

extern void analogpinInit(uint8 a);
extern inline int32 analogRead(uint8 c);
extern void InitAnalog(void);
extern volatile uint16 ADCValues[];

//______________________________________________________________________________________________

// clock.c

extern volatile uint32 TicksuS;

extern void cycleCounterInit(void);
extern uint32 uSClock(void);
extern void Delay1uS(uint16);
extern uint32 mSClock(void);
extern void Delay1mS(uint16);


//______________________________________________________________________________________________

// isr.c

#define SpektrumISR USART2_IRQHandler
#define TelemetryISR USART3_IRQHandler
#define GPSISR USART4_IRQHandler

#define RISING 0
#define FALLING 1
#define CHANGE 3

#define DWT_CTRL	(*(volatile uint32 *)0xE0001000)
#define DWT_CYCCNT	((volatile uint32 *)0xE0001004)
#define CYCCNTENA	(1 << 0)

extern void NMI_Handler(void);
extern void HardFault_Handler(void);
extern void MemManage_Handler(void);
extern void BusFault_Handler(void);
extern void UsageFault_Handler(void);
extern void SVC_Handler(void);
extern void DebugMon_Handler(void);
extern void PendSV_Handler(void);
extern void SysTick_Handler(void);

// Clocks

extern volatile uint32 sysTickUptime;
extern volatile uint32 sysTickCycleCounter;

extern void SysTick_Handler(void);

extern void GPSInISR(void);

//______________________________________________________________________________________________

// i2c.c

typedef struct { // TODO: possibly combine with Port def
	boolean error;
	boolean busy;
	uint32 i2cErrors;
	boolean subaddress_sent;
	boolean final_stop;
	boolean writing;
	boolean reading;
	uint8 addr; // 8bit
	uint8 reg;
	uint16 bytes;
	uint8* write_p;
	uint8* read_p;
} i2cStateDef;

typedef struct {
	boolean Used;
	I2C_TypeDef* I2C;
	uint8 I2CNo;
	GPIO_TypeDef* SCLPort;
	uint16 SCLPin;
	uint8 SCLPinSource;
	GPIO_TypeDef* SDAPort;
	uint16 SDAPin;
	uint8 SDAPinSource;
	uint8 I2C_AF;
} I2CPortDef;

extern void Catastrophe(void);

extern void i2c_er_handler(uint8 I2CCurr);
extern void i2c_ev_handler(uint8 I2CCurr);

extern void i2cUpdateClockRate(uint32 c);
extern boolean i2cWriteBlock(uint8 I2CCurr, uint8 addr_, uint8 reg_, uint8 len_,uint8* buf);
extern inline boolean i2cWrite(uint8 I2CCurr, uint8 addr_, uint8 reg_, uint8 data);
extern boolean i2cRead(uint8 I2CCurr, uint8 addr_, uint8 reg_, uint8 len, uint8* buf);
extern uint32 i2cGetErrors(uint8 I2CCurr);
extern boolean i2cBlockread(uint8 I2CCurr, uint8 d, uint8 l, uint8 *S);
extern boolean i2cBlockreadi16v(uint8 I2CCurr, uint8 d, uint8 l, int16 *v, boolean h);
extern boolean i2cBlockreadi16vataddr(uint8 I2CCurr, uint8 d, uint8 a, uint8 l,
		int16 *v, boolean h);
extern boolean i2cBlockwrite(uint8 I2CCurr, uint8 d, uint8 l, uint8 *S);
extern boolean i2cBlockwriteataddr(uint8 I2CCurr, uint8 d, uint8 a, uint8 l, uint8 *S);
extern inline boolean i2cResponse(uint8 I2CCurr, uint8 d);

extern void InitI2C(uint8 I2CCurr);

extern volatile i2cStateDef i2cState[];

extern uint32 I2CClockRate;
extern boolean I2CFatal;

//______________________________________________________________________________________________

// LEDs.c

extern void SaveLEDs(void);
extern void RestoreLEDs(void);
extern void LEDsOn(void);
extern void LEDsOff(void);
extern void LEDToggle(uint8 l);
extern void LEDOn(uint8 l);
extern void LEDOff(uint8 l);
extern void LEDChaser(void);
extern boolean LEDIsOn(uint8 l);

extern void DoLEDs(void);
extern void PowerOutput(uint8);
extern void LEDsAndBuzzer(uint8 s);
extern void InitLEDs(void);

extern inline void BeeperOff(void);
extern inline void BeeperOn(void);
extern inline void BeeperToggle(void);
extern inline boolean BeeperIsOn(void);



//______________________________________________________________________________________________

// nonvolatile.c

#define EEPROM_ID 0xa0

#define NV_EEPROM_SIZE 65536
#define NV_GHOST_SIZE 1024

void ReadBlockNV(uint32 a, uint16 l, uint8 * v);
boolean WriteBlockNV(uint32 a, uint16 l, uint8 * v);

// drives.c

#define MAX_DRIVES		10		// for telemetry NOT max PWM channels
#define PWM_PERIOD    	(1000000/450) // pulse period (400Hz)
#define PWM_WIDTH		(1000) // 1ms pulse width
#define PWM_MIN			(800)
#define PWM_MAX			(2200) // must preserve a synchronisation gap for ESCs
#define PWM_PERIOD_DIGITAL  (1000000/200) // pulse period for digital servo
#define PWM_PERIOD_ANALOG  (22500) //1000000/50) // pulse period for analog servo
#ifdef USE_DIGITAL_SERVOS
#define PWM_PERIOD_SERVO PWM_PERIOD_DIGITAL
#else
#define PWM_PERIOD_SERVO PWM_PERIOD_ANALOG
#endif

extern uint8 CurrMaxPWMOutputs;

extern void InitPWM(void);
extern void pwmWrite(uint8 p, uint16 pulseWidth);
extern void pwmSetPeriod(uint8 channel, uint32 period);

extern void ShowESCType(uint8 s);
extern uint8 I2CESCLimit(int16);


//__________________________________________________________________________________________

// rc.c

#define ENABLE_STICK_CHANGE_FAILSAFE

#define RC_MAX_CHANNELS 20
extern uint8 NoOfPPMInputs[];



// PPM
#define MIN_PPM_SYNC_PAUSE_US 2800

// Spektrum
#define MIN_SPEK_SYNC_PAUSE_US 5000
#define SPEK_MAX_CHANNEL 7
#define SPEK_FRAME_SIZE 16

typedef struct {
	uint8 Pin;
	int32 PrevEdge;
	boolean PrevState;
	boolean State;
	int32 RisingEdge;
	int32 FallingEdge;
	int32 Raw;
} RCInpDef;

extern void CheckSpektrumSBus(void);
extern void DoRCSerial(uint32 Now);
extern void DoSpektrum(void);
extern uint8 SpekChanShift;
extern uint8 SpekChanMask;
extern uint8 spekFrame[];

// Futaba SBus
#define SBUS_FRAME_SIZE 25

extern void DoSBus(void);
extern uint8 SBus[];
extern uint8 SBusIndex;

// General
extern void InitRC(void);
extern void InitRCPins(uint8 PPMInputs);
extern void CheckRC(void);
extern void MapRC(void);
extern void CheckSticksHaveChanged(void);
extern void UpdateControls(void);
extern void CaptureTrims(void);
extern void CheckThrottleMoved(void);
extern void ReceiverTest(uint8 s);

// ISR

extern void RCSerialISR(uint32 Now);
extern void RCParallelISR(TIM_TypeDef *tim);
extern void SpektrumSBusISR(uint8 s);

extern volatile uint8 RCFrame[];
extern uint8 FrameIndex;

extern RCInpDef RCInp[];
extern uint32 RCFrameInterval, RCLastFrame;
extern uint8 Channel;
extern int8 SignalCount;
extern uint16 RCGlitches;

extern uint8 Map[], RMap[];
extern real32 RC[], RCp[], Trim[];
extern uint8 NoOfRCChannels;
extern uint8 NoOfControls;
extern int8 RCStart;

//__________________________________________________________________________________________

// serial.c

#define SERIAL_BUFFER_SIZE    512

extern volatile uint8 TxQ[][SERIAL_BUFFER_SIZE];
extern volatile int16 TxQTail[];
extern volatile int16 TxQHead[];
extern volatile int16 TxQNewHead[];

extern volatile uint8 RxQ[][SERIAL_BUFFER_SIZE];
extern volatile int16 RxQTail[];
extern volatile int16 RxQHead[];
extern volatile int16 RxQNewHead[];

extern volatile uint32 RxDMAPos[];

extern void InitSerial(void);
extern void serialISR(uint8 s);

extern void TxChar(uint8 s, uint8 ch);
extern boolean serialAvailable(uint8 s);
extern uint8 PollRxChar(uint8 s);
extern uint8 RxChar(uint8 s);

extern void serialTxDMA(uint8 s);

//__________________________________________________________________________________________

// harness.c

#define I2CLite 1
#define I2CIMU	1
#define I2CBaro	1
#define I2CAcc	1
#define I2CGyro	1
#define I2CMag	1
#define I2CTemp	1
#define I2CESC	1
#define I2CEEPROM 1

#define TelemetrySerial 0
#define GPSSerial 0 // Using Telemetry Rx 1
#define RCSerial 1 // 2
enum GPIOSelectors {
	BeeperSel,
	ArmedSel,
	UnusedForLite,
	MPU6050IntSel,
	HMC5883DRdySel,
	LandingSel,
	Aux1Sel,
	Aux2Sel,
	ProbeSel
};
enum GPIOSelectorsBF {
	BaroXCLRSel = 2, BaroEOCSel = 3
};

enum ADCSelectors {
	RangefinderAnalogSel,
	BattCurrentAnalogSel,
	BattVoltsAnalogSel,
	RollAnalogSel,
	PitchAnalogSel,
	YawAnalogSel
};
#define TopAnalogSel YawAnalogSel

enum LEDSelectors {
	LEDYellowSel, LEDRedSel, LEDBlueSel, LEDGreenSel,
};
#define TopLEDSel Aux3Sel

typedef struct {
	TIM_TypeDef *Tim;
	uint16 Channel;
	uint16 CC;
#ifdef STM32F1
	volatile uint16 * CCR;
#else
	volatile uint32 * CCR;
#endif
	uint8 TimAF;
} TIMChannelDef;

extern TIM_ICInitTypeDef TIM_ICInitStructure;

typedef struct {
	GPIO_TypeDef* Port;
	uint16 Pin;
	uint16 PinSource;
	GPIOMode_TypeDef Mode;
#ifdef STM32F1
	uint16 U1, U2;
#else
	GPIOOType_TypeDef OType;
	GPIOPuPd_TypeDef PuPd;
#endif

	boolean TimerUsed;
	TIMChannelDef Timer;

	IRQn_Type PinISR;

	uint32 Period;
	uint16 MinPW;
	uint16 MaxPW;
	uint16 Width;
	int8 Sense; // for servo directions
} PinDef;

typedef struct {
	ADC_TypeDef* ADCx;
	GPIO_TypeDef* Port;
	uint16 Pin;
	uint32 ADCChannel;
	uint32 DMAChannel;
#ifdef STM32F1
	DMA_Channel_TypeDef * DMAStream;
#else
	DMA_Stream_TypeDef* DMAStream;
#endif
	uint8 Rank;
} AnalogPinDef;

#ifdef STM32F1

typedef struct {
	boolean Used;
	USART_TypeDef* USART;
	uint8 USART_AF;
	GPIO_TypeDef* Port;
	uint16 TxPin;
	uint16 TxPinSource;
	uint16 RxPin;
	uint16 RxPinSource;
	//	uint32 Clk;
	//	uint32 USARTClk;
	//	uint32 AF;
	boolean InterruptsUsed;
	IRQn_Type ISR;

	boolean DMAUsed;
	uint32 DMAChannel;
	DMA_Channel_TypeDef * TxDMAStream;
	IRQn_Type TxDMAISR;
	DMA_Channel_TypeDef * RxDMAStream;
	uint32 Baud;

}SerialPortDef;

#else

typedef struct {
	boolean Used;
	USART_TypeDef* USART;
	uint8 USART_AF;
	GPIO_TypeDef* Port;
	uint16 TxPin;
	uint16 TxPinSource;
	uint16 RxPin;
	uint16 RxPinSource;

	boolean InterruptsUsed;
	IRQn_Type ISR;

	boolean DMAUsed;
	uint32 DMAChannel;
	DMA_Stream_TypeDef * TxDMAStream;
	IRQn_Type TxDMAISR;
	DMA_Stream_TypeDef * RxDMAStream;
	uint32 Baud;

} SerialPortDef;

#endif // STM32F1

#define MAX_RC_INPS 8
#define ANALOG_CHANNELS 6
#define MAX_GPIO_PINS 10
#define MAX_LEDS 4

enum SPIIOSelectors {
	SPICSSel, SPISCLSel, SPISDASel
};
extern PinDef SPIIOPins[];
#define MAX_SPIIO_PINS 3

#define MAX_I2C_PORTS 2
#define MAX_SERIAL_PORTS 2
#define MAX_PWM_OUTPUTS 10


extern void systemReset(boolean toBootloader);

extern void InitHarness(void);
extern void InitPortCombination(void);
extern void pinInit(PinDef * d);
extern void serialPortInit(uint8 s);
extern void InitPWMPins(uint8 s);

extern void InitI2C(uint8 I2CCurr);
extern void i2cUnstick(uint8 I2CCurr);

extern void InitClocks(void);

extern inline boolean digitalRead(PinDef * d);
extern inline void digitalWrite(PinDef * d, uint8 m);
extern inline void digitalToggle(PinDef * d);

extern PinDef RCPins[];
extern I2CPortDef I2CPorts[];
extern PinDef SPIIOPins[];
extern AnalogPinDef AnalogPins[];
extern PinDef LEDPins[];
extern PinDef GPIOPins[];
extern PinDef PWMPins[];
extern SerialPortDef SerialPorts[];

#endif



