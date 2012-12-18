SHELL=cmd

//AIRFRAME = GIMBAL
//AIRFRAME = BI
//AIRFRAME = TRI
//AIRFRAME = QUADP
AIRFRAME = QUADX
//AIRFRAME = Y4
//AIRFRAME = Y6
//AIRFRAME = HEX6
//AIRFRAME = HEX6X
//AIRFRAME = OCTOX8
//AIRFRAME = OCTOFLATP
//AIRFRAME = OCTOFLATX
//AIRFRAME = FLYING_WING
//AIRFRAME = VTAIL4
//AIRFRAME = AIRPLANE
//AIRFRAME = SINGLECOPTER
//AIRFRAME = DUALCOPTER
//AIRFRAME = HELI_120_CCPM
//AIRFRAME = HELI_90_DEG

//TARGET=UAVXArm32F4_MW2_1_$(AIRFRAME)_CPPM_JR
//CONFIG= -DHARMONISE -D$(AIRFRAME) -DUSE_SERIAL_PPM_JR -DARM_MATH_CM4 -DHSE_VALUE=8000000

//TARGET=UAVXArm32F4_MW2_1_$(AIRFRAME)_CPPM_FUT
//CONFIG= -DHARMONISE -D$(AIRFRAME) -DUSE_SERIAL_PPM_FUT -DARM_MATH_CM4 -DHSE_VALUE=8000000

TARGET=UAVXArm32F4_MW2_1_$(AIRFRAME)
CONFIG= -DHARMONISE -D$(AIRFRAME) -DARM_MATH_CM4 -DHSE_VALUE=8000000

#TARGET=MWArm32F4_Test
#CONFIG= -DCOMMISSIONING_TEST -DARM_MATH_CM4 -DHSE_VALUE=8000000

LINK_SCRIPT=stm32f4_eeprom16k.ld

# System configuration - UNCOMMENT AS DESIRED

#Windows 7

#TCHAIN="C:/Program Files (x86)/Atollic/TrueSTUDIO for STMicroelectronics STM32 Lite 2.3.0/ARMTools/bin/"
#TCHAIN="C:/Program Files (x86)/yagarto/bin/"
TCHAIN="C:/Program Files (x86)/CodeSourcery/Sourcery_CodeBench_Lite_for_ARM_EABI/bin/"

#Windows XP
#TCHAIN="C:/Program Files/Atollic/TrueSTUDIO for STMicroelectronics STM32 Lite 2.3.0/ARMTools/bin/"
#TCHAIN="C:/Program Files/yagarto/bin/"

#Atollic TrueStudio
#CC=$(TCHAIN)arm-atollic-eabi-gcc
#OBJCOPY=$(TCHAIN)arm-atollic-eabi-objcopy
#OPT = -O0

#Yagarto currently gcc 4.6.2
#OPT = -Os
#CC=$(TCHAIN)arm-none-eabi-gcc
#OBJCOPY=$(TCHAIN)arm-none-eabi-objcopy

#Code Sourcery current gcc 4.6.1
OPT = -O3
#OPT = -Os
CC = $(TCHAIN)arm-none-eabi-gcc
OBJCOPY = $(TCHAIN)arm-none-eabi-objcopy

FPFLAGS= -mcpu=cortex-m4 -mthumb -mfloat-abi=softfp -mfpu=fpv4-sp-d16 -mlittle-endian 
#mthumb-interwork
RM = rm -rf

# Define output directory
OBJECT_DIR = obj
BIN_DIR = $(OBJECT_DIR)

# Assembler, Compiler and Linker flags and linker script settings
LINKER_FLAGS=-lm $(FPFLAGS) -Wl,--gc-sections -T$(LINK_SCRIPT) -static  -Wl,-cref "-Wl,-Map=$(BIN_DIR)/MWArm32F4.map" -Wl,--defsym=malloc_getpagesize_P=0x1000
ASSEMBLER_FLAGS=-c -g $(OPT) $(FPFLAGS) -x assembler-with-cpp  -Isrc -ILibraries\CMSIS\Include -ILibraries\Device\STM32F4xx\Include -ILibraries\STM32F4xx_StdPeriph_Driver\inc
COMPILER_FLAGS=-c -g  $(OPT) $(FPFLAGS) $(CONFIG) -Wall -ffunction-sections -fdata-sections -D"STM32F4XX" -D"USE_STDPERIPH_DRIVER"   -Isrc -ILibraries\CMSIS\Include -ILibraries\Device\STM32F4xx\Include -ILibraries\STM32F4xx_StdPeriph_Driver\inc 

# Define sources and objects
SRC := $(wildcard */*/*/*/*/*/*/*.c) \
	$(wildcard */*/*/*/*/*/*.c) \
	$(wildcard */*/*/*/*/*.c) \
	$(wildcard */*/*/*/*.c) \
	$(wildcard */*/*/*.c) \
	$(wildcard */*/*.c) \
	$(wildcard */*.c)
SRCSASM := $(wildcard */*/startup_stm32f4xx.s) \
#SRCSASM := $(wildcard */*/*/*/*/*/*/*/*.s) \
#	$(wildcard */*/*/*/*/*/*/*.s) \
#	$(wildcard */*/*/*/*/*/*.s) \
#	$(wildcard */*/*/*/*/*.s) \
#	$(wildcard */*/*/*/*.s) \
#	$(wildcard */*/*/*.s) \
#	$(wildcard */*/*.s) \
#	$(wildcard */*.s)
OBJS := $(SRC:%.c=$(OBJECT_DIR)/%.o) $(SRCSASM:%.s=$(OBJECT_DIR)/%.o)
OBJS := $(OBJS:%.S=$(OBJECT_DIR)/%.o)  

	
all: mwarm
	$(OBJCOPY) -O ihex "$(BIN_DIR)/$(TARGET).elf" "$(BIN_DIR)/$(TARGET).hex"
	$(RM) $(OBJS) "$(BIN_DIR)/*.elf" "$(BIN_DIR)/$(TARGET).map" "$(BIN_DIR)/src" "$(BIN_DIR)/Libraries"

mwarm: $(OBJS)
	$(CC) -o "$(BIN_DIR)/$(TARGET).elf" $(OBJS) $(LINKER_FLAGS)

clean:
#	$(RM) $(OBJS) "$(BIN_DIR)/*.*"
	$(RM) $(OBJS) "$(BIN_DIR)/*.elf" "$(BIN_DIR)/$(TARGET).map" "$(BIN_DIR)/src" "$(BIN_DIR)/lib"

$(OBJECT_DIR)/main.o: main.c
	@mkdir $(subst /,\,$(dir $@)) 2> NUL || echo off
	$(CC) $(COMPILER_FLAGS) main.c -o $(OBJECT_DIR)/main.o 

$(OBJECT_DIR)/%.o: %.c
	@mkdir $(subst /,\,$(dir $@)) 2> NUL || echo off
	$(CC) $(COMPILER_FLAGS) $< -o $@

$(OBJECT_DIR)/%.o: %.s
	@mkdir $(subst /,\,$(dir $@)) 2> NUL || echo off
	$(CC) $(ASSEMBLER_FLAGS) $< -o $@
	
$(OBJECT_DIR)/%.o: %.S
	@mkdir $(subst /,\,$(dir $@)) 2> NUL || echo off
	$(CC) $(ASSEMBLER_FLAGS) $< -o $@
	
