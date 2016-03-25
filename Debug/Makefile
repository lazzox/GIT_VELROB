################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL := cmd.exe
RM := rm -rf

USER_OBJS :=

LIBS := 
PROJ := 

O_SRCS := 
C_SRCS := 
S_SRCS := 
S_UPPER_SRCS := 
OBJ_SRCS := 
ASM_SRCS := 
PREPROCESSING_SRCS := 
OBJS := 
OBJS_AS_ARGS := 
C_DEPS := 
C_DEPS_AS_ARGS := 
EXECUTABLES := 
OUTPUT_FILE_PATH :=
OUTPUT_FILE_PATH_AS_ARGS :=
AVR_APP_PATH :=$$$AVR_APP_PATH$$$
QUOTE := "
ADDITIONAL_DEPENDENCIES:=
OUTPUT_FILE_DEP:=
LIB_DEP:=

# Every subdirectory with source files must be described here
SUBDIRS :=  \
../Headers


# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS +=  \
../adc_driver.c \
../hardware.c \
../interrupt.c \
../mechanism.c \
../port_driver.c \
../TC_driver.c \
../testiranje.c \
../usart_driver.c \
../X-Drive_Odometry.c


PREPROCESSING_SRCS += 


ASM_SRCS += 


OBJS +=  \
adc_driver.o \
hardware.o \
interrupt.o \
mechanism.o \
port_driver.o \
TC_driver.o \
testiranje.o \
usart_driver.o \
X-Drive_Odometry.o

OBJS_AS_ARGS +=  \
adc_driver.o \
hardware.o \
interrupt.o \
mechanism.o \
port_driver.o \
TC_driver.o \
testiranje.o \
usart_driver.o \
X-Drive_Odometry.o

C_DEPS +=  \
adc_driver.d \
hardware.d \
interrupt.d \
mechanism.d \
port_driver.d \
TC_driver.d \
testiranje.d \
usart_driver.d \
X-Drive_Odometry.d

C_DEPS_AS_ARGS +=  \
adc_driver.d \
hardware.d \
interrupt.d \
mechanism.d \
port_driver.d \
TC_driver.d \
testiranje.d \
usart_driver.d \
X-Drive_Odometry.d

OUTPUT_FILE_PATH +=XDRIVE_TIKVIC_V2.0.elf

OUTPUT_FILE_PATH_AS_ARGS +=XDRIVE_TIKVIC_V2.0.elf

ADDITIONAL_DEPENDENCIES:=

OUTPUT_FILE_DEP:= ./makedep.mk

LIB_DEP+= 

# AVR32/GNU C Compiler



















./%.o: .././%.c
	@echo Building file: $<
	@echo Invoking: AVR/GNU C Compiler : 3.4.2
	$(QUOTE)C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.2.1002\avr8-gnu-toolchain\bin\avr-gcc.exe$(QUOTE)  -funsigned-char -funsigned-bitfields -DDEBUG  -O1 -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -mrelax -g2 -Wall -mmcu=atxmega128a1 -c -std=gnu99 -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<"
	@echo Finished building: $<
	



# AVR32/GNU Preprocessing Assembler



# AVR32/GNU Assembler




ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: $(OUTPUT_FILE_PATH) $(ADDITIONAL_DEPENDENCIES)

$(OUTPUT_FILE_PATH): $(OBJS) $(USER_OBJS) $(OUTPUT_FILE_DEP) $(LIB_DEP)
	@echo Building target: $@
	@echo Invoking: AVR/GNU Linker : 3.4.2
	$(QUOTE)C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.2.1002\avr8-gnu-toolchain\bin\avr-gcc.exe$(QUOTE) -o$(OUTPUT_FILE_PATH_AS_ARGS) $(OBJS_AS_ARGS) $(USER_OBJS) $(LIBS) -Wl,-Map="XDRIVE_TIKVIC_V2.0.map" -Wl,--start-group -Wl,-lm  -Wl,--end-group -Wl,--gc-sections -mrelax -mmcu=atxmega128a1 
	@echo Finished building target: $@
	"C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.2.1002\avr8-gnu-toolchain\bin\avr-objcopy.exe" -O ihex -R .eeprom -R .fuse -R .lock -R .signature  "XDRIVE_TIKVIC_V2.0.elf" "XDRIVE_TIKVIC_V2.0.hex"
	"C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.2.1002\avr8-gnu-toolchain\bin\avr-objcopy.exe" -j .eeprom  --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0  --no-change-warnings -O ihex "XDRIVE_TIKVIC_V2.0.elf" "XDRIVE_TIKVIC_V2.0.eep" || exit 0
	"C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.2.1002\avr8-gnu-toolchain\bin\avr-objdump.exe" -h -S "XDRIVE_TIKVIC_V2.0.elf" > "XDRIVE_TIKVIC_V2.0.lss"
	"C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.2.1002\avr8-gnu-toolchain\bin\avr-objcopy.exe" -O srec -R .eeprom -R .fuse -R .lock -R .signature  "XDRIVE_TIKVIC_V2.0.elf" "XDRIVE_TIKVIC_V2.0.srec"
	"C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.2.1002\avr8-gnu-toolchain\bin\avr-size.exe" "XDRIVE_TIKVIC_V2.0.elf"
	
	





# Other Targets
clean:
	-$(RM) $(OBJS_AS_ARGS) $(EXECUTABLES)  
	-$(RM) $(C_DEPS_AS_ARGS)   
	rm -rf "XDRIVE_TIKVIC_V2.0.elf" "XDRIVE_TIKVIC_V2.0.a" "XDRIVE_TIKVIC_V2.0.hex" "XDRIVE_TIKVIC_V2.0.lss" "XDRIVE_TIKVIC_V2.0.eep" "XDRIVE_TIKVIC_V2.0.map" "XDRIVE_TIKVIC_V2.0.srec"
	