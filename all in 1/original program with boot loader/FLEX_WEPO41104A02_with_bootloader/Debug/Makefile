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

# Every subdirectory with source files must be described here
SUBDIRS :=  \
../inc


# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS +=  \
../at86rf231.c \
../bootloader.c \
../ds620.c \
../eeprom_abbild.c \
../fuses.c \
../pal_trx_access.c \
../pm_config_protocol.c \
../sw_twi.c \
../SX8723c.c \
../WEPO41104A02.c


PREPROCESSING_SRCS += 


ASM_SRCS += 


OBJS +=  \
at86rf231.o \
bootloader.o \
ds620.o \
eeprom_abbild.o \
fuses.o \
pal_trx_access.o \
pm_config_protocol.o \
sw_twi.o \
SX8723c.o \
WEPO41104A02.o


OBJS_AS_ARGS +=  \
at86rf231.o \
bootloader.o \
ds620.o \
eeprom_abbild.o \
fuses.o \
pal_trx_access.o \
pm_config_protocol.o \
sw_twi.o \
SX8723c.o \
WEPO41104A02.o


C_DEPS +=  \
at86rf231.d \
bootloader.d \
ds620.d \
eeprom_abbild.d \
fuses.d \
pal_trx_access.d \
pm_config_protocol.d \
sw_twi.d \
SX8723c.d \
WEPO41104A02.d


C_DEPS_AS_ARGS +=  \
at86rf231.d \
bootloader.d \
ds620.d \
eeprom_abbild.d \
fuses.d \
pal_trx_access.d \
pm_config_protocol.d \
sw_twi.d \
SX8723c.d \
WEPO41104A02.d


OUTPUT_FILE_PATH +=WEPO41104A02.elf

OUTPUT_FILE_PATH_AS_ARGS +=WEPO41104A02.elf

ADDITIONAL_DEPENDENCIES:=

OUTPUT_FILE_DEP:= ./makedep.mk

# AVR32/GNU C Compiler





















./%.o: .././%.c
	@echo Building file: $<
	@echo Invoking: AVR/GNU C Compiler
	$(QUOTE)C:\Program Files (x86)\Atmel\AVR Studio 5.1\extensions\Atmel\AVRGCC\3.3.1.27\AVRToolchain\bin\avr-gcc.exe$(QUOTE)  -funsigned-char -funsigned-bitfields -O3 -fpack-struct -fshort-enums -g2 -Wall -c -std=gnu99 -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)"  -mmcu=atmega324p  -o"$@" "$<" 
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

$(OUTPUT_FILE_PATH): $(OBJS) $(USER_OBJS) $(OUTPUT_FILE_DEP)
	@echo Building target: $@
	@echo Invoking: AVR/GNU C Linker
	$(QUOTE)C:\Program Files (x86)\Atmel\AVR Studio 5.1\extensions\Atmel\AVRGCC\3.3.1.27\AVRToolchain\bin\avr-gcc.exe$(QUOTE) -o$(OUTPUT_FILE_PATH_AS_ARGS) $(OBJS_AS_ARGS) $(USER_OBJS) $(LIBS) -Wl,-Map="WEPO41104A02.map" -Wl,-lm  -Wl,--section-start=.boot=0x7000 -Wl,--section-start=.bootloader=0x7100  -mmcu=atmega324p  
	@echo Finished building target: $@
	"C:\Program Files (x86)\Atmel\AVR Studio 5.1\extensions\Atmel\AVRGCC\3.3.1.27\AVRToolchain\bin\avr-objcopy.exe" -O ihex -R .eeprom -R .fuse -R .lock -R .signature  "WEPO41104A02.elf" "WEPO41104A02.hex"
	"C:\Program Files (x86)\Atmel\AVR Studio 5.1\extensions\Atmel\AVRGCC\3.3.1.27\AVRToolchain\bin\avr-objcopy.exe" -j .eeprom  --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0  --no-change-warnings -O ihex "WEPO41104A02.elf" "WEPO41104A02.eep" || exit 0
	"C:\Program Files (x86)\Atmel\AVR Studio 5.1\extensions\Atmel\AVRGCC\3.3.1.27\AVRToolchain\bin\avr-objdump.exe" -h -S "WEPO41104A02.elf" > "WEPO41104A02.lss"
	"C:\Program Files (x86)\Atmel\AVR Studio 5.1\extensions\Atmel\AVRGCC\3.3.1.27\AVRToolchain\bin\avr-size.exe" -C --mcu=atmega324p  "WEPO41104A02.elf"
	




# Other Targets
clean:
	-$(RM) $(OBJS_AS_ARGS)$(C_DEPS_AS_ARGS) $(EXECUTABLES) 
	rm -rf "WEPO41104A02.hex" "WEPO41104A02.lss" "WEPO41104A02.eep" "WEPO41104A02.map"
	