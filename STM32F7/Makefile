#######################################################################
# Makefile for STM32F767ZI Nucleo projects

PROJECT = main
CUBE_PATH = /root/STM32CubeF7
HEAP_SIZE = 0x100
FLASH_OFFSET=0x08000000

FLASH = st-flash


################
# Sources

SOURCES_S = ${CUBE_PATH}/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/gcc/startup_stm32f767xx.s

SOURCES_C = src/main.c src/dma.c src/i2c.c src/8731.c src/hw_i2c.c src/hw_i2s.c src/sai.c src/wavetable.c src/stm32f7xx_hal_msp.c src/stm32f7xx_it.c src/system_stm32f7xx.c

SOURCES_C += sys/stubs.c sys/_sbrk.c
#SOURCES_C += ${CUBE_PATH}/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/system_stm32f7xx.c
SOURCES_C += ${CUBE_PATH}/Drivers/BSP/STM32F7xx_Nucleo_144/stm32f7xx_nucleo_144.c
SOURCES_C += ${CUBE_PATH}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_gpio.c
SOURCES_C += ${CUBE_PATH}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_adc.c
SOURCES_C += ${CUBE_PATH}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_adc_ex.c
SOURCES_C += ${CUBE_PATH}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pwr.c
SOURCES_C += ${CUBE_PATH}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pwr_ex.c
SOURCES_C += ${CUBE_PATH}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_rcc.c
SOURCES_C += ${CUBE_PATH}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_rcc_ex.c
#SOURCES_C += ${CUBE_PATH}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_spi.c
SOURCES_C += ${CUBE_PATH}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_dma.c
SOURCES_C += ${CUBE_PATH}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_cortex.c
SOURCES_C += ${CUBE_PATH}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_exti.c
#SOURCES_C += ${CUBE_PATH}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_i2s.c
SOURCES_C += ${CUBE_PATH}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_i2c.c
SOURCES_C += ${CUBE_PATH}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_i2c_ex.c
SOURCES_C += ${CUBE_PATH}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_sai.c
SOURCES_C += ${CUBE_PATH}/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal.c
SOURCES_CPP =

SOURCES = $(SOURCES_S) $(SOURCES_C) $(SOURCES_CPP)
OBJS = $(SOURCES_S:.s=.o) $(SOURCES_C:.c=.o) $(SOURCES_CPP:.cpp=.o)

################
# Includes and Defines

INCLUDES += -I src
INCLUDES += -I ${CUBE_PATH}/Drivers/CMSIS/Include
INCLUDES += -I ${CUBE_PATH}/Drivers/CMSIS/DSP/Include
INCLUDES += -I ${CUBE_PATH}/Drivers/CMSIS/Device/ST/STM32F7xx/Include
INCLUDES += -I ${CUBE_PATH}/Drivers/STM32F7xx_HAL_Driver/Inc
INCLUDES += -I ${CUBE_PATH}/Drivers/BSP/STM32F7xx_Nucleo_144

DEFINES = -DSTM32 -DSTM32F7 -DSTM32F767xx

################
# Compiler/Assembler/Linker/etc

PREFIX = arm-none-eabi

CC = $(PREFIX)-gcc
AS = $(PREFIX)-as
AR = $(PREFIX)-ar
LD = $(PREFIX)-gcc
NM = $(PREFIX)-nm
OBJCOPY = $(PREFIX)-objcopy
OBJDUMP = $(PREFIX)-objdump
READELF = $(PREFIX)-readelf
SIZE = $(PREFIX)-size
GDB = $(PREFIX)-gdb
RM = rm -f

################
# Compiler options

MCUFLAGS = -mcpu=cortex-m7 -mlittle-endian
MCUFLAGS += -mfloat-abi=hard -mfpu=fpv5-sp-d16
MCUFLAGS += -mthumb

#DEBUG_OPTIMIZE_FLAGS = -O0 -g -ggdb3
DEBUG_OPTIMIZE_FLAGS = -O2

CFLAGS = -std=c11
CFLAGS += -Wall -Wextra --pedantic
# generate listing files
CFLAGS += -Wa,-aghlms=$(<:%.c=%.lst)
CFLAGS += -DHEAP_SIZE=$(HEAP_SIZE) -DARM_MATH_CM7
CFLAGS += -fstack-usage

CFLAGS_EXTRA = -nostartfiles -nodefaultlibs -nostdlib
CFLAGS_EXTRA += -fdata-sections -ffunction-sections

CFLAGS += $(DEFINES) $(MCUFLAGS) $(DEBUG_OPTIMIZE_FLAGS) $(CFLAGS_EXTRA) $(INCLUDES)

LDFLAGS = -static $(MCUFLAGS)
LDFLAGS += -Wl,--start-group -lgcc -lm -lc -lg -lstdc++ -lsupc++ -Wl,--end-group
LDFLAGS += -Wl,--gc-sections -Wl,--print-gc-sections -Wl,--cref,-Map=$(@:%.elf=%.map)
LDFLAGS += -Wl,--print-memory-usage 
LDFLAGS += -L ${CUBE_PATH}/Projects/STM32F767ZI-Nucleo/Demonstrations/SW4STM32/STM32767ZI_Nucleo/ -T STM32F767ZITx_FLASH.ld

################
# phony rules

.PHONY: all clean flash erase

all: $(PROJECT).bin $(PROJECT).hex $(PROJECT).asm

clean:
	$(RM) $(OBJS) $(OBJS:$.o=$.lst) $(OBJS:$.o=$.su) $(PROJECT).elf $(PROJECT).bin $(PROJECT).hex $(PROJECT).map $(PROJECT).asm
u
BMP_DEVICE ?= /dev/ttyACM0
JLINK_CPUNAME ?= STM32F737ZI

flash: $(PROJECT).bin
	$(FLASH) write main.bin 0x08000000

flash-bmp: $(PROJECT).elf
	# assuming:
	#  * Black Magic Probe connected to $(BMP_DEVICE)
	#  * compatible board connected via SWD
	$(GDB) $(PROJECT).elf \
		-ex 'set confirm off' \
		-ex 'target extended-remote $(BMP_DEVICE)' \
		-ex 'mon hard_srst' \
		-ex 'mon swdp_scan' \
		-ex 'attach 1' \
		-ex 'load' \
		-ex 'compare-sections' \
		-ex 'quit'

flash-jlink: $(PROJECT).bin
	# assuming:
	#  * any type of Segger JLINK that is usable with an STM32
	#    (e.g. the embedded jlink on the DK)
	#  * compatible board connected via SWD
	#  * installed JLink Software
	printf "erase\nloadfile $< ${FLASH_OFFSET}\nr\nq\n" | JLinkExe -nogui 1 -autoconnect 1 -device $(JLINK_CPUNAME) -if swd -speed 4000

erase-jlink:
	printf "erase\nr\nq\n" | JLinkExe -nogui 1 -autoconnect 1 -device $(JLINK_CPUNAME) -if swd -speed 4000

################
# dependency graphs for wildcard rules

$(PROJECT).elf: $(OBJS)

################
# wildcard rules

%.elf:
	$(LD) $(OBJS) $(LDFLAGS) -o $@
	$(SIZE) -A $@

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@

%.hex: %.elf
	$(OBJCOPY) -O ihex $< $@

%.asm: %.elf
	$(OBJDUMP) -dgCxwsSh --show-raw-insn $< > $@

# EOF
