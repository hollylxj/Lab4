#This is the Makefile for Lab 1 v9 for ELEC424 Fall 2013
# Authors: Steven Arroyo and Lin Zhong, Rice University


# use the arm compiler 
CC = arm-none-eabi-gcc

# Define paths for all included files.  This assumes STM standard library was placed unmodified 
# in a folder called lib/ as outlined in the lab description.
LIB = lib
STM_LIB = $(LIB)/STM32F10x_StdPeriph_Lib_V3.5.0/Libraries
STM_STD_PERIF = $(STM_LIB)/STM32F10x_StdPeriph_Driver
STM_DEVICE_SUPPORT = $(STM_LIB)/CMSIS/CM3/DeviceSupport/ST/STM32F10x
STM_CORE_SUPPORT = $(STM_LIB)/CMSIS/CM3/CoreSupport
STM_STARTUP = $(STM_LIB)/CMSIS/CM3/DeviceSupport/ST/STM32F10x/startup/gcc_ride7
BI = liblab3.a
FREERTOS_SOURCE = FreeRTOS/Source
FREERTOS_HEADER = $(FREERTOS_SOURCE)/include
PORT = $(FREERTOS_SOURCE)/portable/GCC/ARM_CM3
HEAP = $(FREERTOS_SOURCE)/portable/MemMang

#mcpu=cortex-m3 This specifies the name of the target ARM processor. GCC uses this name to determine what kind of instructions it can emit when generating assembly code. 
# -mthumb/-marm: change between thumb/arm state, that is, use thumb/arm instructions
#Idir: Add the directory dir to the head of the list of directories to be searched for header files. 
CFLAGS = -O0 -g3 -mcpu=cortex-m3 -mthumb -I$(STM_STD_PERIF)/inc -I$(PORT) -I$(HEAP) -I$(STM_STD_PERIF)/src -I$(STM_STARTUP) -I$(STM_CORE_SUPPORT) -I$(FREERTOS_SOURCE) -I$(FREERTOS_HEADER) -I$(STM_DEVICE_SUPPORT) -DSTM32F10X_MD -include stm32f10x_conf.h  -Wl,--gc-sections -T stm32_flash.ld
#-include system_stm32f10x.h 
# build all relevant files and create .elf
#all: v9
#v9:  stm32f10x_gpio.elf stm32f10x_rcc.elf v9.elf
#	$(CC) $(CFLAGS) stm32f10x_gpio.elf stm32f10x_rcc.elf v9.elf -elf v9
#v9.elf: v9.c
#	$(CC) -c $(STM_STARTUP)/startup_stm32f10x_md.s v9.c 
#stm32f10x_gpio.elf: stm32f10x_gpio.c	
#	$(CC) -c $(STM_STD_PERIF)/src/stm32f10x_gpio.c 
#tm32f10x_rcc.elf:tm32f10x_rcc.c
#	$(STM_STD_PERIF)/src/stm32f10x_rcc.c -o v9.c
all:
	$(CC) $(CFLAGS) $(STM_DEVICE_SUPPORT)/system_stm32f10x.c $(STM_DEVICE_SUPPORT)/system_stm32f10x.h $(STM_STD_PERIF)/src/stm32f10x_exti.c $(STM_STD_PERIF)/src/stm32f10x_dma.c $(FREERTOS_SOURCE)/tasks.c $(FREERTOS_SOURCE)/list.c $(FREERTOS_SOURCE)/queue.c $(STM_STD_PERIF)/src/stm32f10x_usart.c $(STM_STARTUP)/startup_stm32f10x_md.s $(BI) $(STM_STD_PERIF)/src/stm32f10x_gpio.c $(STM_STD_PERIF)/src/stm32f10x_rcc.c $(STM_STD_PERIF)/src/misc.c $(STM_STD_PERIF)/src/stm32f10x_tim.c $(STM_STD_PERIF)/src/stm32f10x_flash.c $(HEAP)/heap_2.c v9.c $(PORT)/port.c liblab3.a -o v9.elf
# 
clean:
	rm -rf *o v9

# program elf into crazyflie flash memory with busblaster
flash:
	openocd -d0 -f interface/busblaster.cfg -f target/stm32f1x.cfg -c init -c targets -c "reset halt" \
                 -c "flash write_image erase v9.elf" -c "verify_image v9.elf" -c "reset run" -c shutdown

#Start openocd, so we can use it with gdb 
openocd:
	openocd -d0 -f interface/busblaster.cfg -f target/stm32f1x.cfg -c init -c targets

# Example output when V=1 is passed (make V=1)
# arm-none-eabi-gcc -O0 -g3 -mcpu=cortex-m3 -mthumb -Ilib/STM32F10x_StdPeriph_Lib_V3.5.0/Libraries/STM32F10x_StdPeriph_Driver/inc -Ilib/STM32F10x_StdPeriph_Lib_V3.5.0/Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x/startup/gcc_ride7 -Ilib/STM32F10x_StdPeriph_Lib_V3.5.0/Libraries/CMSIS/CM3/CoreSupport -Ilib/STM32F10x_StdPeriph_Lib_V3.5.0/Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x -DSTM32F10X_MD -include stm32f10x_conf.h -Wl,--gc-sections -T stm32_flash.ld lib/STM32F10x_StdPeriph_Lib_V3.5.0/Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x/startup/gcc_ride7/startup_stm32f10x_md.s v9.c lib/STM32F10x_StdPeriph_Lib_V3.5.0/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_gpio.c lib/STM32F10x_StdPeriph_Lib_V3.5.0/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_rcc.c -o v9.elf 
