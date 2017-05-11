################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/AD7988.c \
../src/ADN8810.c \
../src/CN0395.c \
../src/Communication.c \
../src/Flash.c \
../src/SHT30.c \
../src/Timer.c \
../src/main.c 

OBJS += \
./src/AD7988.o \
./src/ADN8810.o \
./src/CN0395.o \
./src/Communication.o \
./src/Flash.o \
./src/SHT30.o \
./src/Timer.o \
./src/main.o 

C_DEPS += \
./src/AD7988.d \
./src/ADN8810.d \
./src/CN0395.d \
./src/Communication.d \
./src/Flash.d \
./src/SHT30.d \
./src/Timer.d \
./src/main.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DTRACE -DOS_USE_TRACE_SEMIHOSTING_DEBUG -I"../include" -I"../system/include" -I"../system/include/cmsis" -I"../system/include/aducm360" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


