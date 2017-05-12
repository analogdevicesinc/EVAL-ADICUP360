################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/AD7124.c \
../src/AD7798.c \
../src/CN0370.c \
../src/CN0397.c \
../src/CN0398.c \
../src/Command.c \
../src/Communication.c \
../src/Timer.c \
../src/lcd.c \
../src/main.c 

OBJS += \
./src/AD7124.o \
./src/AD7798.o \
./src/CN0370.o \
./src/CN0397.o \
./src/CN0398.o \
./src/Command.o \
./src/Communication.o \
./src/Timer.o \
./src/lcd.o \
./src/main.o 

C_DEPS += \
./src/AD7124.d \
./src/AD7798.d \
./src/CN0370.d \
./src/CN0397.d \
./src/CN0398.d \
./src/Command.d \
./src/Communication.d \
./src/Timer.d \
./src/lcd.d \
./src/main.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DTRACE -I"../include" -I"../system/include" -I"../system/include/cmsis" -I"../system/include/aducm360" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


