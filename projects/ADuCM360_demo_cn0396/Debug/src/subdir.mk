################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/AD5270.c \
../src/AD7798.c \
../src/ADT7310.c \
../src/CN0396.c \
../src/Communication.c \
../src/Timer.c \
../src/main.c 

OBJS += \
./src/AD5270.o \
./src/AD7798.o \
./src/ADT7310.o \
./src/CN0396.o \
./src/Communication.o \
./src/Timer.o \
./src/main.o 

C_DEPS += \
./src/AD5270.d \
./src/AD7798.d \
./src/ADT7310.d \
./src/CN0396.d \
./src/Communication.d \
./src/Timer.d \
./src/main.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DTRACE -I"../include" -I"../system/include" -I"../system/include/cmsis" -I"../system/include/aducm360" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


