################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../system/src/aducm360/AdcLib.c \
../system/src/aducm360/ClkLib.c \
../system/src/aducm360/DacLib.c \
../system/src/aducm360/DioLib.c \
../system/src/aducm360/DmaLib.c \
../system/src/aducm360/FeeLib.c \
../system/src/aducm360/GptLib.c \
../system/src/aducm360/I2cLib.c \
../system/src/aducm360/IexcLib.c \
../system/src/aducm360/IntLib.c \
../system/src/aducm360/PwmLib.c \
../system/src/aducm360/PwrLib.c \
../system/src/aducm360/RstLib.c \
../system/src/aducm360/SpiLib.c \
../system/src/aducm360/UrtLib.c \
../system/src/aducm360/WdtLib.c \
../system/src/aducm360/WutLib.c 

OBJS += \
./system/src/aducm360/AdcLib.o \
./system/src/aducm360/ClkLib.o \
./system/src/aducm360/DacLib.o \
./system/src/aducm360/DioLib.o \
./system/src/aducm360/DmaLib.o \
./system/src/aducm360/FeeLib.o \
./system/src/aducm360/GptLib.o \
./system/src/aducm360/I2cLib.o \
./system/src/aducm360/IexcLib.o \
./system/src/aducm360/IntLib.o \
./system/src/aducm360/PwmLib.o \
./system/src/aducm360/PwrLib.o \
./system/src/aducm360/RstLib.o \
./system/src/aducm360/SpiLib.o \
./system/src/aducm360/UrtLib.o \
./system/src/aducm360/WdtLib.o \
./system/src/aducm360/WutLib.o 

C_DEPS += \
./system/src/aducm360/AdcLib.d \
./system/src/aducm360/ClkLib.d \
./system/src/aducm360/DacLib.d \
./system/src/aducm360/DioLib.d \
./system/src/aducm360/DmaLib.d \
./system/src/aducm360/FeeLib.d \
./system/src/aducm360/GptLib.d \
./system/src/aducm360/I2cLib.d \
./system/src/aducm360/IexcLib.d \
./system/src/aducm360/IntLib.d \
./system/src/aducm360/PwmLib.d \
./system/src/aducm360/PwrLib.d \
./system/src/aducm360/RstLib.d \
./system/src/aducm360/SpiLib.d \
./system/src/aducm360/UrtLib.d \
./system/src/aducm360/WdtLib.d \
./system/src/aducm360/WutLib.d 


# Each subdirectory must supply rules for building sources it contributes
system/src/aducm360/%.o: ../system/src/aducm360/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DTRACE -DOS_USE_TRACE_SEMIHOSTING_DEBUG -I"../include" -I"../system/include" -I"../system/include/CMSIS" -I"../system/include/aducm360" -std=gnu11 -Wno-padded -Wno-conversion -Wno-sign-conversion -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


