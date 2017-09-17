################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/ADC.c \
../src/CN0394.c \
../src/Communication.c \
../src/Timer.c \
../src/main.c 

SRC_OBJS += \
./src/ADC.o \
./src/CN0394.o \
./src/Communication.o \
./src/Timer.o \
./src/main.o 

C_DEPS += \
./src/ADC.d \
./src/CN0394.d \
./src/Communication.d \
./src/Timer.d \
./src/main.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: CrossCore GCC ARM Embedded C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -ffunction-sections -fdata-sections -DCORE0 -D_DEBUG -D_RTE_ -DADuCM360 -I"/Users/amclach/Work/BitBucket/eval-adicup360/projects/ADuCM360_demo_cn0394/system" -I"/Users/amclach/Work/BitBucket/eval-adicup360/projects/ADuCM360_demo_cn0394/include" -I"/Users/amclach/Applications/CrossCore Embedded Studio.app/Contents/ARM/packs/ARM/CMSIS/5.1.0/CMSIS/Include" -I"/Users/amclach/Applications/CrossCore Embedded Studio.app/Contents/ARM/packs/AnalogDevices/ADuCM36x_DFP/1.0.2/Device/Include" -I"/Users/amclach/Work/BitBucket/eval-adicup360/projects/ADuCM360_demo_cn0394/RTE" -I"/Users/amclach/Work/BitBucket/eval-adicup360/projects/ADuCM360_demo_cn0394/RTE/Device/ADuCM360" -Wall -c -mcpu=cortex-m3 -mthumb -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


