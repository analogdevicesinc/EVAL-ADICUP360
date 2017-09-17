################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/AD7124.cpp \
../src/CN0391.cpp \
../src/Communication.cpp \
../src/Timer.cpp \
../src/main.cpp 

SRC_OBJS += \
./src/AD7124.o \
./src/CN0391.o \
./src/Communication.o \
./src/Timer.o \
./src/main.o 

CPP_DEPS += \
./src/AD7124.d \
./src/CN0391.d \
./src/Communication.d \
./src/Timer.d \
./src/main.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: CrossCore GCC ARM Embedded C++ Compiler'
	arm-none-eabi-g++ -g -gdwarf-2 -ffunction-sections -fdata-sections -DCORE0 -D_RTE_ -DADuCM360 -I"/Users/amclach/Work/BitBucket/eval-adicup360/projects/ADuCM360_demo_cn0391/include" -I"/Users/amclach/Applications/CrossCore Embedded Studio.app/Contents/ARM/packs/ARM/CMSIS/5.1.0/CMSIS/Include" -I"/Users/amclach/Applications/CrossCore Embedded Studio.app/Contents/ARM/packs/AnalogDevices/ADuCM36x_DFP/1.0.2/Device/Include" -I"/Users/amclach/Work/BitBucket/eval-adicup360/projects/ADuCM360_demo_cn0391/RTE" -I"/Users/amclach/Work/BitBucket/eval-adicup360/projects/ADuCM360_demo_cn0391/RTE/Device/ADuCM360" -Wall -std=gnu++11 -c -mcpu=cortex-m3 -mthumb -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


