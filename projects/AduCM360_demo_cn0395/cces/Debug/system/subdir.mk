################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../system/adi_initialize.c 

SRC_OBJS += \
./system/adi_initialize.o 

C_DEPS += \
./system/adi_initialize.d 


# Each subdirectory must supply rules for building sources it contributes
system/%.o: ../system/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: CrossCore GCC ARM Embedded C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -ffunction-sections -fdata-sections -DCORE0 -D_DEBUG -D_RTE_ -DADuCM360 -I"C:\Users\HHe2\Documents\eval-adicup360\projects\AduCM360_demo_cn0395\cces\system" -I"C:\Users\HHe2\Documents\eval-adicup360\projects\AduCM360_demo_cn0395\cces\include" -I"C:/Analog Devices/CrossCore Embedded Studio 2.6.0/ARM/packs/ARM/CMSIS/5.1.0/CMSIS/Include" -I"C:/Analog Devices/CrossCore Embedded Studio 2.6.0/ARM/packs/AnalogDevices/ADuCM36x_DFP/1.0.2/Device/Include" -I"C:\Users\HHe2\Documents\eval-adicup360\projects\AduCM360_demo_cn0395\cces/RTE" -I"C:\Users\HHe2\Documents\eval-adicup360\projects\AduCM360_demo_cn0395\cces/RTE/Device/ADuCM360" -Wall -c -mcpu=cortex-m3 -mthumb -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


