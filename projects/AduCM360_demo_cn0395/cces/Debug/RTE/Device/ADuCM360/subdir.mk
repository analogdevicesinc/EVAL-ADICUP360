################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Analog\ Devices/CrossCore\ Embedded\ Studio\ 2.6.0/ARM/packs/AnalogDevices/ADuCM36x_DFP/1.0.2/Device/Source/drivers/AdcLib.c \
C:/Analog\ Devices/CrossCore\ Embedded\ Studio\ 2.6.0/ARM/packs/AnalogDevices/ADuCM36x_DFP/1.0.2/Device/Source/drivers/ClkLib.c \
C:/Analog\ Devices/CrossCore\ Embedded\ Studio\ 2.6.0/ARM/packs/AnalogDevices/ADuCM36x_DFP/1.0.2/Device/Source/drivers/DioLib.c \
C:/Analog\ Devices/CrossCore\ Embedded\ Studio\ 2.6.0/ARM/packs/AnalogDevices/ADuCM36x_DFP/1.0.2/Device/Source/drivers/DmaLib.c \
C:/Analog\ Devices/CrossCore\ Embedded\ Studio\ 2.6.0/ARM/packs/AnalogDevices/ADuCM36x_DFP/1.0.2/Device/Source/drivers/IntLib.c \
C:/Analog\ Devices/CrossCore\ Embedded\ Studio\ 2.6.0/ARM/packs/AnalogDevices/ADuCM36x_DFP/1.0.2/Device/Source/drivers/PwmLib.c \
C:/Analog\ Devices/CrossCore\ Embedded\ Studio\ 2.6.0/ARM/packs/AnalogDevices/ADuCM36x_DFP/1.0.2/Device/Source/drivers/SpiLib.c \
C:/Analog\ Devices/CrossCore\ Embedded\ Studio\ 2.6.0/ARM/packs/AnalogDevices/ADuCM36x_DFP/1.0.2/Device/Source/drivers/UrtLib.c \
../RTE/Device/ADuCM360/startup_ADuCM360.c \
../RTE/Device/ADuCM360/system_ADuCM360.c 

SRC_OBJS += \
./RTE/Device/ADuCM360/AdcLib.o \
./RTE/Device/ADuCM360/ClkLib.o \
./RTE/Device/ADuCM360/DioLib.o \
./RTE/Device/ADuCM360/DmaLib.o \
./RTE/Device/ADuCM360/IntLib.o \
./RTE/Device/ADuCM360/PwmLib.o \
./RTE/Device/ADuCM360/SpiLib.o \
./RTE/Device/ADuCM360/UrtLib.o \
./RTE/Device/ADuCM360/startup_ADuCM360.o \
./RTE/Device/ADuCM360/system_ADuCM360.o 

C_DEPS += \
./RTE/Device/ADuCM360/AdcLib.d \
./RTE/Device/ADuCM360/ClkLib.d \
./RTE/Device/ADuCM360/DioLib.d \
./RTE/Device/ADuCM360/DmaLib.d \
./RTE/Device/ADuCM360/IntLib.d \
./RTE/Device/ADuCM360/PwmLib.d \
./RTE/Device/ADuCM360/SpiLib.d \
./RTE/Device/ADuCM360/UrtLib.d \
./RTE/Device/ADuCM360/startup_ADuCM360.d \
./RTE/Device/ADuCM360/system_ADuCM360.d 


# Each subdirectory must supply rules for building sources it contributes
RTE/Device/ADuCM360/AdcLib.o: C:/Analog\ Devices/CrossCore\ Embedded\ Studio\ 2.6.0/ARM/packs/AnalogDevices/ADuCM36x_DFP/1.0.2/Device/Source/drivers/AdcLib.c
	@echo 'Building file: $<'
	@echo 'Invoking: CrossCore GCC ARM Embedded C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -ffunction-sections -fdata-sections -DCORE0 -D_DEBUG -D_RTE_ -DADuCM360 -I"C:\Users\HHe2\Documents\eval-adicup360\projects\AduCM360_demo_cn0395\cces\system" -I"C:\Users\HHe2\Documents\eval-adicup360\projects\AduCM360_demo_cn0395\cces\include" -I"C:/Analog Devices/CrossCore Embedded Studio 2.6.0/ARM/packs/ARM/CMSIS/5.1.0/CMSIS/Include" -I"C:/Analog Devices/CrossCore Embedded Studio 2.6.0/ARM/packs/AnalogDevices/ADuCM36x_DFP/1.0.2/Device/Include" -I"C:\Users\HHe2\Documents\eval-adicup360\projects\AduCM360_demo_cn0395\cces/RTE" -I"C:\Users\HHe2\Documents\eval-adicup360\projects\AduCM360_demo_cn0395\cces/RTE/Device/ADuCM360" -Wall -c -mcpu=cortex-m3 -mthumb -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

RTE/Device/ADuCM360/ClkLib.o: C:/Analog\ Devices/CrossCore\ Embedded\ Studio\ 2.6.0/ARM/packs/AnalogDevices/ADuCM36x_DFP/1.0.2/Device/Source/drivers/ClkLib.c
	@echo 'Building file: $<'
	@echo 'Invoking: CrossCore GCC ARM Embedded C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -ffunction-sections -fdata-sections -DCORE0 -D_DEBUG -D_RTE_ -DADuCM360 -I"C:\Users\HHe2\Documents\eval-adicup360\projects\AduCM360_demo_cn0395\cces\system" -I"C:\Users\HHe2\Documents\eval-adicup360\projects\AduCM360_demo_cn0395\cces\include" -I"C:/Analog Devices/CrossCore Embedded Studio 2.6.0/ARM/packs/ARM/CMSIS/5.1.0/CMSIS/Include" -I"C:/Analog Devices/CrossCore Embedded Studio 2.6.0/ARM/packs/AnalogDevices/ADuCM36x_DFP/1.0.2/Device/Include" -I"C:\Users\HHe2\Documents\eval-adicup360\projects\AduCM360_demo_cn0395\cces/RTE" -I"C:\Users\HHe2\Documents\eval-adicup360\projects\AduCM360_demo_cn0395\cces/RTE/Device/ADuCM360" -Wall -c -mcpu=cortex-m3 -mthumb -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

RTE/Device/ADuCM360/DioLib.o: C:/Analog\ Devices/CrossCore\ Embedded\ Studio\ 2.6.0/ARM/packs/AnalogDevices/ADuCM36x_DFP/1.0.2/Device/Source/drivers/DioLib.c
	@echo 'Building file: $<'
	@echo 'Invoking: CrossCore GCC ARM Embedded C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -ffunction-sections -fdata-sections -DCORE0 -D_DEBUG -D_RTE_ -DADuCM360 -I"C:\Users\HHe2\Documents\eval-adicup360\projects\AduCM360_demo_cn0395\cces\system" -I"C:\Users\HHe2\Documents\eval-adicup360\projects\AduCM360_demo_cn0395\cces\include" -I"C:/Analog Devices/CrossCore Embedded Studio 2.6.0/ARM/packs/ARM/CMSIS/5.1.0/CMSIS/Include" -I"C:/Analog Devices/CrossCore Embedded Studio 2.6.0/ARM/packs/AnalogDevices/ADuCM36x_DFP/1.0.2/Device/Include" -I"C:\Users\HHe2\Documents\eval-adicup360\projects\AduCM360_demo_cn0395\cces/RTE" -I"C:\Users\HHe2\Documents\eval-adicup360\projects\AduCM360_demo_cn0395\cces/RTE/Device/ADuCM360" -Wall -c -mcpu=cortex-m3 -mthumb -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

RTE/Device/ADuCM360/DmaLib.o: C:/Analog\ Devices/CrossCore\ Embedded\ Studio\ 2.6.0/ARM/packs/AnalogDevices/ADuCM36x_DFP/1.0.2/Device/Source/drivers/DmaLib.c
	@echo 'Building file: $<'
	@echo 'Invoking: CrossCore GCC ARM Embedded C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -ffunction-sections -fdata-sections -DCORE0 -D_DEBUG -D_RTE_ -DADuCM360 -I"C:\Users\HHe2\Documents\eval-adicup360\projects\AduCM360_demo_cn0395\cces\system" -I"C:\Users\HHe2\Documents\eval-adicup360\projects\AduCM360_demo_cn0395\cces\include" -I"C:/Analog Devices/CrossCore Embedded Studio 2.6.0/ARM/packs/ARM/CMSIS/5.1.0/CMSIS/Include" -I"C:/Analog Devices/CrossCore Embedded Studio 2.6.0/ARM/packs/AnalogDevices/ADuCM36x_DFP/1.0.2/Device/Include" -I"C:\Users\HHe2\Documents\eval-adicup360\projects\AduCM360_demo_cn0395\cces/RTE" -I"C:\Users\HHe2\Documents\eval-adicup360\projects\AduCM360_demo_cn0395\cces/RTE/Device/ADuCM360" -Wall -c -mcpu=cortex-m3 -mthumb -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

RTE/Device/ADuCM360/IntLib.o: C:/Analog\ Devices/CrossCore\ Embedded\ Studio\ 2.6.0/ARM/packs/AnalogDevices/ADuCM36x_DFP/1.0.2/Device/Source/drivers/IntLib.c
	@echo 'Building file: $<'
	@echo 'Invoking: CrossCore GCC ARM Embedded C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -ffunction-sections -fdata-sections -DCORE0 -D_DEBUG -D_RTE_ -DADuCM360 -I"C:\Users\HHe2\Documents\eval-adicup360\projects\AduCM360_demo_cn0395\cces\system" -I"C:\Users\HHe2\Documents\eval-adicup360\projects\AduCM360_demo_cn0395\cces\include" -I"C:/Analog Devices/CrossCore Embedded Studio 2.6.0/ARM/packs/ARM/CMSIS/5.1.0/CMSIS/Include" -I"C:/Analog Devices/CrossCore Embedded Studio 2.6.0/ARM/packs/AnalogDevices/ADuCM36x_DFP/1.0.2/Device/Include" -I"C:\Users\HHe2\Documents\eval-adicup360\projects\AduCM360_demo_cn0395\cces/RTE" -I"C:\Users\HHe2\Documents\eval-adicup360\projects\AduCM360_demo_cn0395\cces/RTE/Device/ADuCM360" -Wall -c -mcpu=cortex-m3 -mthumb -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

RTE/Device/ADuCM360/PwmLib.o: C:/Analog\ Devices/CrossCore\ Embedded\ Studio\ 2.6.0/ARM/packs/AnalogDevices/ADuCM36x_DFP/1.0.2/Device/Source/drivers/PwmLib.c
	@echo 'Building file: $<'
	@echo 'Invoking: CrossCore GCC ARM Embedded C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -ffunction-sections -fdata-sections -DCORE0 -D_DEBUG -D_RTE_ -DADuCM360 -I"C:\Users\HHe2\Documents\eval-adicup360\projects\AduCM360_demo_cn0395\cces\system" -I"C:\Users\HHe2\Documents\eval-adicup360\projects\AduCM360_demo_cn0395\cces\include" -I"C:/Analog Devices/CrossCore Embedded Studio 2.6.0/ARM/packs/ARM/CMSIS/5.1.0/CMSIS/Include" -I"C:/Analog Devices/CrossCore Embedded Studio 2.6.0/ARM/packs/AnalogDevices/ADuCM36x_DFP/1.0.2/Device/Include" -I"C:\Users\HHe2\Documents\eval-adicup360\projects\AduCM360_demo_cn0395\cces/RTE" -I"C:\Users\HHe2\Documents\eval-adicup360\projects\AduCM360_demo_cn0395\cces/RTE/Device/ADuCM360" -Wall -c -mcpu=cortex-m3 -mthumb -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

RTE/Device/ADuCM360/SpiLib.o: C:/Analog\ Devices/CrossCore\ Embedded\ Studio\ 2.6.0/ARM/packs/AnalogDevices/ADuCM36x_DFP/1.0.2/Device/Source/drivers/SpiLib.c
	@echo 'Building file: $<'
	@echo 'Invoking: CrossCore GCC ARM Embedded C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -ffunction-sections -fdata-sections -DCORE0 -D_DEBUG -D_RTE_ -DADuCM360 -I"C:\Users\HHe2\Documents\eval-adicup360\projects\AduCM360_demo_cn0395\cces\system" -I"C:\Users\HHe2\Documents\eval-adicup360\projects\AduCM360_demo_cn0395\cces\include" -I"C:/Analog Devices/CrossCore Embedded Studio 2.6.0/ARM/packs/ARM/CMSIS/5.1.0/CMSIS/Include" -I"C:/Analog Devices/CrossCore Embedded Studio 2.6.0/ARM/packs/AnalogDevices/ADuCM36x_DFP/1.0.2/Device/Include" -I"C:\Users\HHe2\Documents\eval-adicup360\projects\AduCM360_demo_cn0395\cces/RTE" -I"C:\Users\HHe2\Documents\eval-adicup360\projects\AduCM360_demo_cn0395\cces/RTE/Device/ADuCM360" -Wall -c -mcpu=cortex-m3 -mthumb -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

RTE/Device/ADuCM360/UrtLib.o: C:/Analog\ Devices/CrossCore\ Embedded\ Studio\ 2.6.0/ARM/packs/AnalogDevices/ADuCM36x_DFP/1.0.2/Device/Source/drivers/UrtLib.c
	@echo 'Building file: $<'
	@echo 'Invoking: CrossCore GCC ARM Embedded C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -ffunction-sections -fdata-sections -DCORE0 -D_DEBUG -D_RTE_ -DADuCM360 -I"C:\Users\HHe2\Documents\eval-adicup360\projects\AduCM360_demo_cn0395\cces\system" -I"C:\Users\HHe2\Documents\eval-adicup360\projects\AduCM360_demo_cn0395\cces\include" -I"C:/Analog Devices/CrossCore Embedded Studio 2.6.0/ARM/packs/ARM/CMSIS/5.1.0/CMSIS/Include" -I"C:/Analog Devices/CrossCore Embedded Studio 2.6.0/ARM/packs/AnalogDevices/ADuCM36x_DFP/1.0.2/Device/Include" -I"C:\Users\HHe2\Documents\eval-adicup360\projects\AduCM360_demo_cn0395\cces/RTE" -I"C:\Users\HHe2\Documents\eval-adicup360\projects\AduCM360_demo_cn0395\cces/RTE/Device/ADuCM360" -Wall -c -mcpu=cortex-m3 -mthumb -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

RTE/Device/ADuCM360/%.o: ../RTE/Device/ADuCM360/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: CrossCore GCC ARM Embedded C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -ffunction-sections -fdata-sections -DCORE0 -D_DEBUG -D_RTE_ -DADuCM360 -I"C:\Users\HHe2\Documents\eval-adicup360\projects\AduCM360_demo_cn0395\cces\system" -I"C:\Users\HHe2\Documents\eval-adicup360\projects\AduCM360_demo_cn0395\cces\include" -I"C:/Analog Devices/CrossCore Embedded Studio 2.6.0/ARM/packs/ARM/CMSIS/5.1.0/CMSIS/Include" -I"C:/Analog Devices/CrossCore Embedded Studio 2.6.0/ARM/packs/AnalogDevices/ADuCM36x_DFP/1.0.2/Device/Include" -I"C:\Users\HHe2\Documents\eval-adicup360\projects\AduCM360_demo_cn0395\cces/RTE" -I"C:\Users\HHe2\Documents\eval-adicup360\projects\AduCM360_demo_cn0395\cces/RTE/Device/ADuCM360" -Wall -c -mcpu=cortex-m3 -mthumb -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


