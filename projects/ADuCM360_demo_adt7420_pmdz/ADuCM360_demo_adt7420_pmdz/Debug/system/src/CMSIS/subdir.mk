################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../system/src/CMSIS/system_ADuCM360.c \
../system/src/CMSIS/vectors_ADuCM360.c 

OBJS += \
./system/src/CMSIS/system_ADuCM360.o \
./system/src/CMSIS/vectors_ADuCM360.o 

C_DEPS += \
./system/src/CMSIS/system_ADuCM360.d \
./system/src/CMSIS/vectors_ADuCM360.d 


# Each subdirectory must supply rules for building sources it contributes
system/src/CMSIS/system_ADuCM360.o: ../system/src/CMSIS/system_ADuCM360.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DTRACE -DOS_USE_TRACE_SEMIHOSTING_DEBUG -I"../include" -I"../system/include" -I"../system/include/CMSIS" -I"../system/include/aducm360" -std=gnu11 -Wno-missing-prototypes -Wno-padded -Wno-missing-declarations -MMD -MP -MF"$(@:%.o=%.d)" -MT"system/src/CMSIS/system_ADuCM360.d" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

system/src/CMSIS/%.o: ../system/src/CMSIS/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DTRACE -DOS_USE_TRACE_SEMIHOSTING_DEBUG -I"../include" -I"../system/include" -I"../system/include/CMSIS" -I"../system/include/aducm360" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


