################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../system/src/cmsis/system_ADuCM360.c \
../system/src/cmsis/vectors_ADuCM360.c 

OBJS += \
./system/src/cmsis/system_ADuCM360.o \
./system/src/cmsis/vectors_ADuCM360.o 

C_DEPS += \
./system/src/cmsis/system_ADuCM360.d \
./system/src/cmsis/vectors_ADuCM360.d 


# Each subdirectory must supply rules for building sources it contributes
system/src/cmsis/system_ADuCM360.o: ../system/src/cmsis/system_ADuCM360.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DTRACE -DOS_USE_TRACE_ITM -I"../include" -I"../system/include" -I"../system/include/cmsis" -I"../system/include/aducm360" -std=gnu11 -Wno-missing-prototypes -Wno-padded -Wno-missing-declarations -MMD -MP -MF"$(@:%.o=%.d)" -MT"system/src/cmsis/system_ADuCM360.d" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

system/src/cmsis/%.o: ../system/src/cmsis/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DTRACE -DOS_USE_TRACE_ITM -I"../include" -I"../system/include" -I"../system/include/cmsis" -I"../system/include/aducm360" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


