################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include system/subdir.mk
-include src/subdir.mk
-include RTE/Device/ADuCM360/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
OBJS := $(SRC_OBJS)
# Quote makefile variable : OBJS 
OBJS := $(subst \ ,@, $(OBJS))
OBJS := $(foreach var, $(OBJS), "$(subst @, ,$(var))")
# Quote makefile variable : C_DEPS 
C_DEPS := $(subst \ ,@, $(C_DEPS))
C_DEPS := $(foreach var, $(C_DEPS), "$(subst @, ,$(var))")

# All Target
all: ADuCM360_demo_cn0396

# Tool invocations
ADuCM360_demo_cn0396: $(USER_OBJS) $(SRC_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: CrossCore GCC ARM Embedded C Linker'
	arm-none-eabi-gcc -T/Users/amclach/Work/BitBucket/eval-adicup360/projects/ADuCM360_demo_cn0396/RTE/Device/ADuCM360/ADuCM360.ld -Wl,--gc-sections -mcpu=cortex-m3 -mthumb -specs=nosys.specs -o  "ADuCM360_demo_cn0396" @input-file.txt -lm
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(C_DEPS) ADuCM360_demo_cn0396
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
