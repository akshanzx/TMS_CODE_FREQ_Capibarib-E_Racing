################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../.claude/worktrees/lucid-poitras-51fb1e/Core/Startup/startup_stm32g474retx.s 

S_DEPS += \
./.claude/worktrees/lucid-poitras-51fb1e/Core/Startup/startup_stm32g474retx.d 

OBJS += \
./.claude/worktrees/lucid-poitras-51fb1e/Core/Startup/startup_stm32g474retx.o 


# Each subdirectory must supply rules for building sources it contributes
.claude/worktrees/lucid-poitras-51fb1e/Core/Startup/%.o: ../.claude/worktrees/lucid-poitras-51fb1e/Core/Startup/%.s .claude/worktrees/lucid-poitras-51fb1e/Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean--2e-claude-2f-worktrees-2f-lucid-2d-poitras-2d-51fb1e-2f-Core-2f-Startup

clean--2e-claude-2f-worktrees-2f-lucid-2d-poitras-2d-51fb1e-2f-Core-2f-Startup:
	-$(RM) ./.claude/worktrees/lucid-poitras-51fb1e/Core/Startup/startup_stm32g474retx.d ./.claude/worktrees/lucid-poitras-51fb1e/Core/Startup/startup_stm32g474retx.o

.PHONY: clean--2e-claude-2f-worktrees-2f-lucid-2d-poitras-2d-51fb1e-2f-Core-2f-Startup

