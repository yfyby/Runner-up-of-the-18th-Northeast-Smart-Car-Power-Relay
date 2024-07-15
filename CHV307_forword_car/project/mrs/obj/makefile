################################################################################
# MRS Version: {"version":"1.8.4","date":"2023/02/015"}
# 自动生成的文件。不要编辑！
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include zf_driver/subdir.mk
-include zf_device/subdir.mk
-include zf_common/subdir.mk
-include user_c/subdir.mk
-include sdk/Startup/subdir.mk
-include sdk/Peripheral/subdir.mk
-include sdk/Core/subdir.mk
-include code/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_FLASH += \
CH32V307VCT6_ForwordCar.hex \

SECONDARY_LIST += \
CH32V307VCT6_ForwordCar.lst \

SECONDARY_SIZE += \
CH32V307VCT6_ForwordCar.siz \


# 所有目标
all: CH32V307VCT6_ForwordCar.elf secondary-outputs

# 工具调用
CH32V307VCT6_ForwordCar.elf: $(OBJS) $(USER_OBJS)
	@	@	riscv-none-embed-gcc -march=rv32imafc -mabi=ilp32f -msmall-data-limit=8 -mno-save-restore -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -pedantic -Wunused -Wuninitialized -Wall  -g -T "C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Ld\Link.ld" -nostartfiles -Xlinker --gc-sections -L"../" -L"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_device" -Wl,-Map,"CH32V307VCT6_ForwordCar.map" --specs=nano.specs -u _printf_float --specs=nosys.specs -Wl,--print-memory-usage -o "CH32V307VCT6_ForwordCar.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@	@
CH32V307VCT6_ForwordCar.hex: CH32V307VCT6_ForwordCar.elf
	@	riscv-none-embed-objcopy -O ihex "CH32V307VCT6_ForwordCar.elf"  "CH32V307VCT6_ForwordCar.hex"
	@	@
CH32V307VCT6_ForwordCar.lst: CH32V307VCT6_ForwordCar.elf
	@	riscv-none-embed-objdump --source --all-headers --demangle --line-numbers --wide "CH32V307VCT6_ForwordCar.elf" > "CH32V307VCT6_ForwordCar.lst"
	@	@
CH32V307VCT6_ForwordCar.siz: CH32V307VCT6_ForwordCar.elf
	@	riscv-none-embed-size --format=berkeley "CH32V307VCT6_ForwordCar.elf"
	@	@
# 其他目标
clean:
	-$(RM) $(ASM_UPPER_DEPS)$(OBJS)$(SECONDARY_FLASH)$(SECONDARY_LIST)$(SECONDARY_SIZE)$(ASM_DEPS)$(S_DEPS)$(S_UPPER_DEPS)$(C_DEPS) CH32V307VCT6_ForwordCar.elf
	-@
secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_LIST) $(SECONDARY_SIZE)

.PHONY: all clean dependents

-include ../makefile.targets
