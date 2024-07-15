################################################################################
# MRS Version: {"version":"1.8.4","date":"2023/02/015"}
# 自动生成的文件。不要编辑！
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/linjias/Desktop/car/CHV307_forword_car/libraries/sdk/Core/core_riscv.c 

OBJS += \
./sdk/Core/core_riscv.o 

C_DEPS += \
./sdk/Core/core_riscv.d 


# Each subdirectory must supply rules for building sources it contributes
sdk/Core/core_riscv.o: C:/Users/linjias/Desktop/car/CHV307_forword_car/libraries/sdk/Core/core_riscv.c
	@	@	riscv-none-embed-gcc -march=rv32imafc -mabi=ilp32f -msmall-data-limit=8 -mno-save-restore -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -pedantic -Wunused -Wuninitialized -Wall  -g -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\Libraries\doc" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Core" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Ld" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Peripheral" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Startup" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\project\user\inc" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_common" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_device" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\project\code" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_driver" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@

