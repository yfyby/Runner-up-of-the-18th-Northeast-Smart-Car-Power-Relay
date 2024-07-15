################################################################################
# MRS Version: {"version":"1.8.4","date":"2023/02/015"}
# 自动生成的文件。不要编辑！
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/linjias/Desktop/car/CHV307_forword_car/project/code/ANO_DT.c \
C:/Users/linjias/Desktop/car/CHV307_forword_car/project/code/Been.c \
C:/Users/linjias/Desktop/car/CHV307_forword_car/project/code/Stadiometry.c \
C:/Users/linjias/Desktop/car/CHV307_forword_car/project/code/communication.c \
C:/Users/linjias/Desktop/car/CHV307_forword_car/project/code/four_elements_resolving.c \
C:/Users/linjias/Desktop/car/CHV307_forword_car/project/code/fuzzy.c \
C:/Users/linjias/Desktop/car/CHV307_forword_car/project/code/image.c \
C:/Users/linjias/Desktop/car/CHV307_forword_car/project/code/image_deal.c \
C:/Users/linjias/Desktop/car/CHV307_forword_car/project/code/imgproc.c \
C:/Users/linjias/Desktop/car/CHV307_forword_car/project/code/inductor.c \
C:/Users/linjias/Desktop/car/CHV307_forword_car/project/code/motor.c \
C:/Users/linjias/Desktop/car/CHV307_forword_car/project/code/mpu_angle.c \
C:/Users/linjias/Desktop/car/CHV307_forword_car/project/code/mypid.c \
C:/Users/linjias/Desktop/car/CHV307_forword_car/project/code/perspective.c \
C:/Users/linjias/Desktop/car/CHV307_forword_car/project/code/some_algorithm.c \
C:/Users/linjias/Desktop/car/CHV307_forword_car/project/code/special_contral.c \
C:/Users/linjias/Desktop/car/CHV307_forword_car/project/code/speed_mode.c \
C:/Users/linjias/Desktop/car/CHV307_forword_car/project/code/streeing.c \
C:/Users/linjias/Desktop/car/CHV307_forword_car/project/code/swj.c \
C:/Users/linjias/Desktop/car/CHV307_forword_car/project/code/system.c \
C:/Users/linjias/Desktop/car/CHV307_forword_car/project/code/ui_key.c \
C:/Users/linjias/Desktop/car/CHV307_forword_car/project/code/wheel.c 

OBJS += \
./code/ANO_DT.o \
./code/Been.o \
./code/Stadiometry.o \
./code/communication.o \
./code/four_elements_resolving.o \
./code/fuzzy.o \
./code/image.o \
./code/image_deal.o \
./code/imgproc.o \
./code/inductor.o \
./code/motor.o \
./code/mpu_angle.o \
./code/mypid.o \
./code/perspective.o \
./code/some_algorithm.o \
./code/special_contral.o \
./code/speed_mode.o \
./code/streeing.o \
./code/swj.o \
./code/system.o \
./code/ui_key.o \
./code/wheel.o 

C_DEPS += \
./code/ANO_DT.d \
./code/Been.d \
./code/Stadiometry.d \
./code/communication.d \
./code/four_elements_resolving.d \
./code/fuzzy.d \
./code/image.d \
./code/image_deal.d \
./code/imgproc.d \
./code/inductor.d \
./code/motor.d \
./code/mpu_angle.d \
./code/mypid.d \
./code/perspective.d \
./code/some_algorithm.d \
./code/special_contral.d \
./code/speed_mode.d \
./code/streeing.d \
./code/swj.d \
./code/system.d \
./code/ui_key.d \
./code/wheel.d 


# Each subdirectory must supply rules for building sources it contributes
code/ANO_DT.o: C:/Users/linjias/Desktop/car/CHV307_forword_car/project/code/ANO_DT.c
	@	@	riscv-none-embed-gcc -march=rv32imafc -mabi=ilp32f -msmall-data-limit=8 -mno-save-restore -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -pedantic -Wunused -Wuninitialized -Wall  -g -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\Libraries\doc" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Core" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Ld" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Peripheral" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Startup" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\project\user\inc" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_common" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_device" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\project\code" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_driver" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@
code/Been.o: C:/Users/linjias/Desktop/car/CHV307_forword_car/project/code/Been.c
	@	@	riscv-none-embed-gcc -march=rv32imafc -mabi=ilp32f -msmall-data-limit=8 -mno-save-restore -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -pedantic -Wunused -Wuninitialized -Wall  -g -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\Libraries\doc" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Core" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Ld" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Peripheral" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Startup" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\project\user\inc" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_common" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_device" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\project\code" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_driver" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@
code/Stadiometry.o: C:/Users/linjias/Desktop/car/CHV307_forword_car/project/code/Stadiometry.c
	@	@	riscv-none-embed-gcc -march=rv32imafc -mabi=ilp32f -msmall-data-limit=8 -mno-save-restore -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -pedantic -Wunused -Wuninitialized -Wall  -g -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\Libraries\doc" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Core" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Ld" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Peripheral" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Startup" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\project\user\inc" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_common" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_device" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\project\code" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_driver" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@
code/communication.o: C:/Users/linjias/Desktop/car/CHV307_forword_car/project/code/communication.c
	@	@	riscv-none-embed-gcc -march=rv32imafc -mabi=ilp32f -msmall-data-limit=8 -mno-save-restore -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -pedantic -Wunused -Wuninitialized -Wall  -g -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\Libraries\doc" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Core" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Ld" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Peripheral" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Startup" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\project\user\inc" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_common" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_device" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\project\code" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_driver" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@
code/four_elements_resolving.o: C:/Users/linjias/Desktop/car/CHV307_forword_car/project/code/four_elements_resolving.c
	@	@	riscv-none-embed-gcc -march=rv32imafc -mabi=ilp32f -msmall-data-limit=8 -mno-save-restore -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -pedantic -Wunused -Wuninitialized -Wall  -g -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\Libraries\doc" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Core" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Ld" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Peripheral" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Startup" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\project\user\inc" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_common" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_device" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\project\code" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_driver" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@
code/fuzzy.o: C:/Users/linjias/Desktop/car/CHV307_forword_car/project/code/fuzzy.c
	@	@	riscv-none-embed-gcc -march=rv32imafc -mabi=ilp32f -msmall-data-limit=8 -mno-save-restore -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -pedantic -Wunused -Wuninitialized -Wall  -g -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\Libraries\doc" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Core" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Ld" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Peripheral" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Startup" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\project\user\inc" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_common" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_device" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\project\code" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_driver" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@
code/image.o: C:/Users/linjias/Desktop/car/CHV307_forword_car/project/code/image.c
	@	@	riscv-none-embed-gcc -march=rv32imafc -mabi=ilp32f -msmall-data-limit=8 -mno-save-restore -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -pedantic -Wunused -Wuninitialized -Wall  -g -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\Libraries\doc" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Core" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Ld" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Peripheral" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Startup" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\project\user\inc" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_common" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_device" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\project\code" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_driver" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@
code/image_deal.o: C:/Users/linjias/Desktop/car/CHV307_forword_car/project/code/image_deal.c
	@	@	riscv-none-embed-gcc -march=rv32imafc -mabi=ilp32f -msmall-data-limit=8 -mno-save-restore -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -pedantic -Wunused -Wuninitialized -Wall  -g -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\Libraries\doc" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Core" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Ld" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Peripheral" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Startup" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\project\user\inc" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_common" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_device" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\project\code" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_driver" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@
code/imgproc.o: C:/Users/linjias/Desktop/car/CHV307_forword_car/project/code/imgproc.c
	@	@	riscv-none-embed-gcc -march=rv32imafc -mabi=ilp32f -msmall-data-limit=8 -mno-save-restore -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -pedantic -Wunused -Wuninitialized -Wall  -g -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\Libraries\doc" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Core" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Ld" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Peripheral" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Startup" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\project\user\inc" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_common" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_device" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\project\code" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_driver" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@
code/inductor.o: C:/Users/linjias/Desktop/car/CHV307_forword_car/project/code/inductor.c
	@	@	riscv-none-embed-gcc -march=rv32imafc -mabi=ilp32f -msmall-data-limit=8 -mno-save-restore -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -pedantic -Wunused -Wuninitialized -Wall  -g -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\Libraries\doc" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Core" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Ld" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Peripheral" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Startup" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\project\user\inc" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_common" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_device" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\project\code" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_driver" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@
code/motor.o: C:/Users/linjias/Desktop/car/CHV307_forword_car/project/code/motor.c
	@	@	riscv-none-embed-gcc -march=rv32imafc -mabi=ilp32f -msmall-data-limit=8 -mno-save-restore -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -pedantic -Wunused -Wuninitialized -Wall  -g -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\Libraries\doc" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Core" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Ld" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Peripheral" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Startup" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\project\user\inc" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_common" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_device" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\project\code" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_driver" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@
code/mpu_angle.o: C:/Users/linjias/Desktop/car/CHV307_forword_car/project/code/mpu_angle.c
	@	@	riscv-none-embed-gcc -march=rv32imafc -mabi=ilp32f -msmall-data-limit=8 -mno-save-restore -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -pedantic -Wunused -Wuninitialized -Wall  -g -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\Libraries\doc" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Core" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Ld" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Peripheral" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Startup" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\project\user\inc" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_common" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_device" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\project\code" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_driver" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@
code/mypid.o: C:/Users/linjias/Desktop/car/CHV307_forword_car/project/code/mypid.c
	@	@	riscv-none-embed-gcc -march=rv32imafc -mabi=ilp32f -msmall-data-limit=8 -mno-save-restore -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -pedantic -Wunused -Wuninitialized -Wall  -g -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\Libraries\doc" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Core" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Ld" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Peripheral" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Startup" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\project\user\inc" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_common" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_device" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\project\code" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_driver" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@
code/perspective.o: C:/Users/linjias/Desktop/car/CHV307_forword_car/project/code/perspective.c
	@	@	riscv-none-embed-gcc -march=rv32imafc -mabi=ilp32f -msmall-data-limit=8 -mno-save-restore -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -pedantic -Wunused -Wuninitialized -Wall  -g -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\Libraries\doc" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Core" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Ld" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Peripheral" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Startup" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\project\user\inc" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_common" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_device" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\project\code" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_driver" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@
code/some_algorithm.o: C:/Users/linjias/Desktop/car/CHV307_forword_car/project/code/some_algorithm.c
	@	@	riscv-none-embed-gcc -march=rv32imafc -mabi=ilp32f -msmall-data-limit=8 -mno-save-restore -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -pedantic -Wunused -Wuninitialized -Wall  -g -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\Libraries\doc" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Core" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Ld" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Peripheral" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Startup" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\project\user\inc" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_common" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_device" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\project\code" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_driver" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@
code/special_contral.o: C:/Users/linjias/Desktop/car/CHV307_forword_car/project/code/special_contral.c
	@	@	riscv-none-embed-gcc -march=rv32imafc -mabi=ilp32f -msmall-data-limit=8 -mno-save-restore -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -pedantic -Wunused -Wuninitialized -Wall  -g -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\Libraries\doc" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Core" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Ld" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Peripheral" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Startup" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\project\user\inc" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_common" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_device" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\project\code" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_driver" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@
code/speed_mode.o: C:/Users/linjias/Desktop/car/CHV307_forword_car/project/code/speed_mode.c
	@	@	riscv-none-embed-gcc -march=rv32imafc -mabi=ilp32f -msmall-data-limit=8 -mno-save-restore -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -pedantic -Wunused -Wuninitialized -Wall  -g -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\Libraries\doc" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Core" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Ld" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Peripheral" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Startup" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\project\user\inc" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_common" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_device" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\project\code" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_driver" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@
code/streeing.o: C:/Users/linjias/Desktop/car/CHV307_forword_car/project/code/streeing.c
	@	@	riscv-none-embed-gcc -march=rv32imafc -mabi=ilp32f -msmall-data-limit=8 -mno-save-restore -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -pedantic -Wunused -Wuninitialized -Wall  -g -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\Libraries\doc" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Core" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Ld" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Peripheral" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Startup" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\project\user\inc" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_common" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_device" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\project\code" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_driver" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@
code/swj.o: C:/Users/linjias/Desktop/car/CHV307_forword_car/project/code/swj.c
	@	@	riscv-none-embed-gcc -march=rv32imafc -mabi=ilp32f -msmall-data-limit=8 -mno-save-restore -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -pedantic -Wunused -Wuninitialized -Wall  -g -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\Libraries\doc" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Core" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Ld" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Peripheral" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Startup" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\project\user\inc" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_common" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_device" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\project\code" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_driver" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@
code/system.o: C:/Users/linjias/Desktop/car/CHV307_forword_car/project/code/system.c
	@	@	riscv-none-embed-gcc -march=rv32imafc -mabi=ilp32f -msmall-data-limit=8 -mno-save-restore -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -pedantic -Wunused -Wuninitialized -Wall  -g -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\Libraries\doc" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Core" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Ld" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Peripheral" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Startup" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\project\user\inc" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_common" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_device" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\project\code" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_driver" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@
code/ui_key.o: C:/Users/linjias/Desktop/car/CHV307_forword_car/project/code/ui_key.c
	@	@	riscv-none-embed-gcc -march=rv32imafc -mabi=ilp32f -msmall-data-limit=8 -mno-save-restore -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -pedantic -Wunused -Wuninitialized -Wall  -g -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\Libraries\doc" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Core" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Ld" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Peripheral" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Startup" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\project\user\inc" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_common" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_device" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\project\code" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_driver" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@
code/wheel.o: C:/Users/linjias/Desktop/car/CHV307_forword_car/project/code/wheel.c
	@	@	riscv-none-embed-gcc -march=rv32imafc -mabi=ilp32f -msmall-data-limit=8 -mno-save-restore -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -pedantic -Wunused -Wuninitialized -Wall  -g -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\Libraries\doc" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Core" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Ld" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Peripheral" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\sdk\Startup" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\project\user\inc" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_common" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_device" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\project\code" -I"C:\Users\linjias\Desktop\car\CHV307_forword_car\libraries\zf_driver" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@

