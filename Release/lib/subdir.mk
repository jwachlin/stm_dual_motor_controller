################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../lib/adc_interface.c \
../lib/can_messages.c \
../lib/encoder_interface.c \
../lib/impedance_controller.c \
../lib/inverse_kinematics.c \
../lib/motion_primitives.c \
../lib/pid_controller.c 

OBJS += \
./lib/adc_interface.o \
./lib/can_messages.o \
./lib/encoder_interface.o \
./lib/impedance_controller.o \
./lib/inverse_kinematics.o \
./lib/motion_primitives.o \
./lib/pid_controller.o 

C_DEPS += \
./lib/adc_interface.d \
./lib/can_messages.d \
./lib/encoder_interface.d \
./lib/impedance_controller.d \
./lib/inverse_kinematics.d \
./lib/motion_primitives.d \
./lib/pid_controller.d 


# Each subdirectory must supply rules for building sources it contributes
lib/adc_interface.o: ../lib/adc_interface.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -D__FPU_PRESENT -DSTM32F407xx -c -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../lib -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O2 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"lib/adc_interface.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
lib/can_messages.o: ../lib/can_messages.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -D__FPU_PRESENT -DSTM32F407xx -c -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../lib -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O2 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"lib/can_messages.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
lib/encoder_interface.o: ../lib/encoder_interface.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -D__FPU_PRESENT -DSTM32F407xx -c -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../lib -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O2 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"lib/encoder_interface.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
lib/impedance_controller.o: ../lib/impedance_controller.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -D__FPU_PRESENT -DSTM32F407xx -c -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../lib -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O2 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"lib/impedance_controller.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
lib/inverse_kinematics.o: ../lib/inverse_kinematics.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -D__FPU_PRESENT -DSTM32F407xx -c -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../lib -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O2 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"lib/inverse_kinematics.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
lib/motion_primitives.o: ../lib/motion_primitives.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -D__FPU_PRESENT -DSTM32F407xx -c -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../lib -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O2 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"lib/motion_primitives.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
lib/pid_controller.o: ../lib/pid_controller.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -D__FPU_PRESENT -DSTM32F407xx -c -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../lib -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O2 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"lib/pid_controller.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

