################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/14695/STM32CubeIDE/workspace/gophercan-lib/GopherCAN.c \
C:/Users/14695/STM32CubeIDE/workspace/gophercan-lib/GopherCAN_ids.c \
C:/Users/14695/STM32CubeIDE/workspace/gophercan-lib/GopherCAN_names.c \
C:/Users/14695/STM32CubeIDE/workspace/gophercan-lib/GopherCAN_ring_buffer.c 

OBJS += \
./Core/gophercan-lib/GopherCAN.o \
./Core/gophercan-lib/GopherCAN_ids.o \
./Core/gophercan-lib/GopherCAN_names.o \
./Core/gophercan-lib/GopherCAN_ring_buffer.o 

C_DEPS += \
./Core/gophercan-lib/GopherCAN.d \
./Core/gophercan-lib/GopherCAN_ids.d \
./Core/gophercan-lib/GopherCAN_names.d \
./Core/gophercan-lib/GopherCAN_ring_buffer.d 


# Each subdirectory must supply rules for building sources it contributes
Core/gophercan-lib/GopherCAN.o: C:/Users/14695/STM32CubeIDE/workspace/gophercan-lib/GopherCAN.c Core/gophercan-lib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/14695/STM32CubeIDE/workspace/gophercan-lib" -I"C:/Users/14695/STM32CubeIDE/workspace/Gopher_Sense" -I"C:/Users/14695/STM32CubeIDE/workspace/Gopher_Sense/Build" -I"C:/Users/14695/STM32CubeIDE/workspace/Gopher_Sense/lib" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/gophercan-lib/GopherCAN_ids.o: C:/Users/14695/STM32CubeIDE/workspace/gophercan-lib/GopherCAN_ids.c Core/gophercan-lib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/14695/STM32CubeIDE/workspace/gophercan-lib" -I"C:/Users/14695/STM32CubeIDE/workspace/Gopher_Sense" -I"C:/Users/14695/STM32CubeIDE/workspace/Gopher_Sense/Build" -I"C:/Users/14695/STM32CubeIDE/workspace/Gopher_Sense/lib" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/gophercan-lib/GopherCAN_names.o: C:/Users/14695/STM32CubeIDE/workspace/gophercan-lib/GopherCAN_names.c Core/gophercan-lib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/14695/STM32CubeIDE/workspace/gophercan-lib" -I"C:/Users/14695/STM32CubeIDE/workspace/Gopher_Sense" -I"C:/Users/14695/STM32CubeIDE/workspace/Gopher_Sense/Build" -I"C:/Users/14695/STM32CubeIDE/workspace/Gopher_Sense/lib" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/gophercan-lib/GopherCAN_ring_buffer.o: C:/Users/14695/STM32CubeIDE/workspace/gophercan-lib/GopherCAN_ring_buffer.c Core/gophercan-lib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/14695/STM32CubeIDE/workspace/gophercan-lib" -I"C:/Users/14695/STM32CubeIDE/workspace/Gopher_Sense" -I"C:/Users/14695/STM32CubeIDE/workspace/Gopher_Sense/Build" -I"C:/Users/14695/STM32CubeIDE/workspace/Gopher_Sense/lib" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-gophercan-2d-lib

clean-Core-2f-gophercan-2d-lib:
	-$(RM) ./Core/gophercan-lib/GopherCAN.d ./Core/gophercan-lib/GopherCAN.o ./Core/gophercan-lib/GopherCAN.su ./Core/gophercan-lib/GopherCAN_ids.d ./Core/gophercan-lib/GopherCAN_ids.o ./Core/gophercan-lib/GopherCAN_ids.su ./Core/gophercan-lib/GopherCAN_names.d ./Core/gophercan-lib/GopherCAN_names.o ./Core/gophercan-lib/GopherCAN_names.su ./Core/gophercan-lib/GopherCAN_ring_buffer.d ./Core/gophercan-lib/GopherCAN_ring_buffer.o ./Core/gophercan-lib/GopherCAN_ring_buffer.su

.PHONY: clean-Core-2f-gophercan-2d-lib

