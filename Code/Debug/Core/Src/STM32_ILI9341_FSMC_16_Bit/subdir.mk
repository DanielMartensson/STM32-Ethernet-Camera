################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/STM32_ILI9341_FSMC_16_Bit/STLogo.c \
../Core/Src/STM32_ILI9341_FSMC_16_Bit/font12.c \
../Core/Src/STM32_ILI9341_FSMC_16_Bit/font16.c \
../Core/Src/STM32_ILI9341_FSMC_16_Bit/font20.c \
../Core/Src/STM32_ILI9341_FSMC_16_Bit/font24.c \
../Core/Src/STM32_ILI9341_FSMC_16_Bit/font8.c \
../Core/Src/STM32_ILI9341_FSMC_16_Bit/ili9341.c 

OBJS += \
./Core/Src/STM32_ILI9341_FSMC_16_Bit/STLogo.o \
./Core/Src/STM32_ILI9341_FSMC_16_Bit/font12.o \
./Core/Src/STM32_ILI9341_FSMC_16_Bit/font16.o \
./Core/Src/STM32_ILI9341_FSMC_16_Bit/font20.o \
./Core/Src/STM32_ILI9341_FSMC_16_Bit/font24.o \
./Core/Src/STM32_ILI9341_FSMC_16_Bit/font8.o \
./Core/Src/STM32_ILI9341_FSMC_16_Bit/ili9341.o 

C_DEPS += \
./Core/Src/STM32_ILI9341_FSMC_16_Bit/STLogo.d \
./Core/Src/STM32_ILI9341_FSMC_16_Bit/font12.d \
./Core/Src/STM32_ILI9341_FSMC_16_Bit/font16.d \
./Core/Src/STM32_ILI9341_FSMC_16_Bit/font20.d \
./Core/Src/STM32_ILI9341_FSMC_16_Bit/font24.d \
./Core/Src/STM32_ILI9341_FSMC_16_Bit/font8.d \
./Core/Src/STM32_ILI9341_FSMC_16_Bit/ili9341.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/STM32_ILI9341_FSMC_16_Bit/%.o Core/Src/STM32_ILI9341_FSMC_16_Bit/%.su: ../Core/Src/STM32_ILI9341_FSMC_16_Bit/%.c Core/Src/STM32_ILI9341_FSMC_16_Bit/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F407xx -c -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Include -I../LWIP/App -I../LWIP/Target -I../Middlewares/Third_Party/LwIP/src/include -I../Middlewares/Third_Party/LwIP/system -I../Drivers/BSP/Components/dp83848 -I../Middlewares/Third_Party/LwIP/src/include/netif/ppp -I../Middlewares/Third_Party/LwIP/src/include/lwip -I../Middlewares/Third_Party/LwIP/src/include/lwip/apps -I../Middlewares/Third_Party/LwIP/src/include/lwip/priv -I../Middlewares/Third_Party/LwIP/src/include/lwip/prot -I../Middlewares/Third_Party/LwIP/src/include/netif -I../Middlewares/Third_Party/LwIP/src/include/compat/posix -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/arpa -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/net -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/sys -I../Middlewares/Third_Party/LwIP/src/include/compat/stdc -I../Middlewares/Third_Party/LwIP/system/arch -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-STM32_ILI9341_FSMC_16_Bit

clean-Core-2f-Src-2f-STM32_ILI9341_FSMC_16_Bit:
	-$(RM) ./Core/Src/STM32_ILI9341_FSMC_16_Bit/STLogo.d ./Core/Src/STM32_ILI9341_FSMC_16_Bit/STLogo.o ./Core/Src/STM32_ILI9341_FSMC_16_Bit/STLogo.su ./Core/Src/STM32_ILI9341_FSMC_16_Bit/font12.d ./Core/Src/STM32_ILI9341_FSMC_16_Bit/font12.o ./Core/Src/STM32_ILI9341_FSMC_16_Bit/font12.su ./Core/Src/STM32_ILI9341_FSMC_16_Bit/font16.d ./Core/Src/STM32_ILI9341_FSMC_16_Bit/font16.o ./Core/Src/STM32_ILI9341_FSMC_16_Bit/font16.su ./Core/Src/STM32_ILI9341_FSMC_16_Bit/font20.d ./Core/Src/STM32_ILI9341_FSMC_16_Bit/font20.o ./Core/Src/STM32_ILI9341_FSMC_16_Bit/font20.su ./Core/Src/STM32_ILI9341_FSMC_16_Bit/font24.d ./Core/Src/STM32_ILI9341_FSMC_16_Bit/font24.o ./Core/Src/STM32_ILI9341_FSMC_16_Bit/font24.su ./Core/Src/STM32_ILI9341_FSMC_16_Bit/font8.d ./Core/Src/STM32_ILI9341_FSMC_16_Bit/font8.o ./Core/Src/STM32_ILI9341_FSMC_16_Bit/font8.su ./Core/Src/STM32_ILI9341_FSMC_16_Bit/ili9341.d ./Core/Src/STM32_ILI9341_FSMC_16_Bit/ili9341.o ./Core/Src/STM32_ILI9341_FSMC_16_Bit/ili9341.su

.PHONY: clean-Core-2f-Src-2f-STM32_ILI9341_FSMC_16_Bit

