################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../core/GPIOcontrol.cpp \
../core/I2c_Codec.cpp \
../core/PRU.cpp \
../core/RTAudio.cpp \
../core/RTAudioCommandLine.cpp \
../core/Utilities.cpp 

OBJS += \
./core/GPIOcontrol.o \
./core/I2c_Codec.o \
./core/PRU.o \
./core/RTAudio.o \
./core/RTAudioCommandLine.o \
./core/Utilities.o 

CPP_DEPS += \
./core/GPIOcontrol.d \
./core/I2c_Codec.d \
./core/PRU.d \
./core/RTAudio.d \
./core/RTAudioCommandLine.d \
./core/Utilities.d 


# Each subdirectory must supply rules for building sources it contributes
core/%.o: ../core/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	arm-linux-gnueabihf-g++ -I/import/teaching/ECS732/arm-gcc/arm-linux-gnueabihf/include/xenomai/include -I/usr/arm-linux-gnueabihf/include/xenomai/include -I/import/teaching/ECS732/arm-gcc/arm-linux-gnueabihf/include -I/usr/arm-linux-gnueabihf/include/ne10 -O2 -g3 -Wall -c -fmessage-length=0 -Wpointer-arith -Wunused-result -D_GNU_SOURCE -D_REENTRANT -D__XENO__ -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


