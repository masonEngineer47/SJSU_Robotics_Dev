################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../husky_highlevel_controller/src/src/scan.cpp 

OBJS += \
./husky_highlevel_controller/src/src/scan.o 

CPP_DEPS += \
./husky_highlevel_controller/src/src/scan.d 


# Each subdirectory must supply rules for building sources it contributes
husky_highlevel_controller/src/src/%.o: ../husky_highlevel_controller/src/src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


