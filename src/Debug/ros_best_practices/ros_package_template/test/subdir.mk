################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../ros_best_practices/ros_package_template/test/AlgorithmTest.cpp \
../ros_best_practices/ros_package_template/test/test_ros_package_template.cpp 

OBJS += \
./ros_best_practices/ros_package_template/test/AlgorithmTest.o \
./ros_best_practices/ros_package_template/test/test_ros_package_template.o 

CPP_DEPS += \
./ros_best_practices/ros_package_template/test/AlgorithmTest.d \
./ros_best_practices/ros_package_template/test/test_ros_package_template.d 


# Each subdirectory must supply rules for building sources it contributes
ros_best_practices/ros_package_template/test/%.o: ../ros_best_practices/ros_package_template/test/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


