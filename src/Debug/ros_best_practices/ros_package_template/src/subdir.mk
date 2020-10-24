################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../ros_best_practices/ros_package_template/src/Algorithm.cpp \
../ros_best_practices/ros_package_template/src/RosPackageTemplate.cpp \
../ros_best_practices/ros_package_template/src/ros_package_template_node.cpp 

OBJS += \
./ros_best_practices/ros_package_template/src/Algorithm.o \
./ros_best_practices/ros_package_template/src/RosPackageTemplate.o \
./ros_best_practices/ros_package_template/src/ros_package_template_node.o 

CPP_DEPS += \
./ros_best_practices/ros_package_template/src/Algorithm.d \
./ros_best_practices/ros_package_template/src/RosPackageTemplate.d \
./ros_best_practices/ros_package_template/src/ros_package_template_node.d 


# Each subdirectory must supply rules for building sources it contributes
ros_best_practices/ros_package_template/src/%.o: ../ros_best_practices/ros_package_template/src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


