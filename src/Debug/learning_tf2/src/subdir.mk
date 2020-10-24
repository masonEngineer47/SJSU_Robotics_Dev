################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../learning_tf2/src/frame_tf2_broadcaster.cpp \
../learning_tf2/src/static_turtle_tf2_broadcaster.cpp \
../learning_tf2/src/turtle_tf2_broadcaster.cpp \
../learning_tf2/src/turtle_tf2_listener.cpp 

OBJS += \
./learning_tf2/src/frame_tf2_broadcaster.o \
./learning_tf2/src/static_turtle_tf2_broadcaster.o \
./learning_tf2/src/turtle_tf2_broadcaster.o \
./learning_tf2/src/turtle_tf2_listener.o 

CPP_DEPS += \
./learning_tf2/src/frame_tf2_broadcaster.d \
./learning_tf2/src/static_turtle_tf2_broadcaster.d \
./learning_tf2/src/turtle_tf2_broadcaster.d \
./learning_tf2/src/turtle_tf2_listener.d 


# Each subdirectory must supply rules for building sources it contributes
learning_tf2/src/%.o: ../learning_tf2/src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


