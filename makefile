# PATHS
BUILD_FOLDER := build
APPS_FOLDER := apps
LIBRARY_FOLDER := library
CONTROLLER_FOLDER := controller
MOTOR_FOLDER:= motor
CONFIG_FOLER := config
LIB_PATH := $(LIBRARY_FOLDER)
BUILD_PATH := $(BUILD_FOLDER)
APPS_PATH := $(APPS_FOLDER)
CONTROLLER_PATH := $(LIB_PATH)/$(CONTROLLER_FOLDER)
MOTOR_PATH := $(LIB_PATH)/$(MOTOR_FOLDER)
CONFIG_PATH := $(CONFIG_FOLER)
# COMPILATION
CPP = g++
# DEPENDENCIES
INCLUDES += -I$(CONTROLLER_PATH)
INCLUDES += -I$(CONFIG_PATH)
INCLUDES += -I$(MOTOR_PATH)
# OBJECTS
OBJS += $(BUILD_PATH)/jetbot.o
OBJS += $(BUILD_PATH)/controller.o
OBJS += $(BUILD_PATH)/adafruitdcmotor.o
OBJS += $(BUILD_PATH)/adafruitmotorhat.o
OBJS += $(BUILD_PATH)/drivetrain.o
OBJS += $(BUILD_PATH)/i2cdevice.o
OBJS += $(BUILD_PATH)/pwm.o
#THREADING
THREAD_LIB := #-pthread

################################################################################################################################################
jetbot: $(OBJS) $(BUILD_PATH)/jetbot.o
	$(CPP) $^ $(THREAD_LIB) -o $(BUILD_PATH)/$@.run
	+@echo "====================================================="
	+@echo "=============== Compiled Successfuly ================"
	+@echo "====================================================="

$(BUILD_PATH)/jetbot.o: $(APPS_PATH)/jetbot.cpp
	mkdir $(BUILD_FOLDER)
	+@echo "Compile: jetbot.cpp"
	$(CPP) -c $(APPS_PATH)/jetbot.cpp $(INCLUDES) -o $@

$(BUILD_PATH)/controller.o: $(CONTROLLER_PATH)/controller.cpp $(CONTROLLER_PATH)/controller.h
	+@echo "Compile: controller.cpp"
	$(CPP) -c $< $(INCLUDES) -o $@

$(BUILD_PATH)/adafruitdcmotor.o: $(MOTOR_PATH)/adafruitdcmotor.cpp $(MOTOR_PATH)/adafruitdcmotor.h
	+@echo "Compile: adafruitdcmotor.cpp"
	$(CPP) -c $< $(INCLUDES) -o $@

$(BUILD_PATH)/adafruitmotorhat.o: $(MOTOR_PATH)/adafruitmotorhat.cpp $(MOTOR_PATH)/adafruitmotorhat.h
	+@echo "Compile: adafruitmotorhat.cpp"
	$(CPP) -c $< $(INCLUDES) -o $@

$(BUILD_PATH)/drivetrain.o: $(MOTOR_PATH)/drivetrain.cpp $(MOTOR_PATH)/drivetrain.h
	+@echo "Compile: drivetrain.cpp"
	$(CPP) -c $< $(INCLUDES) -o $@

$(BUILD_PATH)/i2cdevice.o: $(MOTOR_PATH)/i2cdevice.cpp $(MOTOR_PATH)/i2cdevice.h
	+@echo "Compile: i2cdevice.cpp"
	$(CPP) -c $< $(INCLUDES) -o $@

$(BUILD_PATH)/pwm.o: $(MOTOR_PATH)/pwm.cpp $(MOTOR_PATH)/pwm.h
	+@echo "Compile: pwm.cpp"
	$(CPP) -c $< $(INCLUDES) -o $@	

clean:
	rm -rf $(BUILD_PATH)
	+@echo "====================================================="
	+@echo "=============== Cleaned Build Folder ================"
	+@echo "====================================================="