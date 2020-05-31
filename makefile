# PATHS
BUILD_FOLDER := build
APPS_FOLDER := apps
LIBRARY_FOLDER := library
COMMS_FOLDER := communications
CONTROLLER_FOLDER := controller
LIB_PATH := /$(LIBRARY_FOLDER)
BUILD_PATH := /$(BUILD_FOLDER)
APPS_PATH := /$(APPS_FOLDER)
COMMS_PATH := $(LIB_PATH)/$(COMMS_FOLDER)
CONTROLLER_PATH := $(LIB_PATH)/$(CONTROLLER_FOLDER)
# COMPILATION
CPP = g++
INCLUDE_FLAG = -I
# DEPENDENCIES
INCLUDE = $(COMMS_PATH)/*.h #add all headers here
INCLUDE += $(CONTROLLER_PATH)/*.h
# OBJECTS
OBJS = $(BUILD_PATH)/*.o #add all objects here

# $(BUILD_PATH)/%.o: %.cpp $(INCLUDE) \
	$(CPP) -c -o $@ $< $(INCLUDE_FLAG) $(INCLUDE) \
$(BUILD_PATH)/jetbot: $(OBJS) \
	$(CPP) -o $@ $(OBJS) $(INCLUDE_FLAG) $(INCLUDE)
################################################################################################################################################
jetbot.out: $(BUILD_PATH)main.o $(BUILD_PATH)/client.o $(BUILD_PATH)/server.o $(BUILD_PATH)/controller.o
	g++ $(BUILD_PATH)main.o $(BUILD_PATH)client.o $(BUILD_PATH)server.o $(BUILD_PATH)controller.o -o $(BUILD_PATH)/jetbot.out
main.o: apps/jetbot/main.cpp
	g++ -c apps/jetbot/main.cpp -I library/controller -o $(BUILD_PATH)/main.o
client.o: library/communications/client.cpp library/communications/client.h
	g++ -c library/communications/client.cpp -I config -o $(BUILD_PATH)/client.o
server.o: library/communications/server.cpp library/communications/server.h
	g++ -c library/communications/server.cpp -I config -o $(BUILD_PATH)/server.o
controller.o: library/controller/controller.cpp library/controller/controller.h
	g++ -c library/controller/controller.cpp -o $(BUILD_PATH)/controller.o



