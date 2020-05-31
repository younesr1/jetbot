# PATHS
BUILD_FOLDER := build
APPS_FOLDER := apps
LIBRARY_FOLDER := library
COMMS_FOLDER := communications
CONTROLLER_FOLDER := controller
LIB_PATH := $(LIBRARY_FOLDER)
BUILD_PATH := $(BUILD_FOLDER)
APPS_PATH := $(APPS_FOLDER)
COMMS_PATH := $(LIB_PATH)/$(COMMS_FOLDER)
CONTROLLER_PATH := $(LIB_PATH)/$(CONTROLLER_FOLDER)
# COMPILATION
CPP = g++
INCLUDE_FLAG = -I
# DEPENDENCIES
INCLUDE = $(COMMS_PATH)/*.h #add all headers here
INCLUDE += $(CONTROLLER_PATH)/*.h
# OBJECTS
OBJS = $(BUILD_PATH)/main.o
OBJS += $(BUILD_PATH)/client.o
OBJS += $(BUILD_PATH)/server.o
OBJS += $(BUILD_PATH)/controller.o



# $(BUILD_PATH)/%.o: %.cpp $(INCLUDE) \
	$(CPP) -c -o $@ $< $(INCLUDE_FLAG) $(INCLUDE) \
$(BUILD_PATH)/jetbot: $(OBJS) \
	$(CPP) -o $@ $(OBJS) $(INCLUDE_FLAG) $(INCLUDE)
################################################################################################################################################
jetbot: $(OBJS)
	$(CPP) $^ -o $(BUILD_PATH)/$@
	+@echo "====================================================="
	+@echo "=============== Compiled Successfuly ================"
	+@echo "====================================================="
$(BUILD_PATH)/main.o: apps/jetbot/main.cpp
	+@echo "Compile: main.cpp"
	$(CPP) -c $(APPS_PATH)/jetbot/main.cpp -I library/controller -o $(BUILD_PATH)/main.o
$(BUILD_PATH)/client.o: library/communications/client.cpp library/communications/client.h
	+@echo "Compile: client.cpp"
	$(CPP) -c $(COMMS_PATH)/client.cpp -I config -o $(BUILD_PATH)/client.o
$(BUILD_PATH)/server.o: library/communications/server.cpp library/communications/server.h
	+@echo "Compile: server.cpp"
	$(CPP) -c $(COMMS_PATH)/server.cpp -I config -o $(BUILD_PATH)/server.o
$(BUILD_PATH)/controller.o: library/controller/controller.cpp library/controller/controller.h
	+@echo "Compile: controller.cpp"
	$(CPP) -c $(CONTROLLER_PATH)/controller.cpp -o $(BUILD_PATH)/controller.o
clean:
	rm -rf $(BUILD_PATH)/*