mkfile_path := $(dir $(lastword $(MAKEFILE_LIST)))

APPL_COBJS += \
        balancer.o \
        balancer_param.o \
        parameter.o \

APPL_CXXOBJS += \
	BalancerCpp.o \
	util.o \
        ParamFileRead.o \
        Brain_Calc_Library.o \
        calibration.o \
	ColorParts.o \
	MotorParts.o \
	GyroParts.o \
	SonarParts.o \
	TouchParts.o \
	CruiseCtrl.o \
	DifficultCtrl.o \
	Controller.o \
#	lcdDraw.o \
#	logging.o \
#	ClockTimer.o \

SRCLANG := c++

ifdef CONFIG_EV3RT_APPLICATION

# Include libraries
include $(EV3RT_SDK_LIB_DIR)/libcpp-ev3/Makefile

endif

APPL_DIR += \
        $(mkfile_path)app \
        $(mkfile_path)Utility \
        $(mkfile_path)calibration \
        $(mkfile_path)controller \
        $(mkfile_path)controller/CruiseCtrl \
        $(mkfile_path)controller/DifficultCtrl \
        $(mkfile_path)parts \

INCLUDES += \
        -I$(mkfile_path)app \
        -I$(mkfile_path)Utility \
        -I$(mkfile_path)calibration \
        -I$(mkfile_path)controller \
        -I$(mkfile_path)controller/CruiseCtrl \
        -I$(mkfile_path)controller/DifficultCtrl \
        -I$(mkfile_path)parts \

# COPTS += -fno-use-cxa-atexit
