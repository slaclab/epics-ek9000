TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# Build the IOC application

PROD_IOC = ek9000Test
# ek9000Test.dbd will be created and installed
DBD += ek9000Test.dbd

ifeq ($(ENABLE_ASAN),1)
USR_CXXFLAGS += -fsanitize=address
USR_LDFLAGS += -fsanitize=address -static-libasan
endif

# ek9000Test.dbd will be made up from these files:
ek9000Test_DBD += base.dbd
ek9000Test_DBD += devEK9000.dbd
ek9000Test_DBD += asyn.dbd
ek9000Test_DBD += modbusSupport.dbd
ek9000Test_DBD += drvAsynIPPort.dbd

# Include dbd files from all support applications:

# Add all the support libraries needed by this IOC
ek9000Test_LIBS += ek9000Support 
ek9000Test_LIBS += modbus
ek9000Test_LIBS += asyn

# ek9000Test_registerRecordDeviceDriver.cpp derives from ek9000Test.dbd
ek9000Test_SRCS += ek9000Test_registerRecordDeviceDriver.cpp
#ek9000Test_SRCS += devEK9000_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
ek9000Test_SRCS_DEFAULT += ek9000TestMain.cpp
ek9000Test_SRCS_vxWorks += -nil-

# Add support from base/src/vxWorks if needed
#ek9000Test_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Finally link to the EPICS Base libraries
ek9000Test_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

