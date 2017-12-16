TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# build a support library

LIBRARY_IOC += enviro

# xxxRecord.h will be created from xxxRecord.dbd
#DBDINC += xxxRecord
# install enviro.dbd into <top>/dbd
DBD += enviro.dbd

# specify all source files to be compiled and added to the library
#enviro_SRCS += xxx

enviro_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

