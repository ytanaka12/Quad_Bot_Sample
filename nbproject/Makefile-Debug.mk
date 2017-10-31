#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Environment
MKDIR=mkdir
CP=cp
GREP=grep
NM=nm
CCADMIN=CCadmin
RANLIB=ranlib
CC=gcc
CCC=g++
CXX=g++
FC=gfortran
AS=as

# Macros
CND_PLATFORM=GNU-Linux
CND_DLIB_EXT=so
CND_CONF=Debug
CND_DISTDIR=dist
CND_BUILDDIR=build

# Include project Makefile
include Makefile

# Object Directory
OBJECTDIR=${CND_BUILDDIR}/${CND_CONF}/${CND_PLATFORM}

# Object Files
OBJECTFILES= \
	${OBJECTDIR}/main.o \
	${OBJECTDIR}/source/AttitudeControl.o \
	${OBJECTDIR}/source/Bot_Configuration.o \
	${OBJECTDIR}/source/Bot_Walking.o \
	${OBJECTDIR}/source/InsectBot_Motions.o \
	${OBJECTDIR}/source/JoystickDriver.o \
	${OBJECTDIR}/source/KalmanFilter.o \
	${OBJECTDIR}/source/Kinematics3DOF.o \
	${OBJECTDIR}/source/LowPassFilter.o \
	${OBJECTDIR}/source/SensorInfo.o \
	${OBJECTDIR}/source/Serial_CSV_Format.o \
	${OBJECTDIR}/source/ServoAdjuster.o \
	${OBJECTDIR}/source/ServoBlaster.o \
	${OBJECTDIR}/source/TimeCount.o \
	${OBJECTDIR}/source/TimeKeeper.o


# C Compiler Flags
CFLAGS=

# CC Compiler Flags
CCFLAGS=-lwiringPi -std=c++11
CXXFLAGS=-lwiringPi -std=c++11

# Fortran Compiler Flags
FFLAGS=

# Assembler Flags
ASFLAGS=

# Link Libraries and Options
LDLIBSOPTIONS=

# Build Targets
.build-conf: ${BUILD_SUBPROJECTS}
	"${MAKE}"  -f nbproject/Makefile-${CND_CONF}.mk ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/quad_bot_sample

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/quad_bot_sample: ${OBJECTFILES}
	${MKDIR} -p ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}
	${LINK.cc} -o ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/quad_bot_sample ${OBJECTFILES} ${LDLIBSOPTIONS}

${OBJECTDIR}/main.o: main.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -Iinclude -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/main.o main.cpp

${OBJECTDIR}/source/AttitudeControl.o: source/AttitudeControl.cpp
	${MKDIR} -p ${OBJECTDIR}/source
	${RM} "$@.d"
	$(COMPILE.cc) -g -Iinclude -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/source/AttitudeControl.o source/AttitudeControl.cpp

${OBJECTDIR}/source/Bot_Configuration.o: source/Bot_Configuration.cpp
	${MKDIR} -p ${OBJECTDIR}/source
	${RM} "$@.d"
	$(COMPILE.cc) -g -Iinclude -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/source/Bot_Configuration.o source/Bot_Configuration.cpp

${OBJECTDIR}/source/Bot_Walking.o: source/Bot_Walking.cpp
	${MKDIR} -p ${OBJECTDIR}/source
	${RM} "$@.d"
	$(COMPILE.cc) -g -Iinclude -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/source/Bot_Walking.o source/Bot_Walking.cpp

${OBJECTDIR}/source/InsectBot_Motions.o: source/InsectBot_Motions.cpp
	${MKDIR} -p ${OBJECTDIR}/source
	${RM} "$@.d"
	$(COMPILE.cc) -g -Iinclude -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/source/InsectBot_Motions.o source/InsectBot_Motions.cpp

${OBJECTDIR}/source/JoystickDriver.o: source/JoystickDriver.cpp
	${MKDIR} -p ${OBJECTDIR}/source
	${RM} "$@.d"
	$(COMPILE.cc) -g -Iinclude -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/source/JoystickDriver.o source/JoystickDriver.cpp

${OBJECTDIR}/source/KalmanFilter.o: source/KalmanFilter.cpp
	${MKDIR} -p ${OBJECTDIR}/source
	${RM} "$@.d"
	$(COMPILE.cc) -g -Iinclude -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/source/KalmanFilter.o source/KalmanFilter.cpp

${OBJECTDIR}/source/Kinematics3DOF.o: source/Kinematics3DOF.cpp
	${MKDIR} -p ${OBJECTDIR}/source
	${RM} "$@.d"
	$(COMPILE.cc) -g -Iinclude -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/source/Kinematics3DOF.o source/Kinematics3DOF.cpp

${OBJECTDIR}/source/LowPassFilter.o: source/LowPassFilter.cpp
	${MKDIR} -p ${OBJECTDIR}/source
	${RM} "$@.d"
	$(COMPILE.cc) -g -Iinclude -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/source/LowPassFilter.o source/LowPassFilter.cpp

${OBJECTDIR}/source/SensorInfo.o: source/SensorInfo.cpp
	${MKDIR} -p ${OBJECTDIR}/source
	${RM} "$@.d"
	$(COMPILE.cc) -g -Iinclude -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/source/SensorInfo.o source/SensorInfo.cpp

${OBJECTDIR}/source/Serial_CSV_Format.o: source/Serial_CSV_Format.cpp
	${MKDIR} -p ${OBJECTDIR}/source
	${RM} "$@.d"
	$(COMPILE.cc) -g -Iinclude -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/source/Serial_CSV_Format.o source/Serial_CSV_Format.cpp

${OBJECTDIR}/source/ServoAdjuster.o: source/ServoAdjuster.cpp
	${MKDIR} -p ${OBJECTDIR}/source
	${RM} "$@.d"
	$(COMPILE.cc) -g -Iinclude -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/source/ServoAdjuster.o source/ServoAdjuster.cpp

${OBJECTDIR}/source/ServoBlaster.o: source/ServoBlaster.cpp
	${MKDIR} -p ${OBJECTDIR}/source
	${RM} "$@.d"
	$(COMPILE.cc) -g -Iinclude -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/source/ServoBlaster.o source/ServoBlaster.cpp

${OBJECTDIR}/source/TimeCount.o: source/TimeCount.cpp
	${MKDIR} -p ${OBJECTDIR}/source
	${RM} "$@.d"
	$(COMPILE.cc) -g -Iinclude -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/source/TimeCount.o source/TimeCount.cpp

${OBJECTDIR}/source/TimeKeeper.o: source/TimeKeeper.cpp
	${MKDIR} -p ${OBJECTDIR}/source
	${RM} "$@.d"
	$(COMPILE.cc) -g -Iinclude -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/source/TimeKeeper.o source/TimeKeeper.cpp

# Subprojects
.build-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r ${CND_BUILDDIR}/${CND_CONF}

# Subprojects
.clean-subprojects:

# Enable dependency checking
.dep.inc: .depcheck-impl

include .dep.inc
