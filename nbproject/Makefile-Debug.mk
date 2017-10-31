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
	${OBJECTDIR}/AttitudeControl.o \
	${OBJECTDIR}/Bot_Configuration.o \
	${OBJECTDIR}/Bot_Walking.o \
	${OBJECTDIR}/InsectBot_Motions.o \
	${OBJECTDIR}/JoystickDriver.o \
	${OBJECTDIR}/KalmanFilter.o \
	${OBJECTDIR}/Kinematics3DOF.o \
	${OBJECTDIR}/LowPassFilter.o \
	${OBJECTDIR}/SensorInfo.o \
	${OBJECTDIR}/Serial_CSV_Format.o \
	${OBJECTDIR}/ServoAdjuster.o \
	${OBJECTDIR}/ServoBlaster.o \
	${OBJECTDIR}/TimeCount.o \
	${OBJECTDIR}/TimeKeeper.o \
	${OBJECTDIR}/main.o


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
	"${MAKE}"  -f nbproject/Makefile-${CND_CONF}.mk ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/legdriver_3dof_seriallink

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/legdriver_3dof_seriallink: ${OBJECTFILES}
	${MKDIR} -p ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}
	${LINK.cc} -o ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/legdriver_3dof_seriallink ${OBJECTFILES} ${LDLIBSOPTIONS}

${OBJECTDIR}/AttitudeControl.o: AttitudeControl.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/AttitudeControl.o AttitudeControl.cpp

${OBJECTDIR}/Bot_Configuration.o: Bot_Configuration.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/Bot_Configuration.o Bot_Configuration.cpp

${OBJECTDIR}/Bot_Walking.o: Bot_Walking.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/Bot_Walking.o Bot_Walking.cpp

${OBJECTDIR}/InsectBot_Motions.o: InsectBot_Motions.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/InsectBot_Motions.o InsectBot_Motions.cpp

${OBJECTDIR}/JoystickDriver.o: JoystickDriver.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/JoystickDriver.o JoystickDriver.cpp

${OBJECTDIR}/KalmanFilter.o: KalmanFilter.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/KalmanFilter.o KalmanFilter.cpp

${OBJECTDIR}/Kinematics3DOF.o: Kinematics3DOF.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/Kinematics3DOF.o Kinematics3DOF.cpp

${OBJECTDIR}/LowPassFilter.o: LowPassFilter.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/LowPassFilter.o LowPassFilter.cpp

${OBJECTDIR}/SensorInfo.o: SensorInfo.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/SensorInfo.o SensorInfo.cpp

${OBJECTDIR}/Serial_CSV_Format.o: Serial_CSV_Format.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/Serial_CSV_Format.o Serial_CSV_Format.cpp

${OBJECTDIR}/ServoAdjuster.o: ServoAdjuster.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/ServoAdjuster.o ServoAdjuster.cpp

${OBJECTDIR}/ServoBlaster.o: ServoBlaster.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/ServoBlaster.o ServoBlaster.cpp

${OBJECTDIR}/TimeCount.o: TimeCount.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/TimeCount.o TimeCount.cpp

${OBJECTDIR}/TimeKeeper.o: TimeKeeper.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/TimeKeeper.o TimeKeeper.cpp

${OBJECTDIR}/main.o: main.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/main.o main.cpp

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
