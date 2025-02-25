@ECHO OFF
CD %WORKSPACE%/SensorModule/include
IF EXIST "VersionString.h" DEL "VersionString.h"
COPY NUL "VersionString.h"
ECHO #define SENSOR_MODULE_BUILD_VERSION %SENSOR_MODULE_BUILD_VERSION% > VersionString.h
EXIT