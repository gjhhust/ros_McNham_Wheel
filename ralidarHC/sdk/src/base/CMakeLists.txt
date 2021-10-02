INCLUDE_DIRECTORIES(.)

SET(LIDAR_SRC hchead.cpp hclidar.cpp lidar.cpp ReadParsePackage.cpp HC_serial.cpp HcSDK.cpp)
ADD_LIBRARY(lidar ${LIDAR_SRC})
IF (WIN32)
ELSE()
    TARGET_LINK_LIBRARIES(lidar pthread)
ENDIF()
