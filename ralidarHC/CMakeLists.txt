cmake_minimum_required(VERSION 3.0.2)
project(ralidarHC)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(RPLIDAR_SDK_PATH "./sdk/")

FILE(GLOB RPLIDAR_SDK_SRC 
  "${RPLIDAR_SDK_PATH}/src/base/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/*.cpp"
)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rosconsole
  roscpp
  sensor_msgs
  std_srvs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
set(RPLIDAR_SDK_PATH "./sdk/")
include_directories(
  ${RPLIDAR_SDK_PATH}/include
  ${RPLIDAR_SDK_PATH}/src
  ${RPLIDAR_SDK_PATH}/src/base
  ${catkin_INCLUDE_DIRS}
)
link_directories(${RPLIDAR_SDK_PATH}/lib/) # .a文件的路径

catkin_package()


#add_executable(HCrplidarNode src/node.cpp src/sdk_V3_use.cpp ${RPLIDAR_SDK_SRC})
add_executable(HCrplidarNode src/node.cpp src/sdk_V3_use.cpp ${RPLIDAR_SDK_SRC})

target_link_libraries(HCrplidarNode ${catkin_LIBRARIES} lidar )


install(TARGETS HCrplidarNode
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch rviz sdk
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

ADD_SUBDIRECTORY( ${RPLIDAR_SDK_PATH}/src) 

