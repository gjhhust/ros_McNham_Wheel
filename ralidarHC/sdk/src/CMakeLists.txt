INCLUDE_DIRECTORIES(${CAMSENSE_SOURCE_DIR}/src/base)
LINK_DIRECTORIES(${CAMSENSE_SOURCE_DIR}/src/base)


INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/dep
    ${PROJECT_SOURCE_DIR}/dep/eigen3
	)
	
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/lib)
LINK_DIRECTORIES(${PROJECT_SOURCE_DIR}/lib)

	
ADD_SUBDIRECTORY(base)



#ADD_EXECUTABLE (BS_Demo BS_Demo.cpp LidarTest.cpp)
#TARGET_LINK_LIBRARIES (BS_Demo lidar)

#if(UNIX)

#if(CMAKE_TOOLCHAIN_FILE)
#TARGET_LINK_LIBRARIES (BS_Demo lidar libChargingPoint_arm.a)
#else()
#TARGET_LINK_LIBRARIES (BS_Demo lidar libChargingPoint_x64.a)
#endif()

#endif(UNIX)


#if (MSVC)
#    target_link_libraries(BS_Demo lidar ChargingPoint)
#endif (MSVC)