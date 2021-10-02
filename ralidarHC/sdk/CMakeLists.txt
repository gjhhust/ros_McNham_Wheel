project(CAMSENSE_SDK)
cmake_minimum_required (VERSION 2.8)
add_definitions(-std=c++11) # Use C++11


include_directories(
    ${PROJECT_SOURCE_DIR}/dep
    )

LINK_DIRECTORIES(
	#${PROJECT_SOURCE_DIR}/dep/jsoncpp/lib
    )


#SET(CMAKE_CONFIGURATION_TYPES Release CACHE STRING "" FORCE)
SET(EXECUTABLE_OUTPUT_PATH ${CAMSENSE_SDK_SOURCE_DIR}/bin/lib CACHE PATH "Build directory" FORCE)
SET(LIBRARY_OUTPUT_PATH ${CAMSENSE_SDK_SOURCE_DIR}/bin/lib CACHE PATH "Build directory" FORCE)

add_subdirectory(src)
