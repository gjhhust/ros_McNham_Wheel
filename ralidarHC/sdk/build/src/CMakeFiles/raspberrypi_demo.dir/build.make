# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/gjhhust/ROS/user/test/src/ralidarHC/sdk

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/gjhhust/ROS/user/test/src/ralidarHC/sdk/build

# Include any dependencies generated for this target.
include src/CMakeFiles/raspberrypi_demo.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/raspberrypi_demo.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/raspberrypi_demo.dir/flags.make

src/CMakeFiles/raspberrypi_demo.dir/raspberrypi_demo.cpp.o: src/CMakeFiles/raspberrypi_demo.dir/flags.make
src/CMakeFiles/raspberrypi_demo.dir/raspberrypi_demo.cpp.o: ../src/raspberrypi_demo.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/gjhhust/ROS/user/test/src/ralidarHC/sdk/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/raspberrypi_demo.dir/raspberrypi_demo.cpp.o"
	cd /home/gjhhust/ROS/user/test/src/ralidarHC/sdk/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/raspberrypi_demo.dir/raspberrypi_demo.cpp.o -c /home/gjhhust/ROS/user/test/src/ralidarHC/sdk/src/raspberrypi_demo.cpp

src/CMakeFiles/raspberrypi_demo.dir/raspberrypi_demo.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/raspberrypi_demo.dir/raspberrypi_demo.cpp.i"
	cd /home/gjhhust/ROS/user/test/src/ralidarHC/sdk/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/gjhhust/ROS/user/test/src/ralidarHC/sdk/src/raspberrypi_demo.cpp > CMakeFiles/raspberrypi_demo.dir/raspberrypi_demo.cpp.i

src/CMakeFiles/raspberrypi_demo.dir/raspberrypi_demo.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/raspberrypi_demo.dir/raspberrypi_demo.cpp.s"
	cd /home/gjhhust/ROS/user/test/src/ralidarHC/sdk/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/gjhhust/ROS/user/test/src/ralidarHC/sdk/src/raspberrypi_demo.cpp -o CMakeFiles/raspberrypi_demo.dir/raspberrypi_demo.cpp.s

src/CMakeFiles/raspberrypi_demo.dir/raspberrypi_demo.cpp.o.requires:

.PHONY : src/CMakeFiles/raspberrypi_demo.dir/raspberrypi_demo.cpp.o.requires

src/CMakeFiles/raspberrypi_demo.dir/raspberrypi_demo.cpp.o.provides: src/CMakeFiles/raspberrypi_demo.dir/raspberrypi_demo.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/raspberrypi_demo.dir/build.make src/CMakeFiles/raspberrypi_demo.dir/raspberrypi_demo.cpp.o.provides.build
.PHONY : src/CMakeFiles/raspberrypi_demo.dir/raspberrypi_demo.cpp.o.provides

src/CMakeFiles/raspberrypi_demo.dir/raspberrypi_demo.cpp.o.provides.build: src/CMakeFiles/raspberrypi_demo.dir/raspberrypi_demo.cpp.o


src/CMakeFiles/raspberrypi_demo.dir/LidarTest.cpp.o: src/CMakeFiles/raspberrypi_demo.dir/flags.make
src/CMakeFiles/raspberrypi_demo.dir/LidarTest.cpp.o: ../src/LidarTest.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/gjhhust/ROS/user/test/src/ralidarHC/sdk/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/CMakeFiles/raspberrypi_demo.dir/LidarTest.cpp.o"
	cd /home/gjhhust/ROS/user/test/src/ralidarHC/sdk/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/raspberrypi_demo.dir/LidarTest.cpp.o -c /home/gjhhust/ROS/user/test/src/ralidarHC/sdk/src/LidarTest.cpp

src/CMakeFiles/raspberrypi_demo.dir/LidarTest.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/raspberrypi_demo.dir/LidarTest.cpp.i"
	cd /home/gjhhust/ROS/user/test/src/ralidarHC/sdk/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/gjhhust/ROS/user/test/src/ralidarHC/sdk/src/LidarTest.cpp > CMakeFiles/raspberrypi_demo.dir/LidarTest.cpp.i

src/CMakeFiles/raspberrypi_demo.dir/LidarTest.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/raspberrypi_demo.dir/LidarTest.cpp.s"
	cd /home/gjhhust/ROS/user/test/src/ralidarHC/sdk/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/gjhhust/ROS/user/test/src/ralidarHC/sdk/src/LidarTest.cpp -o CMakeFiles/raspberrypi_demo.dir/LidarTest.cpp.s

src/CMakeFiles/raspberrypi_demo.dir/LidarTest.cpp.o.requires:

.PHONY : src/CMakeFiles/raspberrypi_demo.dir/LidarTest.cpp.o.requires

src/CMakeFiles/raspberrypi_demo.dir/LidarTest.cpp.o.provides: src/CMakeFiles/raspberrypi_demo.dir/LidarTest.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/raspberrypi_demo.dir/build.make src/CMakeFiles/raspberrypi_demo.dir/LidarTest.cpp.o.provides.build
.PHONY : src/CMakeFiles/raspberrypi_demo.dir/LidarTest.cpp.o.provides

src/CMakeFiles/raspberrypi_demo.dir/LidarTest.cpp.o.provides.build: src/CMakeFiles/raspberrypi_demo.dir/LidarTest.cpp.o


# Object files for target raspberrypi_demo
raspberrypi_demo_OBJECTS = \
"CMakeFiles/raspberrypi_demo.dir/raspberrypi_demo.cpp.o" \
"CMakeFiles/raspberrypi_demo.dir/LidarTest.cpp.o"

# External object files for target raspberrypi_demo
raspberrypi_demo_EXTERNAL_OBJECTS =

../bin/lib/raspberrypi_demo: src/CMakeFiles/raspberrypi_demo.dir/raspberrypi_demo.cpp.o
../bin/lib/raspberrypi_demo: src/CMakeFiles/raspberrypi_demo.dir/LidarTest.cpp.o
../bin/lib/raspberrypi_demo: src/CMakeFiles/raspberrypi_demo.dir/build.make
../bin/lib/raspberrypi_demo: ../bin/lib/liblidar.a
../bin/lib/raspberrypi_demo: src/CMakeFiles/raspberrypi_demo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/gjhhust/ROS/user/test/src/ralidarHC/sdk/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable ../../bin/lib/raspberrypi_demo"
	cd /home/gjhhust/ROS/user/test/src/ralidarHC/sdk/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/raspberrypi_demo.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/raspberrypi_demo.dir/build: ../bin/lib/raspberrypi_demo

.PHONY : src/CMakeFiles/raspberrypi_demo.dir/build

src/CMakeFiles/raspberrypi_demo.dir/requires: src/CMakeFiles/raspberrypi_demo.dir/raspberrypi_demo.cpp.o.requires
src/CMakeFiles/raspberrypi_demo.dir/requires: src/CMakeFiles/raspberrypi_demo.dir/LidarTest.cpp.o.requires

.PHONY : src/CMakeFiles/raspberrypi_demo.dir/requires

src/CMakeFiles/raspberrypi_demo.dir/clean:
	cd /home/gjhhust/ROS/user/test/src/ralidarHC/sdk/build/src && $(CMAKE_COMMAND) -P CMakeFiles/raspberrypi_demo.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/raspberrypi_demo.dir/clean

src/CMakeFiles/raspberrypi_demo.dir/depend:
	cd /home/gjhhust/ROS/user/test/src/ralidarHC/sdk/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/gjhhust/ROS/user/test/src/ralidarHC/sdk /home/gjhhust/ROS/user/test/src/ralidarHC/sdk/src /home/gjhhust/ROS/user/test/src/ralidarHC/sdk/build /home/gjhhust/ROS/user/test/src/ralidarHC/sdk/build/src /home/gjhhust/ROS/user/test/src/ralidarHC/sdk/build/src/CMakeFiles/raspberrypi_demo.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/raspberrypi_demo.dir/depend

