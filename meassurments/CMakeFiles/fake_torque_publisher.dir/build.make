# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.15

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
CMAKE_COMMAND = /home/peter/Documents/clion-2019.3.3/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/peter/Documents/clion-2019.3.3/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/peter/catkin_devel/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/peter/catkin_devel/src

# Include any dependencies generated for this target.
include meassurments/CMakeFiles/fake_torque_publisher.dir/depend.make

# Include the progress variables for this target.
include meassurments/CMakeFiles/fake_torque_publisher.dir/progress.make

# Include the compile flags for this target's objects.
include meassurments/CMakeFiles/fake_torque_publisher.dir/flags.make

meassurments/CMakeFiles/fake_torque_publisher.dir/src/fake_torque_publisher.cpp.o: meassurments/CMakeFiles/fake_torque_publisher.dir/flags.make
meassurments/CMakeFiles/fake_torque_publisher.dir/src/fake_torque_publisher.cpp.o: meassurments/src/fake_torque_publisher.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/peter/catkin_devel/src/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object meassurments/CMakeFiles/fake_torque_publisher.dir/src/fake_torque_publisher.cpp.o"
	cd /home/peter/catkin_devel/src/meassurments && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/fake_torque_publisher.dir/src/fake_torque_publisher.cpp.o -c /home/peter/catkin_devel/src/meassurments/src/fake_torque_publisher.cpp

meassurments/CMakeFiles/fake_torque_publisher.dir/src/fake_torque_publisher.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/fake_torque_publisher.dir/src/fake_torque_publisher.cpp.i"
	cd /home/peter/catkin_devel/src/meassurments && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/peter/catkin_devel/src/meassurments/src/fake_torque_publisher.cpp > CMakeFiles/fake_torque_publisher.dir/src/fake_torque_publisher.cpp.i

meassurments/CMakeFiles/fake_torque_publisher.dir/src/fake_torque_publisher.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/fake_torque_publisher.dir/src/fake_torque_publisher.cpp.s"
	cd /home/peter/catkin_devel/src/meassurments && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/peter/catkin_devel/src/meassurments/src/fake_torque_publisher.cpp -o CMakeFiles/fake_torque_publisher.dir/src/fake_torque_publisher.cpp.s

meassurments/CMakeFiles/fake_torque_publisher.dir/src/test_Eth.cpp.o: meassurments/CMakeFiles/fake_torque_publisher.dir/flags.make
meassurments/CMakeFiles/fake_torque_publisher.dir/src/test_Eth.cpp.o: meassurments/src/test_Eth.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/peter/catkin_devel/src/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object meassurments/CMakeFiles/fake_torque_publisher.dir/src/test_Eth.cpp.o"
	cd /home/peter/catkin_devel/src/meassurments && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/fake_torque_publisher.dir/src/test_Eth.cpp.o -c /home/peter/catkin_devel/src/meassurments/src/test_Eth.cpp

meassurments/CMakeFiles/fake_torque_publisher.dir/src/test_Eth.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/fake_torque_publisher.dir/src/test_Eth.cpp.i"
	cd /home/peter/catkin_devel/src/meassurments && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/peter/catkin_devel/src/meassurments/src/test_Eth.cpp > CMakeFiles/fake_torque_publisher.dir/src/test_Eth.cpp.i

meassurments/CMakeFiles/fake_torque_publisher.dir/src/test_Eth.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/fake_torque_publisher.dir/src/test_Eth.cpp.s"
	cd /home/peter/catkin_devel/src/meassurments && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/peter/catkin_devel/src/meassurments/src/test_Eth.cpp -o CMakeFiles/fake_torque_publisher.dir/src/test_Eth.cpp.s

# Object files for target fake_torque_publisher
fake_torque_publisher_OBJECTS = \
"CMakeFiles/fake_torque_publisher.dir/src/fake_torque_publisher.cpp.o" \
"CMakeFiles/fake_torque_publisher.dir/src/test_Eth.cpp.o"

# External object files for target fake_torque_publisher
fake_torque_publisher_EXTERNAL_OBJECTS =

/home/peter/catkin_devel/devel/lib/meassurments/fake_torque_publisher: meassurments/CMakeFiles/fake_torque_publisher.dir/src/fake_torque_publisher.cpp.o
/home/peter/catkin_devel/devel/lib/meassurments/fake_torque_publisher: meassurments/CMakeFiles/fake_torque_publisher.dir/src/test_Eth.cpp.o
/home/peter/catkin_devel/devel/lib/meassurments/fake_torque_publisher: meassurments/CMakeFiles/fake_torque_publisher.dir/build.make
/home/peter/catkin_devel/devel/lib/meassurments/fake_torque_publisher: /opt/ros/melodic/lib/libroscpp.so
/home/peter/catkin_devel/devel/lib/meassurments/fake_torque_publisher: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/peter/catkin_devel/devel/lib/meassurments/fake_torque_publisher: /opt/ros/melodic/lib/librosconsole.so
/home/peter/catkin_devel/devel/lib/meassurments/fake_torque_publisher: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/peter/catkin_devel/devel/lib/meassurments/fake_torque_publisher: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/peter/catkin_devel/devel/lib/meassurments/fake_torque_publisher: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/peter/catkin_devel/devel/lib/meassurments/fake_torque_publisher: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/peter/catkin_devel/devel/lib/meassurments/fake_torque_publisher: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/peter/catkin_devel/devel/lib/meassurments/fake_torque_publisher: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/peter/catkin_devel/devel/lib/meassurments/fake_torque_publisher: /opt/ros/melodic/lib/librostime.so
/home/peter/catkin_devel/devel/lib/meassurments/fake_torque_publisher: /opt/ros/melodic/lib/libcpp_common.so
/home/peter/catkin_devel/devel/lib/meassurments/fake_torque_publisher: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/peter/catkin_devel/devel/lib/meassurments/fake_torque_publisher: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/peter/catkin_devel/devel/lib/meassurments/fake_torque_publisher: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/peter/catkin_devel/devel/lib/meassurments/fake_torque_publisher: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/peter/catkin_devel/devel/lib/meassurments/fake_torque_publisher: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/peter/catkin_devel/devel/lib/meassurments/fake_torque_publisher: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/peter/catkin_devel/devel/lib/meassurments/fake_torque_publisher: /opt/ros/melodic/lib/libroslib.so
/home/peter/catkin_devel/devel/lib/meassurments/fake_torque_publisher: /opt/ros/melodic/lib/librospack.so
/home/peter/catkin_devel/devel/lib/meassurments/fake_torque_publisher: /usr/lib/x86_64-linux-gnu/libpython2.7.so
/home/peter/catkin_devel/devel/lib/meassurments/fake_torque_publisher: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/peter/catkin_devel/devel/lib/meassurments/fake_torque_publisher: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
/home/peter/catkin_devel/devel/lib/meassurments/fake_torque_publisher: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/peter/catkin_devel/devel/lib/meassurments/fake_torque_publisher: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
/home/peter/catkin_devel/devel/lib/meassurments/fake_torque_publisher: meassurments/CMakeFiles/fake_torque_publisher.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/peter/catkin_devel/src/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable /home/peter/catkin_devel/devel/lib/meassurments/fake_torque_publisher"
	cd /home/peter/catkin_devel/src/meassurments && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/fake_torque_publisher.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
meassurments/CMakeFiles/fake_torque_publisher.dir/build: /home/peter/catkin_devel/devel/lib/meassurments/fake_torque_publisher

.PHONY : meassurments/CMakeFiles/fake_torque_publisher.dir/build

meassurments/CMakeFiles/fake_torque_publisher.dir/clean:
	cd /home/peter/catkin_devel/src/meassurments && $(CMAKE_COMMAND) -P CMakeFiles/fake_torque_publisher.dir/cmake_clean.cmake
.PHONY : meassurments/CMakeFiles/fake_torque_publisher.dir/clean

meassurments/CMakeFiles/fake_torque_publisher.dir/depend:
	cd /home/peter/catkin_devel/src && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/peter/catkin_devel/src /home/peter/catkin_devel/src/meassurments /home/peter/catkin_devel/src /home/peter/catkin_devel/src/meassurments /home/peter/catkin_devel/src/meassurments/CMakeFiles/fake_torque_publisher.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : meassurments/CMakeFiles/fake_torque_publisher.dir/depend

