# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.26

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /opt/anaconda3/lib/python3.10/site-packages/cmake/data/bin/cmake

# The command to remove a file.
RM = /opt/anaconda3/lib/python3.10/site-packages/cmake/data/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/amax/aubo_sdk_test/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/amax/aubo_sdk_test/build

# Include any dependencies generated for this target.
include robot_sdk/CMakeFiles/main.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include robot_sdk/CMakeFiles/main.dir/compiler_depend.make

# Include the progress variables for this target.
include robot_sdk/CMakeFiles/main.dir/progress.make

# Include the compile flags for this target's objects.
include robot_sdk/CMakeFiles/main.dir/flags.make

robot_sdk/CMakeFiles/main.dir/src/main.cpp.o: robot_sdk/CMakeFiles/main.dir/flags.make
robot_sdk/CMakeFiles/main.dir/src/main.cpp.o: /home/amax/aubo_sdk_test/src/robot_sdk/src/main.cpp
robot_sdk/CMakeFiles/main.dir/src/main.cpp.o: robot_sdk/CMakeFiles/main.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/amax/aubo_sdk_test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object robot_sdk/CMakeFiles/main.dir/src/main.cpp.o"
	cd /home/amax/aubo_sdk_test/build/robot_sdk && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT robot_sdk/CMakeFiles/main.dir/src/main.cpp.o -MF CMakeFiles/main.dir/src/main.cpp.o.d -o CMakeFiles/main.dir/src/main.cpp.o -c /home/amax/aubo_sdk_test/src/robot_sdk/src/main.cpp

robot_sdk/CMakeFiles/main.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/src/main.cpp.i"
	cd /home/amax/aubo_sdk_test/build/robot_sdk && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/amax/aubo_sdk_test/src/robot_sdk/src/main.cpp > CMakeFiles/main.dir/src/main.cpp.i

robot_sdk/CMakeFiles/main.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/src/main.cpp.s"
	cd /home/amax/aubo_sdk_test/build/robot_sdk && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/amax/aubo_sdk_test/src/robot_sdk/src/main.cpp -o CMakeFiles/main.dir/src/main.cpp.s

robot_sdk/CMakeFiles/main.dir/src/example/AuboSdkExample.cpp.o: robot_sdk/CMakeFiles/main.dir/flags.make
robot_sdk/CMakeFiles/main.dir/src/example/AuboSdkExample.cpp.o: /home/amax/aubo_sdk_test/src/robot_sdk/src/example/AuboSdkExample.cpp
robot_sdk/CMakeFiles/main.dir/src/example/AuboSdkExample.cpp.o: robot_sdk/CMakeFiles/main.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/amax/aubo_sdk_test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object robot_sdk/CMakeFiles/main.dir/src/example/AuboSdkExample.cpp.o"
	cd /home/amax/aubo_sdk_test/build/robot_sdk && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT robot_sdk/CMakeFiles/main.dir/src/example/AuboSdkExample.cpp.o -MF CMakeFiles/main.dir/src/example/AuboSdkExample.cpp.o.d -o CMakeFiles/main.dir/src/example/AuboSdkExample.cpp.o -c /home/amax/aubo_sdk_test/src/robot_sdk/src/example/AuboSdkExample.cpp

robot_sdk/CMakeFiles/main.dir/src/example/AuboSdkExample.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/src/example/AuboSdkExample.cpp.i"
	cd /home/amax/aubo_sdk_test/build/robot_sdk && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/amax/aubo_sdk_test/src/robot_sdk/src/example/AuboSdkExample.cpp > CMakeFiles/main.dir/src/example/AuboSdkExample.cpp.i

robot_sdk/CMakeFiles/main.dir/src/example/AuboSdkExample.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/src/example/AuboSdkExample.cpp.s"
	cd /home/amax/aubo_sdk_test/build/robot_sdk && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/amax/aubo_sdk_test/src/robot_sdk/src/example/AuboSdkExample.cpp -o CMakeFiles/main.dir/src/example/AuboSdkExample.cpp.s

robot_sdk/CMakeFiles/main.dir/src/example/example_0.cpp.o: robot_sdk/CMakeFiles/main.dir/flags.make
robot_sdk/CMakeFiles/main.dir/src/example/example_0.cpp.o: /home/amax/aubo_sdk_test/src/robot_sdk/src/example/example_0.cpp
robot_sdk/CMakeFiles/main.dir/src/example/example_0.cpp.o: robot_sdk/CMakeFiles/main.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/amax/aubo_sdk_test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object robot_sdk/CMakeFiles/main.dir/src/example/example_0.cpp.o"
	cd /home/amax/aubo_sdk_test/build/robot_sdk && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT robot_sdk/CMakeFiles/main.dir/src/example/example_0.cpp.o -MF CMakeFiles/main.dir/src/example/example_0.cpp.o.d -o CMakeFiles/main.dir/src/example/example_0.cpp.o -c /home/amax/aubo_sdk_test/src/robot_sdk/src/example/example_0.cpp

robot_sdk/CMakeFiles/main.dir/src/example/example_0.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/src/example/example_0.cpp.i"
	cd /home/amax/aubo_sdk_test/build/robot_sdk && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/amax/aubo_sdk_test/src/robot_sdk/src/example/example_0.cpp > CMakeFiles/main.dir/src/example/example_0.cpp.i

robot_sdk/CMakeFiles/main.dir/src/example/example_0.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/src/example/example_0.cpp.s"
	cd /home/amax/aubo_sdk_test/build/robot_sdk && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/amax/aubo_sdk_test/src/robot_sdk/src/example/example_0.cpp -o CMakeFiles/main.dir/src/example/example_0.cpp.s

robot_sdk/CMakeFiles/main.dir/src/example/example_1.cpp.o: robot_sdk/CMakeFiles/main.dir/flags.make
robot_sdk/CMakeFiles/main.dir/src/example/example_1.cpp.o: /home/amax/aubo_sdk_test/src/robot_sdk/src/example/example_1.cpp
robot_sdk/CMakeFiles/main.dir/src/example/example_1.cpp.o: robot_sdk/CMakeFiles/main.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/amax/aubo_sdk_test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object robot_sdk/CMakeFiles/main.dir/src/example/example_1.cpp.o"
	cd /home/amax/aubo_sdk_test/build/robot_sdk && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT robot_sdk/CMakeFiles/main.dir/src/example/example_1.cpp.o -MF CMakeFiles/main.dir/src/example/example_1.cpp.o.d -o CMakeFiles/main.dir/src/example/example_1.cpp.o -c /home/amax/aubo_sdk_test/src/robot_sdk/src/example/example_1.cpp

robot_sdk/CMakeFiles/main.dir/src/example/example_1.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/src/example/example_1.cpp.i"
	cd /home/amax/aubo_sdk_test/build/robot_sdk && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/amax/aubo_sdk_test/src/robot_sdk/src/example/example_1.cpp > CMakeFiles/main.dir/src/example/example_1.cpp.i

robot_sdk/CMakeFiles/main.dir/src/example/example_1.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/src/example/example_1.cpp.s"
	cd /home/amax/aubo_sdk_test/build/robot_sdk && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/amax/aubo_sdk_test/src/robot_sdk/src/example/example_1.cpp -o CMakeFiles/main.dir/src/example/example_1.cpp.s

robot_sdk/CMakeFiles/main.dir/src/example/example_3.cpp.o: robot_sdk/CMakeFiles/main.dir/flags.make
robot_sdk/CMakeFiles/main.dir/src/example/example_3.cpp.o: /home/amax/aubo_sdk_test/src/robot_sdk/src/example/example_3.cpp
robot_sdk/CMakeFiles/main.dir/src/example/example_3.cpp.o: robot_sdk/CMakeFiles/main.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/amax/aubo_sdk_test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object robot_sdk/CMakeFiles/main.dir/src/example/example_3.cpp.o"
	cd /home/amax/aubo_sdk_test/build/robot_sdk && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT robot_sdk/CMakeFiles/main.dir/src/example/example_3.cpp.o -MF CMakeFiles/main.dir/src/example/example_3.cpp.o.d -o CMakeFiles/main.dir/src/example/example_3.cpp.o -c /home/amax/aubo_sdk_test/src/robot_sdk/src/example/example_3.cpp

robot_sdk/CMakeFiles/main.dir/src/example/example_3.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/src/example/example_3.cpp.i"
	cd /home/amax/aubo_sdk_test/build/robot_sdk && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/amax/aubo_sdk_test/src/robot_sdk/src/example/example_3.cpp > CMakeFiles/main.dir/src/example/example_3.cpp.i

robot_sdk/CMakeFiles/main.dir/src/example/example_3.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/src/example/example_3.cpp.s"
	cd /home/amax/aubo_sdk_test/build/robot_sdk && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/amax/aubo_sdk_test/src/robot_sdk/src/example/example_3.cpp -o CMakeFiles/main.dir/src/example/example_3.cpp.s

robot_sdk/CMakeFiles/main.dir/src/example/example_4.cpp.o: robot_sdk/CMakeFiles/main.dir/flags.make
robot_sdk/CMakeFiles/main.dir/src/example/example_4.cpp.o: /home/amax/aubo_sdk_test/src/robot_sdk/src/example/example_4.cpp
robot_sdk/CMakeFiles/main.dir/src/example/example_4.cpp.o: robot_sdk/CMakeFiles/main.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/amax/aubo_sdk_test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object robot_sdk/CMakeFiles/main.dir/src/example/example_4.cpp.o"
	cd /home/amax/aubo_sdk_test/build/robot_sdk && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT robot_sdk/CMakeFiles/main.dir/src/example/example_4.cpp.o -MF CMakeFiles/main.dir/src/example/example_4.cpp.o.d -o CMakeFiles/main.dir/src/example/example_4.cpp.o -c /home/amax/aubo_sdk_test/src/robot_sdk/src/example/example_4.cpp

robot_sdk/CMakeFiles/main.dir/src/example/example_4.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/src/example/example_4.cpp.i"
	cd /home/amax/aubo_sdk_test/build/robot_sdk && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/amax/aubo_sdk_test/src/robot_sdk/src/example/example_4.cpp > CMakeFiles/main.dir/src/example/example_4.cpp.i

robot_sdk/CMakeFiles/main.dir/src/example/example_4.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/src/example/example_4.cpp.s"
	cd /home/amax/aubo_sdk_test/build/robot_sdk && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/amax/aubo_sdk_test/src/robot_sdk/src/example/example_4.cpp -o CMakeFiles/main.dir/src/example/example_4.cpp.s

robot_sdk/CMakeFiles/main.dir/src/example/example_5.cpp.o: robot_sdk/CMakeFiles/main.dir/flags.make
robot_sdk/CMakeFiles/main.dir/src/example/example_5.cpp.o: /home/amax/aubo_sdk_test/src/robot_sdk/src/example/example_5.cpp
robot_sdk/CMakeFiles/main.dir/src/example/example_5.cpp.o: robot_sdk/CMakeFiles/main.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/amax/aubo_sdk_test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object robot_sdk/CMakeFiles/main.dir/src/example/example_5.cpp.o"
	cd /home/amax/aubo_sdk_test/build/robot_sdk && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT robot_sdk/CMakeFiles/main.dir/src/example/example_5.cpp.o -MF CMakeFiles/main.dir/src/example/example_5.cpp.o.d -o CMakeFiles/main.dir/src/example/example_5.cpp.o -c /home/amax/aubo_sdk_test/src/robot_sdk/src/example/example_5.cpp

robot_sdk/CMakeFiles/main.dir/src/example/example_5.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/src/example/example_5.cpp.i"
	cd /home/amax/aubo_sdk_test/build/robot_sdk && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/amax/aubo_sdk_test/src/robot_sdk/src/example/example_5.cpp > CMakeFiles/main.dir/src/example/example_5.cpp.i

robot_sdk/CMakeFiles/main.dir/src/example/example_5.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/src/example/example_5.cpp.s"
	cd /home/amax/aubo_sdk_test/build/robot_sdk && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/amax/aubo_sdk_test/src/robot_sdk/src/example/example_5.cpp -o CMakeFiles/main.dir/src/example/example_5.cpp.s

robot_sdk/CMakeFiles/main.dir/src/example/example_6.cpp.o: robot_sdk/CMakeFiles/main.dir/flags.make
robot_sdk/CMakeFiles/main.dir/src/example/example_6.cpp.o: /home/amax/aubo_sdk_test/src/robot_sdk/src/example/example_6.cpp
robot_sdk/CMakeFiles/main.dir/src/example/example_6.cpp.o: robot_sdk/CMakeFiles/main.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/amax/aubo_sdk_test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object robot_sdk/CMakeFiles/main.dir/src/example/example_6.cpp.o"
	cd /home/amax/aubo_sdk_test/build/robot_sdk && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT robot_sdk/CMakeFiles/main.dir/src/example/example_6.cpp.o -MF CMakeFiles/main.dir/src/example/example_6.cpp.o.d -o CMakeFiles/main.dir/src/example/example_6.cpp.o -c /home/amax/aubo_sdk_test/src/robot_sdk/src/example/example_6.cpp

robot_sdk/CMakeFiles/main.dir/src/example/example_6.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/src/example/example_6.cpp.i"
	cd /home/amax/aubo_sdk_test/build/robot_sdk && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/amax/aubo_sdk_test/src/robot_sdk/src/example/example_6.cpp > CMakeFiles/main.dir/src/example/example_6.cpp.i

robot_sdk/CMakeFiles/main.dir/src/example/example_6.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/src/example/example_6.cpp.s"
	cd /home/amax/aubo_sdk_test/build/robot_sdk && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/amax/aubo_sdk_test/src/robot_sdk/src/example/example_6.cpp -o CMakeFiles/main.dir/src/example/example_6.cpp.s

robot_sdk/CMakeFiles/main.dir/src/example/example_8.cpp.o: robot_sdk/CMakeFiles/main.dir/flags.make
robot_sdk/CMakeFiles/main.dir/src/example/example_8.cpp.o: /home/amax/aubo_sdk_test/src/robot_sdk/src/example/example_8.cpp
robot_sdk/CMakeFiles/main.dir/src/example/example_8.cpp.o: robot_sdk/CMakeFiles/main.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/amax/aubo_sdk_test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object robot_sdk/CMakeFiles/main.dir/src/example/example_8.cpp.o"
	cd /home/amax/aubo_sdk_test/build/robot_sdk && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT robot_sdk/CMakeFiles/main.dir/src/example/example_8.cpp.o -MF CMakeFiles/main.dir/src/example/example_8.cpp.o.d -o CMakeFiles/main.dir/src/example/example_8.cpp.o -c /home/amax/aubo_sdk_test/src/robot_sdk/src/example/example_8.cpp

robot_sdk/CMakeFiles/main.dir/src/example/example_8.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/src/example/example_8.cpp.i"
	cd /home/amax/aubo_sdk_test/build/robot_sdk && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/amax/aubo_sdk_test/src/robot_sdk/src/example/example_8.cpp > CMakeFiles/main.dir/src/example/example_8.cpp.i

robot_sdk/CMakeFiles/main.dir/src/example/example_8.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/src/example/example_8.cpp.s"
	cd /home/amax/aubo_sdk_test/build/robot_sdk && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/amax/aubo_sdk_test/src/robot_sdk/src/example/example_8.cpp -o CMakeFiles/main.dir/src/example/example_8.cpp.s

robot_sdk/CMakeFiles/main.dir/src/example/example_9.cpp.o: robot_sdk/CMakeFiles/main.dir/flags.make
robot_sdk/CMakeFiles/main.dir/src/example/example_9.cpp.o: /home/amax/aubo_sdk_test/src/robot_sdk/src/example/example_9.cpp
robot_sdk/CMakeFiles/main.dir/src/example/example_9.cpp.o: robot_sdk/CMakeFiles/main.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/amax/aubo_sdk_test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object robot_sdk/CMakeFiles/main.dir/src/example/example_9.cpp.o"
	cd /home/amax/aubo_sdk_test/build/robot_sdk && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT robot_sdk/CMakeFiles/main.dir/src/example/example_9.cpp.o -MF CMakeFiles/main.dir/src/example/example_9.cpp.o.d -o CMakeFiles/main.dir/src/example/example_9.cpp.o -c /home/amax/aubo_sdk_test/src/robot_sdk/src/example/example_9.cpp

robot_sdk/CMakeFiles/main.dir/src/example/example_9.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/src/example/example_9.cpp.i"
	cd /home/amax/aubo_sdk_test/build/robot_sdk && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/amax/aubo_sdk_test/src/robot_sdk/src/example/example_9.cpp > CMakeFiles/main.dir/src/example/example_9.cpp.i

robot_sdk/CMakeFiles/main.dir/src/example/example_9.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/src/example/example_9.cpp.s"
	cd /home/amax/aubo_sdk_test/build/robot_sdk && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/amax/aubo_sdk_test/src/robot_sdk/src/example/example_9.cpp -o CMakeFiles/main.dir/src/example/example_9.cpp.s

robot_sdk/CMakeFiles/main.dir/src/example/example_toolio.cpp.o: robot_sdk/CMakeFiles/main.dir/flags.make
robot_sdk/CMakeFiles/main.dir/src/example/example_toolio.cpp.o: /home/amax/aubo_sdk_test/src/robot_sdk/src/example/example_toolio.cpp
robot_sdk/CMakeFiles/main.dir/src/example/example_toolio.cpp.o: robot_sdk/CMakeFiles/main.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/amax/aubo_sdk_test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building CXX object robot_sdk/CMakeFiles/main.dir/src/example/example_toolio.cpp.o"
	cd /home/amax/aubo_sdk_test/build/robot_sdk && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT robot_sdk/CMakeFiles/main.dir/src/example/example_toolio.cpp.o -MF CMakeFiles/main.dir/src/example/example_toolio.cpp.o.d -o CMakeFiles/main.dir/src/example/example_toolio.cpp.o -c /home/amax/aubo_sdk_test/src/robot_sdk/src/example/example_toolio.cpp

robot_sdk/CMakeFiles/main.dir/src/example/example_toolio.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/src/example/example_toolio.cpp.i"
	cd /home/amax/aubo_sdk_test/build/robot_sdk && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/amax/aubo_sdk_test/src/robot_sdk/src/example/example_toolio.cpp > CMakeFiles/main.dir/src/example/example_toolio.cpp.i

robot_sdk/CMakeFiles/main.dir/src/example/example_toolio.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/src/example/example_toolio.cpp.s"
	cd /home/amax/aubo_sdk_test/build/robot_sdk && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/amax/aubo_sdk_test/src/robot_sdk/src/example/example_toolio.cpp -o CMakeFiles/main.dir/src/example/example_toolio.cpp.s

robot_sdk/CMakeFiles/main.dir/src/example/util.cpp.o: robot_sdk/CMakeFiles/main.dir/flags.make
robot_sdk/CMakeFiles/main.dir/src/example/util.cpp.o: /home/amax/aubo_sdk_test/src/robot_sdk/src/example/util.cpp
robot_sdk/CMakeFiles/main.dir/src/example/util.cpp.o: robot_sdk/CMakeFiles/main.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/amax/aubo_sdk_test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Building CXX object robot_sdk/CMakeFiles/main.dir/src/example/util.cpp.o"
	cd /home/amax/aubo_sdk_test/build/robot_sdk && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT robot_sdk/CMakeFiles/main.dir/src/example/util.cpp.o -MF CMakeFiles/main.dir/src/example/util.cpp.o.d -o CMakeFiles/main.dir/src/example/util.cpp.o -c /home/amax/aubo_sdk_test/src/robot_sdk/src/example/util.cpp

robot_sdk/CMakeFiles/main.dir/src/example/util.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/src/example/util.cpp.i"
	cd /home/amax/aubo_sdk_test/build/robot_sdk && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/amax/aubo_sdk_test/src/robot_sdk/src/example/util.cpp > CMakeFiles/main.dir/src/example/util.cpp.i

robot_sdk/CMakeFiles/main.dir/src/example/util.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/src/example/util.cpp.s"
	cd /home/amax/aubo_sdk_test/build/robot_sdk && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/amax/aubo_sdk_test/src/robot_sdk/src/example/util.cpp -o CMakeFiles/main.dir/src/example/util.cpp.s

# Object files for target main
main_OBJECTS = \
"CMakeFiles/main.dir/src/main.cpp.o" \
"CMakeFiles/main.dir/src/example/AuboSdkExample.cpp.o" \
"CMakeFiles/main.dir/src/example/example_0.cpp.o" \
"CMakeFiles/main.dir/src/example/example_1.cpp.o" \
"CMakeFiles/main.dir/src/example/example_3.cpp.o" \
"CMakeFiles/main.dir/src/example/example_4.cpp.o" \
"CMakeFiles/main.dir/src/example/example_5.cpp.o" \
"CMakeFiles/main.dir/src/example/example_6.cpp.o" \
"CMakeFiles/main.dir/src/example/example_8.cpp.o" \
"CMakeFiles/main.dir/src/example/example_9.cpp.o" \
"CMakeFiles/main.dir/src/example/example_toolio.cpp.o" \
"CMakeFiles/main.dir/src/example/util.cpp.o"

# External object files for target main
main_EXTERNAL_OBJECTS =

/home/amax/aubo_sdk_test/devel/lib/robot_sdk/main: robot_sdk/CMakeFiles/main.dir/src/main.cpp.o
/home/amax/aubo_sdk_test/devel/lib/robot_sdk/main: robot_sdk/CMakeFiles/main.dir/src/example/AuboSdkExample.cpp.o
/home/amax/aubo_sdk_test/devel/lib/robot_sdk/main: robot_sdk/CMakeFiles/main.dir/src/example/example_0.cpp.o
/home/amax/aubo_sdk_test/devel/lib/robot_sdk/main: robot_sdk/CMakeFiles/main.dir/src/example/example_1.cpp.o
/home/amax/aubo_sdk_test/devel/lib/robot_sdk/main: robot_sdk/CMakeFiles/main.dir/src/example/example_3.cpp.o
/home/amax/aubo_sdk_test/devel/lib/robot_sdk/main: robot_sdk/CMakeFiles/main.dir/src/example/example_4.cpp.o
/home/amax/aubo_sdk_test/devel/lib/robot_sdk/main: robot_sdk/CMakeFiles/main.dir/src/example/example_5.cpp.o
/home/amax/aubo_sdk_test/devel/lib/robot_sdk/main: robot_sdk/CMakeFiles/main.dir/src/example/example_6.cpp.o
/home/amax/aubo_sdk_test/devel/lib/robot_sdk/main: robot_sdk/CMakeFiles/main.dir/src/example/example_8.cpp.o
/home/amax/aubo_sdk_test/devel/lib/robot_sdk/main: robot_sdk/CMakeFiles/main.dir/src/example/example_9.cpp.o
/home/amax/aubo_sdk_test/devel/lib/robot_sdk/main: robot_sdk/CMakeFiles/main.dir/src/example/example_toolio.cpp.o
/home/amax/aubo_sdk_test/devel/lib/robot_sdk/main: robot_sdk/CMakeFiles/main.dir/src/example/util.cpp.o
/home/amax/aubo_sdk_test/devel/lib/robot_sdk/main: robot_sdk/CMakeFiles/main.dir/build.make
/home/amax/aubo_sdk_test/devel/lib/robot_sdk/main: robot_sdk/CMakeFiles/main.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/amax/aubo_sdk_test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_13) "Linking CXX executable /home/amax/aubo_sdk_test/devel/lib/robot_sdk/main"
	cd /home/amax/aubo_sdk_test/build/robot_sdk && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/main.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
robot_sdk/CMakeFiles/main.dir/build: /home/amax/aubo_sdk_test/devel/lib/robot_sdk/main
.PHONY : robot_sdk/CMakeFiles/main.dir/build

robot_sdk/CMakeFiles/main.dir/clean:
	cd /home/amax/aubo_sdk_test/build/robot_sdk && $(CMAKE_COMMAND) -P CMakeFiles/main.dir/cmake_clean.cmake
.PHONY : robot_sdk/CMakeFiles/main.dir/clean

robot_sdk/CMakeFiles/main.dir/depend:
	cd /home/amax/aubo_sdk_test/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/amax/aubo_sdk_test/src /home/amax/aubo_sdk_test/src/robot_sdk /home/amax/aubo_sdk_test/build /home/amax/aubo_sdk_test/build/robot_sdk /home/amax/aubo_sdk_test/build/robot_sdk/CMakeFiles/main.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : robot_sdk/CMakeFiles/main.dir/depend
