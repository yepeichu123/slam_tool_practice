# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.9

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ypc/xiaoc/code/slam_tool_practice/g2o_practice/icp_demo

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ypc/xiaoc/code/slam_tool_practice/g2o_practice/icp_demo/build

# Include any dependencies generated for this target.
include CMakeFiles/icp_demo.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/icp_demo.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/icp_demo.dir/flags.make

CMakeFiles/icp_demo.dir/src/main.cpp.o: CMakeFiles/icp_demo.dir/flags.make
CMakeFiles/icp_demo.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ypc/xiaoc/code/slam_tool_practice/g2o_practice/icp_demo/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/icp_demo.dir/src/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/icp_demo.dir/src/main.cpp.o -c /home/ypc/xiaoc/code/slam_tool_practice/g2o_practice/icp_demo/src/main.cpp

CMakeFiles/icp_demo.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/icp_demo.dir/src/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ypc/xiaoc/code/slam_tool_practice/g2o_practice/icp_demo/src/main.cpp > CMakeFiles/icp_demo.dir/src/main.cpp.i

CMakeFiles/icp_demo.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/icp_demo.dir/src/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ypc/xiaoc/code/slam_tool_practice/g2o_practice/icp_demo/src/main.cpp -o CMakeFiles/icp_demo.dir/src/main.cpp.s

CMakeFiles/icp_demo.dir/src/main.cpp.o.requires:

.PHONY : CMakeFiles/icp_demo.dir/src/main.cpp.o.requires

CMakeFiles/icp_demo.dir/src/main.cpp.o.provides: CMakeFiles/icp_demo.dir/src/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/icp_demo.dir/build.make CMakeFiles/icp_demo.dir/src/main.cpp.o.provides.build
.PHONY : CMakeFiles/icp_demo.dir/src/main.cpp.o.provides

CMakeFiles/icp_demo.dir/src/main.cpp.o.provides.build: CMakeFiles/icp_demo.dir/src/main.cpp.o


# Object files for target icp_demo
icp_demo_OBJECTS = \
"CMakeFiles/icp_demo.dir/src/main.cpp.o"

# External object files for target icp_demo
icp_demo_EXTERNAL_OBJECTS =

../bin/icp_demo: CMakeFiles/icp_demo.dir/src/main.cpp.o
../bin/icp_demo: CMakeFiles/icp_demo.dir/build.make
../bin/icp_demo: /usr/lib/x86_64-linux-gnu/libcxsparse.so
../bin/icp_demo: CMakeFiles/icp_demo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ypc/xiaoc/code/slam_tool_practice/g2o_practice/icp_demo/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../bin/icp_demo"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/icp_demo.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/icp_demo.dir/build: ../bin/icp_demo

.PHONY : CMakeFiles/icp_demo.dir/build

CMakeFiles/icp_demo.dir/requires: CMakeFiles/icp_demo.dir/src/main.cpp.o.requires

.PHONY : CMakeFiles/icp_demo.dir/requires

CMakeFiles/icp_demo.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/icp_demo.dir/cmake_clean.cmake
.PHONY : CMakeFiles/icp_demo.dir/clean

CMakeFiles/icp_demo.dir/depend:
	cd /home/ypc/xiaoc/code/slam_tool_practice/g2o_practice/icp_demo/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ypc/xiaoc/code/slam_tool_practice/g2o_practice/icp_demo /home/ypc/xiaoc/code/slam_tool_practice/g2o_practice/icp_demo /home/ypc/xiaoc/code/slam_tool_practice/g2o_practice/icp_demo/build /home/ypc/xiaoc/code/slam_tool_practice/g2o_practice/icp_demo/build /home/ypc/xiaoc/code/slam_tool_practice/g2o_practice/icp_demo/build/CMakeFiles/icp_demo.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/icp_demo.dir/depend

