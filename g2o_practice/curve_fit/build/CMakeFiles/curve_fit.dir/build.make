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
CMAKE_SOURCE_DIR = /home/ypc/xiaoc/code/slam_tool_practice/g2o_practice/curve_fit

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ypc/xiaoc/code/slam_tool_practice/g2o_practice/curve_fit/build

# Include any dependencies generated for this target.
include CMakeFiles/curve_fit.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/curve_fit.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/curve_fit.dir/flags.make

CMakeFiles/curve_fit.dir/src/main.cpp.o: CMakeFiles/curve_fit.dir/flags.make
CMakeFiles/curve_fit.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ypc/xiaoc/code/slam_tool_practice/g2o_practice/curve_fit/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/curve_fit.dir/src/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/curve_fit.dir/src/main.cpp.o -c /home/ypc/xiaoc/code/slam_tool_practice/g2o_practice/curve_fit/src/main.cpp

CMakeFiles/curve_fit.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/curve_fit.dir/src/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ypc/xiaoc/code/slam_tool_practice/g2o_practice/curve_fit/src/main.cpp > CMakeFiles/curve_fit.dir/src/main.cpp.i

CMakeFiles/curve_fit.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/curve_fit.dir/src/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ypc/xiaoc/code/slam_tool_practice/g2o_practice/curve_fit/src/main.cpp -o CMakeFiles/curve_fit.dir/src/main.cpp.s

CMakeFiles/curve_fit.dir/src/main.cpp.o.requires:

.PHONY : CMakeFiles/curve_fit.dir/src/main.cpp.o.requires

CMakeFiles/curve_fit.dir/src/main.cpp.o.provides: CMakeFiles/curve_fit.dir/src/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/curve_fit.dir/build.make CMakeFiles/curve_fit.dir/src/main.cpp.o.provides.build
.PHONY : CMakeFiles/curve_fit.dir/src/main.cpp.o.provides

CMakeFiles/curve_fit.dir/src/main.cpp.o.provides.build: CMakeFiles/curve_fit.dir/src/main.cpp.o


CMakeFiles/curve_fit.dir/src/curve_graph.cpp.o: CMakeFiles/curve_fit.dir/flags.make
CMakeFiles/curve_fit.dir/src/curve_graph.cpp.o: ../src/curve_graph.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ypc/xiaoc/code/slam_tool_practice/g2o_practice/curve_fit/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/curve_fit.dir/src/curve_graph.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/curve_fit.dir/src/curve_graph.cpp.o -c /home/ypc/xiaoc/code/slam_tool_practice/g2o_practice/curve_fit/src/curve_graph.cpp

CMakeFiles/curve_fit.dir/src/curve_graph.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/curve_fit.dir/src/curve_graph.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ypc/xiaoc/code/slam_tool_practice/g2o_practice/curve_fit/src/curve_graph.cpp > CMakeFiles/curve_fit.dir/src/curve_graph.cpp.i

CMakeFiles/curve_fit.dir/src/curve_graph.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/curve_fit.dir/src/curve_graph.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ypc/xiaoc/code/slam_tool_practice/g2o_practice/curve_fit/src/curve_graph.cpp -o CMakeFiles/curve_fit.dir/src/curve_graph.cpp.s

CMakeFiles/curve_fit.dir/src/curve_graph.cpp.o.requires:

.PHONY : CMakeFiles/curve_fit.dir/src/curve_graph.cpp.o.requires

CMakeFiles/curve_fit.dir/src/curve_graph.cpp.o.provides: CMakeFiles/curve_fit.dir/src/curve_graph.cpp.o.requires
	$(MAKE) -f CMakeFiles/curve_fit.dir/build.make CMakeFiles/curve_fit.dir/src/curve_graph.cpp.o.provides.build
.PHONY : CMakeFiles/curve_fit.dir/src/curve_graph.cpp.o.provides

CMakeFiles/curve_fit.dir/src/curve_graph.cpp.o.provides.build: CMakeFiles/curve_fit.dir/src/curve_graph.cpp.o


# Object files for target curve_fit
curve_fit_OBJECTS = \
"CMakeFiles/curve_fit.dir/src/main.cpp.o" \
"CMakeFiles/curve_fit.dir/src/curve_graph.cpp.o"

# External object files for target curve_fit
curve_fit_EXTERNAL_OBJECTS =

../bin/curve_fit: CMakeFiles/curve_fit.dir/src/main.cpp.o
../bin/curve_fit: CMakeFiles/curve_fit.dir/src/curve_graph.cpp.o
../bin/curve_fit: CMakeFiles/curve_fit.dir/build.make
../bin/curve_fit: /usr/lib/x86_64-linux-gnu/libcxsparse.so
../bin/curve_fit: CMakeFiles/curve_fit.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ypc/xiaoc/code/slam_tool_practice/g2o_practice/curve_fit/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable ../bin/curve_fit"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/curve_fit.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/curve_fit.dir/build: ../bin/curve_fit

.PHONY : CMakeFiles/curve_fit.dir/build

CMakeFiles/curve_fit.dir/requires: CMakeFiles/curve_fit.dir/src/main.cpp.o.requires
CMakeFiles/curve_fit.dir/requires: CMakeFiles/curve_fit.dir/src/curve_graph.cpp.o.requires

.PHONY : CMakeFiles/curve_fit.dir/requires

CMakeFiles/curve_fit.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/curve_fit.dir/cmake_clean.cmake
.PHONY : CMakeFiles/curve_fit.dir/clean

CMakeFiles/curve_fit.dir/depend:
	cd /home/ypc/xiaoc/code/slam_tool_practice/g2o_practice/curve_fit/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ypc/xiaoc/code/slam_tool_practice/g2o_practice/curve_fit /home/ypc/xiaoc/code/slam_tool_practice/g2o_practice/curve_fit /home/ypc/xiaoc/code/slam_tool_practice/g2o_practice/curve_fit/build /home/ypc/xiaoc/code/slam_tool_practice/g2o_practice/curve_fit/build /home/ypc/xiaoc/code/slam_tool_practice/g2o_practice/curve_fit/build/CMakeFiles/curve_fit.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/curve_fit.dir/depend
