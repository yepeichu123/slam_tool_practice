# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/xiaoc/xiaoc/code/tool_practice/eigen_practice/RotateTransform

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/xiaoc/xiaoc/code/tool_practice/eigen_practice/RotateTransform/build

# Include any dependencies generated for this target.
include CMakeFiles/RotateTransform.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/RotateTransform.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/RotateTransform.dir/flags.make

CMakeFiles/RotateTransform.dir/src/main.cpp.o: CMakeFiles/RotateTransform.dir/flags.make
CMakeFiles/RotateTransform.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/xiaoc/xiaoc/code/tool_practice/eigen_practice/RotateTransform/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/RotateTransform.dir/src/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/RotateTransform.dir/src/main.cpp.o -c /home/xiaoc/xiaoc/code/tool_practice/eigen_practice/RotateTransform/src/main.cpp

CMakeFiles/RotateTransform.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/RotateTransform.dir/src/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/xiaoc/xiaoc/code/tool_practice/eigen_practice/RotateTransform/src/main.cpp > CMakeFiles/RotateTransform.dir/src/main.cpp.i

CMakeFiles/RotateTransform.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/RotateTransform.dir/src/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/xiaoc/xiaoc/code/tool_practice/eigen_practice/RotateTransform/src/main.cpp -o CMakeFiles/RotateTransform.dir/src/main.cpp.s

CMakeFiles/RotateTransform.dir/src/main.cpp.o.requires:

.PHONY : CMakeFiles/RotateTransform.dir/src/main.cpp.o.requires

CMakeFiles/RotateTransform.dir/src/main.cpp.o.provides: CMakeFiles/RotateTransform.dir/src/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/RotateTransform.dir/build.make CMakeFiles/RotateTransform.dir/src/main.cpp.o.provides.build
.PHONY : CMakeFiles/RotateTransform.dir/src/main.cpp.o.provides

CMakeFiles/RotateTransform.dir/src/main.cpp.o.provides.build: CMakeFiles/RotateTransform.dir/src/main.cpp.o


CMakeFiles/RotateTransform.dir/src/RotateTransform.cpp.o: CMakeFiles/RotateTransform.dir/flags.make
CMakeFiles/RotateTransform.dir/src/RotateTransform.cpp.o: ../src/RotateTransform.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/xiaoc/xiaoc/code/tool_practice/eigen_practice/RotateTransform/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/RotateTransform.dir/src/RotateTransform.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/RotateTransform.dir/src/RotateTransform.cpp.o -c /home/xiaoc/xiaoc/code/tool_practice/eigen_practice/RotateTransform/src/RotateTransform.cpp

CMakeFiles/RotateTransform.dir/src/RotateTransform.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/RotateTransform.dir/src/RotateTransform.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/xiaoc/xiaoc/code/tool_practice/eigen_practice/RotateTransform/src/RotateTransform.cpp > CMakeFiles/RotateTransform.dir/src/RotateTransform.cpp.i

CMakeFiles/RotateTransform.dir/src/RotateTransform.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/RotateTransform.dir/src/RotateTransform.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/xiaoc/xiaoc/code/tool_practice/eigen_practice/RotateTransform/src/RotateTransform.cpp -o CMakeFiles/RotateTransform.dir/src/RotateTransform.cpp.s

CMakeFiles/RotateTransform.dir/src/RotateTransform.cpp.o.requires:

.PHONY : CMakeFiles/RotateTransform.dir/src/RotateTransform.cpp.o.requires

CMakeFiles/RotateTransform.dir/src/RotateTransform.cpp.o.provides: CMakeFiles/RotateTransform.dir/src/RotateTransform.cpp.o.requires
	$(MAKE) -f CMakeFiles/RotateTransform.dir/build.make CMakeFiles/RotateTransform.dir/src/RotateTransform.cpp.o.provides.build
.PHONY : CMakeFiles/RotateTransform.dir/src/RotateTransform.cpp.o.provides

CMakeFiles/RotateTransform.dir/src/RotateTransform.cpp.o.provides.build: CMakeFiles/RotateTransform.dir/src/RotateTransform.cpp.o


# Object files for target RotateTransform
RotateTransform_OBJECTS = \
"CMakeFiles/RotateTransform.dir/src/main.cpp.o" \
"CMakeFiles/RotateTransform.dir/src/RotateTransform.cpp.o"

# External object files for target RotateTransform
RotateTransform_EXTERNAL_OBJECTS =

../bin/RotateTransform: CMakeFiles/RotateTransform.dir/src/main.cpp.o
../bin/RotateTransform: CMakeFiles/RotateTransform.dir/src/RotateTransform.cpp.o
../bin/RotateTransform: CMakeFiles/RotateTransform.dir/build.make
../bin/RotateTransform: CMakeFiles/RotateTransform.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/xiaoc/xiaoc/code/tool_practice/eigen_practice/RotateTransform/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable ../bin/RotateTransform"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/RotateTransform.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/RotateTransform.dir/build: ../bin/RotateTransform

.PHONY : CMakeFiles/RotateTransform.dir/build

CMakeFiles/RotateTransform.dir/requires: CMakeFiles/RotateTransform.dir/src/main.cpp.o.requires
CMakeFiles/RotateTransform.dir/requires: CMakeFiles/RotateTransform.dir/src/RotateTransform.cpp.o.requires

.PHONY : CMakeFiles/RotateTransform.dir/requires

CMakeFiles/RotateTransform.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/RotateTransform.dir/cmake_clean.cmake
.PHONY : CMakeFiles/RotateTransform.dir/clean

CMakeFiles/RotateTransform.dir/depend:
	cd /home/xiaoc/xiaoc/code/tool_practice/eigen_practice/RotateTransform/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/xiaoc/xiaoc/code/tool_practice/eigen_practice/RotateTransform /home/xiaoc/xiaoc/code/tool_practice/eigen_practice/RotateTransform /home/xiaoc/xiaoc/code/tool_practice/eigen_practice/RotateTransform/build /home/xiaoc/xiaoc/code/tool_practice/eigen_practice/RotateTransform/build /home/xiaoc/xiaoc/code/tool_practice/eigen_practice/RotateTransform/build/CMakeFiles/RotateTransform.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/RotateTransform.dir/depend

