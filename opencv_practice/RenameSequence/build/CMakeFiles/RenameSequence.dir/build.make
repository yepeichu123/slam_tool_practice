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
CMAKE_SOURCE_DIR = /home/xiaoc/xiaoc/code/tool_practice/opencv_practice/RenameSequence

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/xiaoc/xiaoc/code/tool_practice/opencv_practice/RenameSequence/build

# Include any dependencies generated for this target.
include CMakeFiles/RenameSequence.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/RenameSequence.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/RenameSequence.dir/flags.make

CMakeFiles/RenameSequence.dir/src/main.cpp.o: CMakeFiles/RenameSequence.dir/flags.make
CMakeFiles/RenameSequence.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/xiaoc/xiaoc/code/tool_practice/opencv_practice/RenameSequence/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/RenameSequence.dir/src/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/RenameSequence.dir/src/main.cpp.o -c /home/xiaoc/xiaoc/code/tool_practice/opencv_practice/RenameSequence/src/main.cpp

CMakeFiles/RenameSequence.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/RenameSequence.dir/src/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/xiaoc/xiaoc/code/tool_practice/opencv_practice/RenameSequence/src/main.cpp > CMakeFiles/RenameSequence.dir/src/main.cpp.i

CMakeFiles/RenameSequence.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/RenameSequence.dir/src/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/xiaoc/xiaoc/code/tool_practice/opencv_practice/RenameSequence/src/main.cpp -o CMakeFiles/RenameSequence.dir/src/main.cpp.s

CMakeFiles/RenameSequence.dir/src/main.cpp.o.requires:

.PHONY : CMakeFiles/RenameSequence.dir/src/main.cpp.o.requires

CMakeFiles/RenameSequence.dir/src/main.cpp.o.provides: CMakeFiles/RenameSequence.dir/src/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/RenameSequence.dir/build.make CMakeFiles/RenameSequence.dir/src/main.cpp.o.provides.build
.PHONY : CMakeFiles/RenameSequence.dir/src/main.cpp.o.provides

CMakeFiles/RenameSequence.dir/src/main.cpp.o.provides.build: CMakeFiles/RenameSequence.dir/src/main.cpp.o


CMakeFiles/RenameSequence.dir/src/RenameSequence.cpp.o: CMakeFiles/RenameSequence.dir/flags.make
CMakeFiles/RenameSequence.dir/src/RenameSequence.cpp.o: ../src/RenameSequence.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/xiaoc/xiaoc/code/tool_practice/opencv_practice/RenameSequence/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/RenameSequence.dir/src/RenameSequence.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/RenameSequence.dir/src/RenameSequence.cpp.o -c /home/xiaoc/xiaoc/code/tool_practice/opencv_practice/RenameSequence/src/RenameSequence.cpp

CMakeFiles/RenameSequence.dir/src/RenameSequence.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/RenameSequence.dir/src/RenameSequence.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/xiaoc/xiaoc/code/tool_practice/opencv_practice/RenameSequence/src/RenameSequence.cpp > CMakeFiles/RenameSequence.dir/src/RenameSequence.cpp.i

CMakeFiles/RenameSequence.dir/src/RenameSequence.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/RenameSequence.dir/src/RenameSequence.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/xiaoc/xiaoc/code/tool_practice/opencv_practice/RenameSequence/src/RenameSequence.cpp -o CMakeFiles/RenameSequence.dir/src/RenameSequence.cpp.s

CMakeFiles/RenameSequence.dir/src/RenameSequence.cpp.o.requires:

.PHONY : CMakeFiles/RenameSequence.dir/src/RenameSequence.cpp.o.requires

CMakeFiles/RenameSequence.dir/src/RenameSequence.cpp.o.provides: CMakeFiles/RenameSequence.dir/src/RenameSequence.cpp.o.requires
	$(MAKE) -f CMakeFiles/RenameSequence.dir/build.make CMakeFiles/RenameSequence.dir/src/RenameSequence.cpp.o.provides.build
.PHONY : CMakeFiles/RenameSequence.dir/src/RenameSequence.cpp.o.provides

CMakeFiles/RenameSequence.dir/src/RenameSequence.cpp.o.provides.build: CMakeFiles/RenameSequence.dir/src/RenameSequence.cpp.o


# Object files for target RenameSequence
RenameSequence_OBJECTS = \
"CMakeFiles/RenameSequence.dir/src/main.cpp.o" \
"CMakeFiles/RenameSequence.dir/src/RenameSequence.cpp.o"

# External object files for target RenameSequence
RenameSequence_EXTERNAL_OBJECTS =

../bin/RenameSequence: CMakeFiles/RenameSequence.dir/src/main.cpp.o
../bin/RenameSequence: CMakeFiles/RenameSequence.dir/src/RenameSequence.cpp.o
../bin/RenameSequence: CMakeFiles/RenameSequence.dir/build.make
../bin/RenameSequence: /usr/local/lib/libopencv_shape.so.3.2.0
../bin/RenameSequence: /usr/local/lib/libopencv_stitching.so.3.2.0
../bin/RenameSequence: /usr/local/lib/libopencv_superres.so.3.2.0
../bin/RenameSequence: /usr/local/lib/libopencv_videostab.so.3.2.0
../bin/RenameSequence: /usr/local/lib/libopencv_viz.so.3.2.0
../bin/RenameSequence: /usr/local/lib/libopencv_objdetect.so.3.2.0
../bin/RenameSequence: /usr/local/lib/libopencv_calib3d.so.3.2.0
../bin/RenameSequence: /usr/local/lib/libopencv_features2d.so.3.2.0
../bin/RenameSequence: /usr/local/lib/libopencv_flann.so.3.2.0
../bin/RenameSequence: /usr/local/lib/libopencv_highgui.so.3.2.0
../bin/RenameSequence: /usr/local/lib/libopencv_ml.so.3.2.0
../bin/RenameSequence: /usr/local/lib/libopencv_photo.so.3.2.0
../bin/RenameSequence: /usr/local/lib/libopencv_video.so.3.2.0
../bin/RenameSequence: /usr/local/lib/libopencv_videoio.so.3.2.0
../bin/RenameSequence: /usr/local/lib/libopencv_imgcodecs.so.3.2.0
../bin/RenameSequence: /usr/local/lib/libopencv_imgproc.so.3.2.0
../bin/RenameSequence: /usr/local/lib/libopencv_core.so.3.2.0
../bin/RenameSequence: CMakeFiles/RenameSequence.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/xiaoc/xiaoc/code/tool_practice/opencv_practice/RenameSequence/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable ../bin/RenameSequence"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/RenameSequence.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/RenameSequence.dir/build: ../bin/RenameSequence

.PHONY : CMakeFiles/RenameSequence.dir/build

CMakeFiles/RenameSequence.dir/requires: CMakeFiles/RenameSequence.dir/src/main.cpp.o.requires
CMakeFiles/RenameSequence.dir/requires: CMakeFiles/RenameSequence.dir/src/RenameSequence.cpp.o.requires

.PHONY : CMakeFiles/RenameSequence.dir/requires

CMakeFiles/RenameSequence.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/RenameSequence.dir/cmake_clean.cmake
.PHONY : CMakeFiles/RenameSequence.dir/clean

CMakeFiles/RenameSequence.dir/depend:
	cd /home/xiaoc/xiaoc/code/tool_practice/opencv_practice/RenameSequence/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/xiaoc/xiaoc/code/tool_practice/opencv_practice/RenameSequence /home/xiaoc/xiaoc/code/tool_practice/opencv_practice/RenameSequence /home/xiaoc/xiaoc/code/tool_practice/opencv_practice/RenameSequence/build /home/xiaoc/xiaoc/code/tool_practice/opencv_practice/RenameSequence/build /home/xiaoc/xiaoc/code/tool_practice/opencv_practice/RenameSequence/build/CMakeFiles/RenameSequence.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/RenameSequence.dir/depend

