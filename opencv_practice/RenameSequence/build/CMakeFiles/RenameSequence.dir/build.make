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
CMAKE_SOURCE_DIR = /home/ypc/xiaoc/code/slam_tool_practice/opencv_practice/RenameSequence

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ypc/xiaoc/code/slam_tool_practice/opencv_practice/RenameSequence/build

# Include any dependencies generated for this target.
include CMakeFiles/RenameSequence.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/RenameSequence.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/RenameSequence.dir/flags.make

CMakeFiles/RenameSequence.dir/src/main.cpp.o: CMakeFiles/RenameSequence.dir/flags.make
CMakeFiles/RenameSequence.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ypc/xiaoc/code/slam_tool_practice/opencv_practice/RenameSequence/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/RenameSequence.dir/src/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/RenameSequence.dir/src/main.cpp.o -c /home/ypc/xiaoc/code/slam_tool_practice/opencv_practice/RenameSequence/src/main.cpp

CMakeFiles/RenameSequence.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/RenameSequence.dir/src/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ypc/xiaoc/code/slam_tool_practice/opencv_practice/RenameSequence/src/main.cpp > CMakeFiles/RenameSequence.dir/src/main.cpp.i

CMakeFiles/RenameSequence.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/RenameSequence.dir/src/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ypc/xiaoc/code/slam_tool_practice/opencv_practice/RenameSequence/src/main.cpp -o CMakeFiles/RenameSequence.dir/src/main.cpp.s

CMakeFiles/RenameSequence.dir/src/main.cpp.o.requires:

.PHONY : CMakeFiles/RenameSequence.dir/src/main.cpp.o.requires

CMakeFiles/RenameSequence.dir/src/main.cpp.o.provides: CMakeFiles/RenameSequence.dir/src/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/RenameSequence.dir/build.make CMakeFiles/RenameSequence.dir/src/main.cpp.o.provides.build
.PHONY : CMakeFiles/RenameSequence.dir/src/main.cpp.o.provides

CMakeFiles/RenameSequence.dir/src/main.cpp.o.provides.build: CMakeFiles/RenameSequence.dir/src/main.cpp.o


CMakeFiles/RenameSequence.dir/src/RenameSequence.cpp.o: CMakeFiles/RenameSequence.dir/flags.make
CMakeFiles/RenameSequence.dir/src/RenameSequence.cpp.o: ../src/RenameSequence.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ypc/xiaoc/code/slam_tool_practice/opencv_practice/RenameSequence/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/RenameSequence.dir/src/RenameSequence.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/RenameSequence.dir/src/RenameSequence.cpp.o -c /home/ypc/xiaoc/code/slam_tool_practice/opencv_practice/RenameSequence/src/RenameSequence.cpp

CMakeFiles/RenameSequence.dir/src/RenameSequence.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/RenameSequence.dir/src/RenameSequence.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ypc/xiaoc/code/slam_tool_practice/opencv_practice/RenameSequence/src/RenameSequence.cpp > CMakeFiles/RenameSequence.dir/src/RenameSequence.cpp.i

CMakeFiles/RenameSequence.dir/src/RenameSequence.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/RenameSequence.dir/src/RenameSequence.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ypc/xiaoc/code/slam_tool_practice/opencv_practice/RenameSequence/src/RenameSequence.cpp -o CMakeFiles/RenameSequence.dir/src/RenameSequence.cpp.s

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
../bin/RenameSequence: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_stitching3.so.3.3.1
../bin/RenameSequence: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_superres3.so.3.3.1
../bin/RenameSequence: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_videostab3.so.3.3.1
../bin/RenameSequence: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_aruco3.so.3.3.1
../bin/RenameSequence: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_bgsegm3.so.3.3.1
../bin/RenameSequence: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_bioinspired3.so.3.3.1
../bin/RenameSequence: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ccalib3.so.3.3.1
../bin/RenameSequence: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_cvv3.so.3.3.1
../bin/RenameSequence: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_dpm3.so.3.3.1
../bin/RenameSequence: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_face3.so.3.3.1
../bin/RenameSequence: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_fuzzy3.so.3.3.1
../bin/RenameSequence: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_hdf3.so.3.3.1
../bin/RenameSequence: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_img_hash3.so.3.3.1
../bin/RenameSequence: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_line_descriptor3.so.3.3.1
../bin/RenameSequence: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_optflow3.so.3.3.1
../bin/RenameSequence: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_reg3.so.3.3.1
../bin/RenameSequence: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_rgbd3.so.3.3.1
../bin/RenameSequence: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_saliency3.so.3.3.1
../bin/RenameSequence: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_stereo3.so.3.3.1
../bin/RenameSequence: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_structured_light3.so.3.3.1
../bin/RenameSequence: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_surface_matching3.so.3.3.1
../bin/RenameSequence: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_tracking3.so.3.3.1
../bin/RenameSequence: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xfeatures2d3.so.3.3.1
../bin/RenameSequence: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ximgproc3.so.3.3.1
../bin/RenameSequence: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xobjdetect3.so.3.3.1
../bin/RenameSequence: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xphoto3.so.3.3.1
../bin/RenameSequence: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_shape3.so.3.3.1
../bin/RenameSequence: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_photo3.so.3.3.1
../bin/RenameSequence: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_datasets3.so.3.3.1
../bin/RenameSequence: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_plot3.so.3.3.1
../bin/RenameSequence: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_text3.so.3.3.1
../bin/RenameSequence: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_dnn3.so.3.3.1
../bin/RenameSequence: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ml3.so.3.3.1
../bin/RenameSequence: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_video3.so.3.3.1
../bin/RenameSequence: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_calib3d3.so.3.3.1
../bin/RenameSequence: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_features2d3.so.3.3.1
../bin/RenameSequence: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_highgui3.so.3.3.1
../bin/RenameSequence: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_videoio3.so.3.3.1
../bin/RenameSequence: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_viz3.so.3.3.1
../bin/RenameSequence: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_phase_unwrapping3.so.3.3.1
../bin/RenameSequence: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_flann3.so.3.3.1
../bin/RenameSequence: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgcodecs3.so.3.3.1
../bin/RenameSequence: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_objdetect3.so.3.3.1
../bin/RenameSequence: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgproc3.so.3.3.1
../bin/RenameSequence: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_core3.so.3.3.1
../bin/RenameSequence: CMakeFiles/RenameSequence.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ypc/xiaoc/code/slam_tool_practice/opencv_practice/RenameSequence/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable ../bin/RenameSequence"
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
	cd /home/ypc/xiaoc/code/slam_tool_practice/opencv_practice/RenameSequence/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ypc/xiaoc/code/slam_tool_practice/opencv_practice/RenameSequence /home/ypc/xiaoc/code/slam_tool_practice/opencv_practice/RenameSequence /home/ypc/xiaoc/code/slam_tool_practice/opencv_practice/RenameSequence/build /home/ypc/xiaoc/code/slam_tool_practice/opencv_practice/RenameSequence/build /home/ypc/xiaoc/code/slam_tool_practice/opencv_practice/RenameSequence/build/CMakeFiles/RenameSequence.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/RenameSequence.dir/depend

