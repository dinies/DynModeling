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
CMAKE_COMMAND = /Applications/CLion.app/Contents/bin/cmake/bin/cmake

# The command to remove a file.
RM = /Applications/CLion.app/Contents/bin/cmake/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/dinies33/GitRepos/DynModeling

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/dinies33/GitRepos/DynModeling/cmake-build-debug

# Include any dependencies generated for this target.
include executables/CMakeFiles/drawPointMovingTest.dir/depend.make

# Include the progress variables for this target.
include executables/CMakeFiles/drawPointMovingTest.dir/progress.make

# Include the compile flags for this target's objects.
include executables/CMakeFiles/drawPointMovingTest.dir/flags.make

executables/CMakeFiles/drawPointMovingTest.dir/drawPointMovingTest.cpp.o: executables/CMakeFiles/drawPointMovingTest.dir/flags.make
executables/CMakeFiles/drawPointMovingTest.dir/drawPointMovingTest.cpp.o: ../executables/drawPointMovingTest.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/dinies33/GitRepos/DynModeling/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object executables/CMakeFiles/drawPointMovingTest.dir/drawPointMovingTest.cpp.o"
	cd /Users/dinies33/GitRepos/DynModeling/cmake-build-debug/executables && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/drawPointMovingTest.dir/drawPointMovingTest.cpp.o -c /Users/dinies33/GitRepos/DynModeling/executables/drawPointMovingTest.cpp

executables/CMakeFiles/drawPointMovingTest.dir/drawPointMovingTest.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drawPointMovingTest.dir/drawPointMovingTest.cpp.i"
	cd /Users/dinies33/GitRepos/DynModeling/cmake-build-debug/executables && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/dinies33/GitRepos/DynModeling/executables/drawPointMovingTest.cpp > CMakeFiles/drawPointMovingTest.dir/drawPointMovingTest.cpp.i

executables/CMakeFiles/drawPointMovingTest.dir/drawPointMovingTest.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drawPointMovingTest.dir/drawPointMovingTest.cpp.s"
	cd /Users/dinies33/GitRepos/DynModeling/cmake-build-debug/executables && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/dinies33/GitRepos/DynModeling/executables/drawPointMovingTest.cpp -o CMakeFiles/drawPointMovingTest.dir/drawPointMovingTest.cpp.s

executables/CMakeFiles/drawPointMovingTest.dir/drawPointMovingTest.cpp.o.requires:

.PHONY : executables/CMakeFiles/drawPointMovingTest.dir/drawPointMovingTest.cpp.o.requires

executables/CMakeFiles/drawPointMovingTest.dir/drawPointMovingTest.cpp.o.provides: executables/CMakeFiles/drawPointMovingTest.dir/drawPointMovingTest.cpp.o.requires
	$(MAKE) -f executables/CMakeFiles/drawPointMovingTest.dir/build.make executables/CMakeFiles/drawPointMovingTest.dir/drawPointMovingTest.cpp.o.provides.build
.PHONY : executables/CMakeFiles/drawPointMovingTest.dir/drawPointMovingTest.cpp.o.provides

executables/CMakeFiles/drawPointMovingTest.dir/drawPointMovingTest.cpp.o.provides.build: executables/CMakeFiles/drawPointMovingTest.dir/drawPointMovingTest.cpp.o


# Object files for target drawPointMovingTest
drawPointMovingTest_OBJECTS = \
"CMakeFiles/drawPointMovingTest.dir/drawPointMovingTest.cpp.o"

# External object files for target drawPointMovingTest
drawPointMovingTest_EXTERNAL_OBJECTS =

../bin/drawPointMovingTest: executables/CMakeFiles/drawPointMovingTest.dir/drawPointMovingTest.cpp.o
../bin/drawPointMovingTest: executables/CMakeFiles/drawPointMovingTest.dir/build.make
../bin/drawPointMovingTest: /usr/local/lib/libopencv_stitching.3.4.1.dylib
../bin/drawPointMovingTest: /usr/local/lib/libopencv_superres.3.4.1.dylib
../bin/drawPointMovingTest: /usr/local/lib/libopencv_videostab.3.4.1.dylib
../bin/drawPointMovingTest: /usr/local/lib/libopencv_aruco.3.4.1.dylib
../bin/drawPointMovingTest: /usr/local/lib/libopencv_bgsegm.3.4.1.dylib
../bin/drawPointMovingTest: /usr/local/lib/libopencv_bioinspired.3.4.1.dylib
../bin/drawPointMovingTest: /usr/local/lib/libopencv_ccalib.3.4.1.dylib
../bin/drawPointMovingTest: /usr/local/lib/libopencv_dnn_objdetect.3.4.1.dylib
../bin/drawPointMovingTest: /usr/local/lib/libopencv_dpm.3.4.1.dylib
../bin/drawPointMovingTest: /usr/local/lib/libopencv_face.3.4.1.dylib
../bin/drawPointMovingTest: /usr/local/lib/libopencv_fuzzy.3.4.1.dylib
../bin/drawPointMovingTest: /usr/local/lib/libopencv_hfs.3.4.1.dylib
../bin/drawPointMovingTest: /usr/local/lib/libopencv_img_hash.3.4.1.dylib
../bin/drawPointMovingTest: /usr/local/lib/libopencv_line_descriptor.3.4.1.dylib
../bin/drawPointMovingTest: /usr/local/lib/libopencv_optflow.3.4.1.dylib
../bin/drawPointMovingTest: /usr/local/lib/libopencv_reg.3.4.1.dylib
../bin/drawPointMovingTest: /usr/local/lib/libopencv_rgbd.3.4.1.dylib
../bin/drawPointMovingTest: /usr/local/lib/libopencv_saliency.3.4.1.dylib
../bin/drawPointMovingTest: /usr/local/lib/libopencv_stereo.3.4.1.dylib
../bin/drawPointMovingTest: /usr/local/lib/libopencv_structured_light.3.4.1.dylib
../bin/drawPointMovingTest: /usr/local/lib/libopencv_surface_matching.3.4.1.dylib
../bin/drawPointMovingTest: /usr/local/lib/libopencv_tracking.3.4.1.dylib
../bin/drawPointMovingTest: /usr/local/lib/libopencv_xfeatures2d.3.4.1.dylib
../bin/drawPointMovingTest: /usr/local/lib/libopencv_ximgproc.3.4.1.dylib
../bin/drawPointMovingTest: /usr/local/lib/libopencv_xobjdetect.3.4.1.dylib
../bin/drawPointMovingTest: /usr/local/lib/libopencv_xphoto.3.4.1.dylib
../bin/drawPointMovingTest: ../bin/libboost_filesystem.a
../bin/drawPointMovingTest: ../bin/libboost_system.a
../bin/drawPointMovingTest: ../bin/libboost_iostreams.a
../bin/drawPointMovingTest: /usr/local/lib/libopencv_shape.3.4.1.dylib
../bin/drawPointMovingTest: /usr/local/lib/libopencv_photo.3.4.1.dylib
../bin/drawPointMovingTest: /usr/local/lib/libopencv_datasets.3.4.1.dylib
../bin/drawPointMovingTest: /usr/local/lib/libopencv_plot.3.4.1.dylib
../bin/drawPointMovingTest: /usr/local/lib/libopencv_text.3.4.1.dylib
../bin/drawPointMovingTest: /usr/local/lib/libopencv_dnn.3.4.1.dylib
../bin/drawPointMovingTest: /usr/local/lib/libopencv_ml.3.4.1.dylib
../bin/drawPointMovingTest: /usr/local/lib/libopencv_video.3.4.1.dylib
../bin/drawPointMovingTest: /usr/local/lib/libopencv_calib3d.3.4.1.dylib
../bin/drawPointMovingTest: /usr/local/lib/libopencv_features2d.3.4.1.dylib
../bin/drawPointMovingTest: /usr/local/lib/libopencv_highgui.3.4.1.dylib
../bin/drawPointMovingTest: /usr/local/lib/libopencv_videoio.3.4.1.dylib
../bin/drawPointMovingTest: /usr/local/lib/libopencv_phase_unwrapping.3.4.1.dylib
../bin/drawPointMovingTest: /usr/local/lib/libopencv_flann.3.4.1.dylib
../bin/drawPointMovingTest: /usr/local/lib/libopencv_imgcodecs.3.4.1.dylib
../bin/drawPointMovingTest: /usr/local/lib/libopencv_objdetect.3.4.1.dylib
../bin/drawPointMovingTest: /usr/local/lib/libopencv_imgproc.3.4.1.dylib
../bin/drawPointMovingTest: /usr/local/lib/libopencv_core.3.4.1.dylib
../bin/drawPointMovingTest: /usr/lib/libbz2.dylib
../bin/drawPointMovingTest: /usr/lib/libz.dylib
../bin/drawPointMovingTest: executables/CMakeFiles/drawPointMovingTest.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/dinies33/GitRepos/DynModeling/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../bin/drawPointMovingTest"
	cd /Users/dinies33/GitRepos/DynModeling/cmake-build-debug/executables && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/drawPointMovingTest.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
executables/CMakeFiles/drawPointMovingTest.dir/build: ../bin/drawPointMovingTest

.PHONY : executables/CMakeFiles/drawPointMovingTest.dir/build

executables/CMakeFiles/drawPointMovingTest.dir/requires: executables/CMakeFiles/drawPointMovingTest.dir/drawPointMovingTest.cpp.o.requires

.PHONY : executables/CMakeFiles/drawPointMovingTest.dir/requires

executables/CMakeFiles/drawPointMovingTest.dir/clean:
	cd /Users/dinies33/GitRepos/DynModeling/cmake-build-debug/executables && $(CMAKE_COMMAND) -P CMakeFiles/drawPointMovingTest.dir/cmake_clean.cmake
.PHONY : executables/CMakeFiles/drawPointMovingTest.dir/clean

executables/CMakeFiles/drawPointMovingTest.dir/depend:
	cd /Users/dinies33/GitRepos/DynModeling/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/dinies33/GitRepos/DynModeling /Users/dinies33/GitRepos/DynModeling/executables /Users/dinies33/GitRepos/DynModeling/cmake-build-debug /Users/dinies33/GitRepos/DynModeling/cmake-build-debug/executables /Users/dinies33/GitRepos/DynModeling/cmake-build-debug/executables/CMakeFiles/drawPointMovingTest.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : executables/CMakeFiles/drawPointMovingTest.dir/depend

