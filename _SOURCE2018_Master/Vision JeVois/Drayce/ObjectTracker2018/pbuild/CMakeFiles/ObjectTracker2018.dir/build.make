# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.7

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
CMAKE_SOURCE_DIR = /home/matthew/ObjectTracker2018

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/matthew/ObjectTracker2018/pbuild

# Include any dependencies generated for this target.
include CMakeFiles/ObjectTracker2018.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/ObjectTracker2018.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/ObjectTracker2018.dir/flags.make

CMakeFiles/ObjectTracker2018.dir/src/Modules/ObjectTracker2018/ObjectTracker2018.C.o: CMakeFiles/ObjectTracker2018.dir/flags.make
CMakeFiles/ObjectTracker2018.dir/src/Modules/ObjectTracker2018/ObjectTracker2018.C.o: ../src/Modules/ObjectTracker2018/ObjectTracker2018.C
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/matthew/ObjectTracker2018/pbuild/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/ObjectTracker2018.dir/src/Modules/ObjectTracker2018/ObjectTracker2018.C.o"
	/usr/share/jevois-sdk/out/sun8iw5p1/linux/common/buildroot/host/usr/bin/arm-buildroot-linux-gnueabihf-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ObjectTracker2018.dir/src/Modules/ObjectTracker2018/ObjectTracker2018.C.o -c /home/matthew/ObjectTracker2018/src/Modules/ObjectTracker2018/ObjectTracker2018.C

CMakeFiles/ObjectTracker2018.dir/src/Modules/ObjectTracker2018/ObjectTracker2018.C.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ObjectTracker2018.dir/src/Modules/ObjectTracker2018/ObjectTracker2018.C.i"
	/usr/share/jevois-sdk/out/sun8iw5p1/linux/common/buildroot/host/usr/bin/arm-buildroot-linux-gnueabihf-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/matthew/ObjectTracker2018/src/Modules/ObjectTracker2018/ObjectTracker2018.C > CMakeFiles/ObjectTracker2018.dir/src/Modules/ObjectTracker2018/ObjectTracker2018.C.i

CMakeFiles/ObjectTracker2018.dir/src/Modules/ObjectTracker2018/ObjectTracker2018.C.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ObjectTracker2018.dir/src/Modules/ObjectTracker2018/ObjectTracker2018.C.s"
	/usr/share/jevois-sdk/out/sun8iw5p1/linux/common/buildroot/host/usr/bin/arm-buildroot-linux-gnueabihf-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/matthew/ObjectTracker2018/src/Modules/ObjectTracker2018/ObjectTracker2018.C -o CMakeFiles/ObjectTracker2018.dir/src/Modules/ObjectTracker2018/ObjectTracker2018.C.s

CMakeFiles/ObjectTracker2018.dir/src/Modules/ObjectTracker2018/ObjectTracker2018.C.o.requires:

.PHONY : CMakeFiles/ObjectTracker2018.dir/src/Modules/ObjectTracker2018/ObjectTracker2018.C.o.requires

CMakeFiles/ObjectTracker2018.dir/src/Modules/ObjectTracker2018/ObjectTracker2018.C.o.provides: CMakeFiles/ObjectTracker2018.dir/src/Modules/ObjectTracker2018/ObjectTracker2018.C.o.requires
	$(MAKE) -f CMakeFiles/ObjectTracker2018.dir/build.make CMakeFiles/ObjectTracker2018.dir/src/Modules/ObjectTracker2018/ObjectTracker2018.C.o.provides.build
.PHONY : CMakeFiles/ObjectTracker2018.dir/src/Modules/ObjectTracker2018/ObjectTracker2018.C.o.provides

CMakeFiles/ObjectTracker2018.dir/src/Modules/ObjectTracker2018/ObjectTracker2018.C.o.provides.build: CMakeFiles/ObjectTracker2018.dir/src/Modules/ObjectTracker2018/ObjectTracker2018.C.o


# Object files for target ObjectTracker2018
ObjectTracker2018_OBJECTS = \
"CMakeFiles/ObjectTracker2018.dir/src/Modules/ObjectTracker2018/ObjectTracker2018.C.o"

# External object files for target ObjectTracker2018
ObjectTracker2018_EXTERNAL_OBJECTS =

ObjectTracker2018.so: CMakeFiles/ObjectTracker2018.dir/src/Modules/ObjectTracker2018/ObjectTracker2018.C.o
ObjectTracker2018.so: CMakeFiles/ObjectTracker2018.dir/build.make
ObjectTracker2018.so: CMakeFiles/ObjectTracker2018.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/matthew/ObjectTracker2018/pbuild/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library ObjectTracker2018.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ObjectTracker2018.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/ObjectTracker2018.dir/build: ObjectTracker2018.so

.PHONY : CMakeFiles/ObjectTracker2018.dir/build

CMakeFiles/ObjectTracker2018.dir/requires: CMakeFiles/ObjectTracker2018.dir/src/Modules/ObjectTracker2018/ObjectTracker2018.C.o.requires

.PHONY : CMakeFiles/ObjectTracker2018.dir/requires

CMakeFiles/ObjectTracker2018.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ObjectTracker2018.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ObjectTracker2018.dir/clean

CMakeFiles/ObjectTracker2018.dir/depend:
	cd /home/matthew/ObjectTracker2018/pbuild && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/matthew/ObjectTracker2018 /home/matthew/ObjectTracker2018 /home/matthew/ObjectTracker2018/pbuild /home/matthew/ObjectTracker2018/pbuild /home/matthew/ObjectTracker2018/pbuild/CMakeFiles/ObjectTracker2018.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ObjectTracker2018.dir/depend

