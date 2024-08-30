# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.30

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
CMAKE_COMMAND = /home/liuhongcheng/anaconda3/envs/Commonroad/lib/python3.9/site-packages/cmake/data/bin/cmake

# The command to remove a file.
RM = /home/liuhongcheng/anaconda3/envs/Commonroad/lib/python3.9/site-packages/cmake/data/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/liuhongcheng/Desktop/AS_cpp/Bezier/opt

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/liuhongcheng/Desktop/AS_cpp/Bezier/opt/build

# Include any dependencies generated for this target.
include bspline/CMakeFiles/bspline.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include bspline/CMakeFiles/bspline.dir/compiler_depend.make

# Include the progress variables for this target.
include bspline/CMakeFiles/bspline.dir/progress.make

# Include the compile flags for this target's objects.
include bspline/CMakeFiles/bspline.dir/flags.make

bspline/CMakeFiles/bspline.dir/src/non_uniform_bspline.cpp.o: bspline/CMakeFiles/bspline.dir/flags.make
bspline/CMakeFiles/bspline.dir/src/non_uniform_bspline.cpp.o: /home/liuhongcheng/Desktop/AS_cpp/Bezier/opt/bspline/src/non_uniform_bspline.cpp
bspline/CMakeFiles/bspline.dir/src/non_uniform_bspline.cpp.o: bspline/CMakeFiles/bspline.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/liuhongcheng/Desktop/AS_cpp/Bezier/opt/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object bspline/CMakeFiles/bspline.dir/src/non_uniform_bspline.cpp.o"
	cd /home/liuhongcheng/Desktop/AS_cpp/Bezier/opt/build/bspline && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT bspline/CMakeFiles/bspline.dir/src/non_uniform_bspline.cpp.o -MF CMakeFiles/bspline.dir/src/non_uniform_bspline.cpp.o.d -o CMakeFiles/bspline.dir/src/non_uniform_bspline.cpp.o -c /home/liuhongcheng/Desktop/AS_cpp/Bezier/opt/bspline/src/non_uniform_bspline.cpp

bspline/CMakeFiles/bspline.dir/src/non_uniform_bspline.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/bspline.dir/src/non_uniform_bspline.cpp.i"
	cd /home/liuhongcheng/Desktop/AS_cpp/Bezier/opt/build/bspline && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/liuhongcheng/Desktop/AS_cpp/Bezier/opt/bspline/src/non_uniform_bspline.cpp > CMakeFiles/bspline.dir/src/non_uniform_bspline.cpp.i

bspline/CMakeFiles/bspline.dir/src/non_uniform_bspline.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/bspline.dir/src/non_uniform_bspline.cpp.s"
	cd /home/liuhongcheng/Desktop/AS_cpp/Bezier/opt/build/bspline && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/liuhongcheng/Desktop/AS_cpp/Bezier/opt/bspline/src/non_uniform_bspline.cpp -o CMakeFiles/bspline.dir/src/non_uniform_bspline.cpp.s

# Object files for target bspline
bspline_OBJECTS = \
"CMakeFiles/bspline.dir/src/non_uniform_bspline.cpp.o"

# External object files for target bspline
bspline_EXTERNAL_OBJECTS =

bspline/libbspline.a: bspline/CMakeFiles/bspline.dir/src/non_uniform_bspline.cpp.o
bspline/libbspline.a: bspline/CMakeFiles/bspline.dir/build.make
bspline/libbspline.a: bspline/CMakeFiles/bspline.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/liuhongcheng/Desktop/AS_cpp/Bezier/opt/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libbspline.a"
	cd /home/liuhongcheng/Desktop/AS_cpp/Bezier/opt/build/bspline && $(CMAKE_COMMAND) -P CMakeFiles/bspline.dir/cmake_clean_target.cmake
	cd /home/liuhongcheng/Desktop/AS_cpp/Bezier/opt/build/bspline && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/bspline.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
bspline/CMakeFiles/bspline.dir/build: bspline/libbspline.a
.PHONY : bspline/CMakeFiles/bspline.dir/build

bspline/CMakeFiles/bspline.dir/clean:
	cd /home/liuhongcheng/Desktop/AS_cpp/Bezier/opt/build/bspline && $(CMAKE_COMMAND) -P CMakeFiles/bspline.dir/cmake_clean.cmake
.PHONY : bspline/CMakeFiles/bspline.dir/clean

bspline/CMakeFiles/bspline.dir/depend:
	cd /home/liuhongcheng/Desktop/AS_cpp/Bezier/opt/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/liuhongcheng/Desktop/AS_cpp/Bezier/opt /home/liuhongcheng/Desktop/AS_cpp/Bezier/opt/bspline /home/liuhongcheng/Desktop/AS_cpp/Bezier/opt/build /home/liuhongcheng/Desktop/AS_cpp/Bezier/opt/build/bspline /home/liuhongcheng/Desktop/AS_cpp/Bezier/opt/build/bspline/CMakeFiles/bspline.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : bspline/CMakeFiles/bspline.dir/depend

