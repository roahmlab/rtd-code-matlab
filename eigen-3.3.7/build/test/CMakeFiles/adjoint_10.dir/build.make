# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/build"

# Include any dependencies generated for this target.
include test/CMakeFiles/adjoint_10.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include test/CMakeFiles/adjoint_10.dir/compiler_depend.make

# Include the progress variables for this target.
include test/CMakeFiles/adjoint_10.dir/progress.make

# Include the compile flags for this target's objects.
include test/CMakeFiles/adjoint_10.dir/flags.make

test/CMakeFiles/adjoint_10.dir/adjoint.cpp.o: test/CMakeFiles/adjoint_10.dir/flags.make
test/CMakeFiles/adjoint_10.dir/adjoint.cpp.o: ../test/adjoint.cpp
test/CMakeFiles/adjoint_10.dir/adjoint.cpp.o: test/CMakeFiles/adjoint_10.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object test/CMakeFiles/adjoint_10.dir/adjoint.cpp.o"
	cd "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/build/test" && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT test/CMakeFiles/adjoint_10.dir/adjoint.cpp.o -MF CMakeFiles/adjoint_10.dir/adjoint.cpp.o.d -o CMakeFiles/adjoint_10.dir/adjoint.cpp.o -c "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/test/adjoint.cpp"

test/CMakeFiles/adjoint_10.dir/adjoint.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/adjoint_10.dir/adjoint.cpp.i"
	cd "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/build/test" && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/test/adjoint.cpp" > CMakeFiles/adjoint_10.dir/adjoint.cpp.i

test/CMakeFiles/adjoint_10.dir/adjoint.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/adjoint_10.dir/adjoint.cpp.s"
	cd "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/build/test" && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/test/adjoint.cpp" -o CMakeFiles/adjoint_10.dir/adjoint.cpp.s

# Object files for target adjoint_10
adjoint_10_OBJECTS = \
"CMakeFiles/adjoint_10.dir/adjoint.cpp.o"

# External object files for target adjoint_10
adjoint_10_EXTERNAL_OBJECTS =

test/adjoint_10: test/CMakeFiles/adjoint_10.dir/adjoint.cpp.o
test/adjoint_10: test/CMakeFiles/adjoint_10.dir/build.make
test/adjoint_10: test/CMakeFiles/adjoint_10.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable adjoint_10"
	cd "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/build/test" && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/adjoint_10.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/CMakeFiles/adjoint_10.dir/build: test/adjoint_10
.PHONY : test/CMakeFiles/adjoint_10.dir/build

test/CMakeFiles/adjoint_10.dir/clean:
	cd "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/build/test" && $(CMAKE_COMMAND) -P CMakeFiles/adjoint_10.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/adjoint_10.dir/clean

test/CMakeFiles/adjoint_10.dir/depend:
	cd "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/build" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7" "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/test" "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/build" "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/build/test" "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/build/test/CMakeFiles/adjoint_10.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : test/CMakeFiles/adjoint_10.dir/depend

