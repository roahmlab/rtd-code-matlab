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
include unsupported/doc/examples/CMakeFiles/example_PolynomialSolver1.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include unsupported/doc/examples/CMakeFiles/example_PolynomialSolver1.dir/compiler_depend.make

# Include the progress variables for this target.
include unsupported/doc/examples/CMakeFiles/example_PolynomialSolver1.dir/progress.make

# Include the compile flags for this target's objects.
include unsupported/doc/examples/CMakeFiles/example_PolynomialSolver1.dir/flags.make

unsupported/doc/examples/CMakeFiles/example_PolynomialSolver1.dir/PolynomialSolver1.cpp.o: unsupported/doc/examples/CMakeFiles/example_PolynomialSolver1.dir/flags.make
unsupported/doc/examples/CMakeFiles/example_PolynomialSolver1.dir/PolynomialSolver1.cpp.o: ../unsupported/doc/examples/PolynomialSolver1.cpp
unsupported/doc/examples/CMakeFiles/example_PolynomialSolver1.dir/PolynomialSolver1.cpp.o: unsupported/doc/examples/CMakeFiles/example_PolynomialSolver1.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object unsupported/doc/examples/CMakeFiles/example_PolynomialSolver1.dir/PolynomialSolver1.cpp.o"
	cd "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/build/unsupported/doc/examples" && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT unsupported/doc/examples/CMakeFiles/example_PolynomialSolver1.dir/PolynomialSolver1.cpp.o -MF CMakeFiles/example_PolynomialSolver1.dir/PolynomialSolver1.cpp.o.d -o CMakeFiles/example_PolynomialSolver1.dir/PolynomialSolver1.cpp.o -c "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/unsupported/doc/examples/PolynomialSolver1.cpp"

unsupported/doc/examples/CMakeFiles/example_PolynomialSolver1.dir/PolynomialSolver1.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/example_PolynomialSolver1.dir/PolynomialSolver1.cpp.i"
	cd "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/build/unsupported/doc/examples" && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/unsupported/doc/examples/PolynomialSolver1.cpp" > CMakeFiles/example_PolynomialSolver1.dir/PolynomialSolver1.cpp.i

unsupported/doc/examples/CMakeFiles/example_PolynomialSolver1.dir/PolynomialSolver1.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/example_PolynomialSolver1.dir/PolynomialSolver1.cpp.s"
	cd "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/build/unsupported/doc/examples" && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/unsupported/doc/examples/PolynomialSolver1.cpp" -o CMakeFiles/example_PolynomialSolver1.dir/PolynomialSolver1.cpp.s

# Object files for target example_PolynomialSolver1
example_PolynomialSolver1_OBJECTS = \
"CMakeFiles/example_PolynomialSolver1.dir/PolynomialSolver1.cpp.o"

# External object files for target example_PolynomialSolver1
example_PolynomialSolver1_EXTERNAL_OBJECTS =

unsupported/doc/examples/example_PolynomialSolver1: unsupported/doc/examples/CMakeFiles/example_PolynomialSolver1.dir/PolynomialSolver1.cpp.o
unsupported/doc/examples/example_PolynomialSolver1: unsupported/doc/examples/CMakeFiles/example_PolynomialSolver1.dir/build.make
unsupported/doc/examples/example_PolynomialSolver1: unsupported/doc/examples/CMakeFiles/example_PolynomialSolver1.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable example_PolynomialSolver1"
	cd "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/build/unsupported/doc/examples" && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/example_PolynomialSolver1.dir/link.txt --verbose=$(VERBOSE)
	cd "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/build/unsupported/doc/examples" && ./example_PolynomialSolver1 >/home/sukruthichidananda/ROAHM\ LAB/eigen-3.3.7/build/unsupported/doc/examples/PolynomialSolver1.out

# Rule to build all files generated by this target.
unsupported/doc/examples/CMakeFiles/example_PolynomialSolver1.dir/build: unsupported/doc/examples/example_PolynomialSolver1
.PHONY : unsupported/doc/examples/CMakeFiles/example_PolynomialSolver1.dir/build

unsupported/doc/examples/CMakeFiles/example_PolynomialSolver1.dir/clean:
	cd "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/build/unsupported/doc/examples" && $(CMAKE_COMMAND) -P CMakeFiles/example_PolynomialSolver1.dir/cmake_clean.cmake
.PHONY : unsupported/doc/examples/CMakeFiles/example_PolynomialSolver1.dir/clean

unsupported/doc/examples/CMakeFiles/example_PolynomialSolver1.dir/depend:
	cd "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/build" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7" "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/unsupported/doc/examples" "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/build" "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/build/unsupported/doc/examples" "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/build/unsupported/doc/examples/CMakeFiles/example_PolynomialSolver1.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : unsupported/doc/examples/CMakeFiles/example_PolynomialSolver1.dir/depend
