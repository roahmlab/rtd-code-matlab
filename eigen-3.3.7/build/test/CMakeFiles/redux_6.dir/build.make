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
include test/CMakeFiles/redux_6.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include test/CMakeFiles/redux_6.dir/compiler_depend.make

# Include the progress variables for this target.
include test/CMakeFiles/redux_6.dir/progress.make

# Include the compile flags for this target's objects.
include test/CMakeFiles/redux_6.dir/flags.make

test/CMakeFiles/redux_6.dir/redux.cpp.o: test/CMakeFiles/redux_6.dir/flags.make
test/CMakeFiles/redux_6.dir/redux.cpp.o: ../test/redux.cpp
test/CMakeFiles/redux_6.dir/redux.cpp.o: test/CMakeFiles/redux_6.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object test/CMakeFiles/redux_6.dir/redux.cpp.o"
	cd "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/build/test" && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT test/CMakeFiles/redux_6.dir/redux.cpp.o -MF CMakeFiles/redux_6.dir/redux.cpp.o.d -o CMakeFiles/redux_6.dir/redux.cpp.o -c "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/test/redux.cpp"

test/CMakeFiles/redux_6.dir/redux.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/redux_6.dir/redux.cpp.i"
	cd "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/build/test" && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/test/redux.cpp" > CMakeFiles/redux_6.dir/redux.cpp.i

test/CMakeFiles/redux_6.dir/redux.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/redux_6.dir/redux.cpp.s"
	cd "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/build/test" && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/test/redux.cpp" -o CMakeFiles/redux_6.dir/redux.cpp.s

# Object files for target redux_6
redux_6_OBJECTS = \
"CMakeFiles/redux_6.dir/redux.cpp.o"

# External object files for target redux_6
redux_6_EXTERNAL_OBJECTS =

test/redux_6: test/CMakeFiles/redux_6.dir/redux.cpp.o
test/redux_6: test/CMakeFiles/redux_6.dir/build.make
test/redux_6: test/CMakeFiles/redux_6.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable redux_6"
	cd "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/build/test" && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/redux_6.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/CMakeFiles/redux_6.dir/build: test/redux_6
.PHONY : test/CMakeFiles/redux_6.dir/build

test/CMakeFiles/redux_6.dir/clean:
	cd "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/build/test" && $(CMAKE_COMMAND) -P CMakeFiles/redux_6.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/redux_6.dir/clean

test/CMakeFiles/redux_6.dir/depend:
	cd "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/build" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7" "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/test" "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/build" "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/build/test" "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/build/test/CMakeFiles/redux_6.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : test/CMakeFiles/redux_6.dir/depend

