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
include doc/snippets/CMakeFiles/compile_PartialRedux_count.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include doc/snippets/CMakeFiles/compile_PartialRedux_count.dir/compiler_depend.make

# Include the progress variables for this target.
include doc/snippets/CMakeFiles/compile_PartialRedux_count.dir/progress.make

# Include the compile flags for this target's objects.
include doc/snippets/CMakeFiles/compile_PartialRedux_count.dir/flags.make

doc/snippets/CMakeFiles/compile_PartialRedux_count.dir/compile_PartialRedux_count.cpp.o: doc/snippets/CMakeFiles/compile_PartialRedux_count.dir/flags.make
doc/snippets/CMakeFiles/compile_PartialRedux_count.dir/compile_PartialRedux_count.cpp.o: doc/snippets/compile_PartialRedux_count.cpp
doc/snippets/CMakeFiles/compile_PartialRedux_count.dir/compile_PartialRedux_count.cpp.o: ../doc/snippets/PartialRedux_count.cpp
doc/snippets/CMakeFiles/compile_PartialRedux_count.dir/compile_PartialRedux_count.cpp.o: doc/snippets/CMakeFiles/compile_PartialRedux_count.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object doc/snippets/CMakeFiles/compile_PartialRedux_count.dir/compile_PartialRedux_count.cpp.o"
	cd "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/build/doc/snippets" && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT doc/snippets/CMakeFiles/compile_PartialRedux_count.dir/compile_PartialRedux_count.cpp.o -MF CMakeFiles/compile_PartialRedux_count.dir/compile_PartialRedux_count.cpp.o.d -o CMakeFiles/compile_PartialRedux_count.dir/compile_PartialRedux_count.cpp.o -c "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/build/doc/snippets/compile_PartialRedux_count.cpp"

doc/snippets/CMakeFiles/compile_PartialRedux_count.dir/compile_PartialRedux_count.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/compile_PartialRedux_count.dir/compile_PartialRedux_count.cpp.i"
	cd "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/build/doc/snippets" && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/build/doc/snippets/compile_PartialRedux_count.cpp" > CMakeFiles/compile_PartialRedux_count.dir/compile_PartialRedux_count.cpp.i

doc/snippets/CMakeFiles/compile_PartialRedux_count.dir/compile_PartialRedux_count.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/compile_PartialRedux_count.dir/compile_PartialRedux_count.cpp.s"
	cd "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/build/doc/snippets" && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/build/doc/snippets/compile_PartialRedux_count.cpp" -o CMakeFiles/compile_PartialRedux_count.dir/compile_PartialRedux_count.cpp.s

# Object files for target compile_PartialRedux_count
compile_PartialRedux_count_OBJECTS = \
"CMakeFiles/compile_PartialRedux_count.dir/compile_PartialRedux_count.cpp.o"

# External object files for target compile_PartialRedux_count
compile_PartialRedux_count_EXTERNAL_OBJECTS =

doc/snippets/compile_PartialRedux_count: doc/snippets/CMakeFiles/compile_PartialRedux_count.dir/compile_PartialRedux_count.cpp.o
doc/snippets/compile_PartialRedux_count: doc/snippets/CMakeFiles/compile_PartialRedux_count.dir/build.make
doc/snippets/compile_PartialRedux_count: doc/snippets/CMakeFiles/compile_PartialRedux_count.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable compile_PartialRedux_count"
	cd "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/build/doc/snippets" && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/compile_PartialRedux_count.dir/link.txt --verbose=$(VERBOSE)
	cd "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/build/doc/snippets" && ./compile_PartialRedux_count >/home/sukruthichidananda/ROAHM\ LAB/eigen-3.3.7/build/doc/snippets/PartialRedux_count.out

# Rule to build all files generated by this target.
doc/snippets/CMakeFiles/compile_PartialRedux_count.dir/build: doc/snippets/compile_PartialRedux_count
.PHONY : doc/snippets/CMakeFiles/compile_PartialRedux_count.dir/build

doc/snippets/CMakeFiles/compile_PartialRedux_count.dir/clean:
	cd "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/build/doc/snippets" && $(CMAKE_COMMAND) -P CMakeFiles/compile_PartialRedux_count.dir/cmake_clean.cmake
.PHONY : doc/snippets/CMakeFiles/compile_PartialRedux_count.dir/clean

doc/snippets/CMakeFiles/compile_PartialRedux_count.dir/depend:
	cd "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/build" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7" "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/doc/snippets" "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/build" "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/build/doc/snippets" "/home/sukruthichidananda/ROAHM LAB/eigen-3.3.7/build/doc/snippets/CMakeFiles/compile_PartialRedux_count.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : doc/snippets/CMakeFiles/compile_PartialRedux_count.dir/depend

