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
CMAKE_SOURCE_DIR = /home/zakhar/work/SI/lcm-1.4.0

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/zakhar/work/SI/lcm-1.4.0/build

# Include any dependencies generated for this target.
include test/c/CMakeFiles/test-c-server.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include test/c/CMakeFiles/test-c-server.dir/compiler_depend.make

# Include the progress variables for this target.
include test/c/CMakeFiles/test-c-server.dir/progress.make

# Include the compile flags for this target's objects.
include test/c/CMakeFiles/test-c-server.dir/flags.make

test/c/CMakeFiles/test-c-server.dir/server.c.o: test/c/CMakeFiles/test-c-server.dir/flags.make
test/c/CMakeFiles/test-c-server.dir/server.c.o: ../test/c/server.c
test/c/CMakeFiles/test-c-server.dir/server.c.o: test/c/CMakeFiles/test-c-server.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zakhar/work/SI/lcm-1.4.0/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object test/c/CMakeFiles/test-c-server.dir/server.c.o"
	cd /home/zakhar/work/SI/lcm-1.4.0/build/test/c && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT test/c/CMakeFiles/test-c-server.dir/server.c.o -MF CMakeFiles/test-c-server.dir/server.c.o.d -o CMakeFiles/test-c-server.dir/server.c.o -c /home/zakhar/work/SI/lcm-1.4.0/test/c/server.c

test/c/CMakeFiles/test-c-server.dir/server.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/test-c-server.dir/server.c.i"
	cd /home/zakhar/work/SI/lcm-1.4.0/build/test/c && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/zakhar/work/SI/lcm-1.4.0/test/c/server.c > CMakeFiles/test-c-server.dir/server.c.i

test/c/CMakeFiles/test-c-server.dir/server.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/test-c-server.dir/server.c.s"
	cd /home/zakhar/work/SI/lcm-1.4.0/build/test/c && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/zakhar/work/SI/lcm-1.4.0/test/c/server.c -o CMakeFiles/test-c-server.dir/server.c.s

test/c/CMakeFiles/test-c-server.dir/common.c.o: test/c/CMakeFiles/test-c-server.dir/flags.make
test/c/CMakeFiles/test-c-server.dir/common.c.o: ../test/c/common.c
test/c/CMakeFiles/test-c-server.dir/common.c.o: test/c/CMakeFiles/test-c-server.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zakhar/work/SI/lcm-1.4.0/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object test/c/CMakeFiles/test-c-server.dir/common.c.o"
	cd /home/zakhar/work/SI/lcm-1.4.0/build/test/c && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT test/c/CMakeFiles/test-c-server.dir/common.c.o -MF CMakeFiles/test-c-server.dir/common.c.o.d -o CMakeFiles/test-c-server.dir/common.c.o -c /home/zakhar/work/SI/lcm-1.4.0/test/c/common.c

test/c/CMakeFiles/test-c-server.dir/common.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/test-c-server.dir/common.c.i"
	cd /home/zakhar/work/SI/lcm-1.4.0/build/test/c && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/zakhar/work/SI/lcm-1.4.0/test/c/common.c > CMakeFiles/test-c-server.dir/common.c.i

test/c/CMakeFiles/test-c-server.dir/common.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/test-c-server.dir/common.c.s"
	cd /home/zakhar/work/SI/lcm-1.4.0/build/test/c && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/zakhar/work/SI/lcm-1.4.0/test/c/common.c -o CMakeFiles/test-c-server.dir/common.c.s

# Object files for target test-c-server
test__c__server_OBJECTS = \
"CMakeFiles/test-c-server.dir/server.c.o" \
"CMakeFiles/test-c-server.dir/common.c.o"

# External object files for target test-c-server
test__c__server_EXTERNAL_OBJECTS =

test/c/test-c-server: test/c/CMakeFiles/test-c-server.dir/server.c.o
test/c/test-c-server: test/c/CMakeFiles/test-c-server.dir/common.c.o
test/c/test-c-server: test/c/CMakeFiles/test-c-server.dir/build.make
test/c/test-c-server: test/types/liblcm-test-types.so
test/c/test-c-server: lcm/liblcm.so.1.4.0
test/c/test-c-server: test/c/CMakeFiles/test-c-server.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zakhar/work/SI/lcm-1.4.0/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking C executable test-c-server"
	cd /home/zakhar/work/SI/lcm-1.4.0/build/test/c && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test-c-server.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/c/CMakeFiles/test-c-server.dir/build: test/c/test-c-server
.PHONY : test/c/CMakeFiles/test-c-server.dir/build

test/c/CMakeFiles/test-c-server.dir/clean:
	cd /home/zakhar/work/SI/lcm-1.4.0/build/test/c && $(CMAKE_COMMAND) -P CMakeFiles/test-c-server.dir/cmake_clean.cmake
.PHONY : test/c/CMakeFiles/test-c-server.dir/clean

test/c/CMakeFiles/test-c-server.dir/depend:
	cd /home/zakhar/work/SI/lcm-1.4.0/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zakhar/work/SI/lcm-1.4.0 /home/zakhar/work/SI/lcm-1.4.0/test/c /home/zakhar/work/SI/lcm-1.4.0/build /home/zakhar/work/SI/lcm-1.4.0/build/test/c /home/zakhar/work/SI/lcm-1.4.0/build/test/c/CMakeFiles/test-c-server.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/c/CMakeFiles/test-c-server.dir/depend

