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
include liblcm-test/CMakeFiles/lcm-buftest-receiver.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include liblcm-test/CMakeFiles/lcm-buftest-receiver.dir/compiler_depend.make

# Include the progress variables for this target.
include liblcm-test/CMakeFiles/lcm-buftest-receiver.dir/progress.make

# Include the compile flags for this target's objects.
include liblcm-test/CMakeFiles/lcm-buftest-receiver.dir/flags.make

liblcm-test/CMakeFiles/lcm-buftest-receiver.dir/buftest-receiver.c.o: liblcm-test/CMakeFiles/lcm-buftest-receiver.dir/flags.make
liblcm-test/CMakeFiles/lcm-buftest-receiver.dir/buftest-receiver.c.o: ../liblcm-test/buftest-receiver.c
liblcm-test/CMakeFiles/lcm-buftest-receiver.dir/buftest-receiver.c.o: liblcm-test/CMakeFiles/lcm-buftest-receiver.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zakhar/work/SI/lcm-1.4.0/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object liblcm-test/CMakeFiles/lcm-buftest-receiver.dir/buftest-receiver.c.o"
	cd /home/zakhar/work/SI/lcm-1.4.0/build/liblcm-test && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT liblcm-test/CMakeFiles/lcm-buftest-receiver.dir/buftest-receiver.c.o -MF CMakeFiles/lcm-buftest-receiver.dir/buftest-receiver.c.o.d -o CMakeFiles/lcm-buftest-receiver.dir/buftest-receiver.c.o -c /home/zakhar/work/SI/lcm-1.4.0/liblcm-test/buftest-receiver.c

liblcm-test/CMakeFiles/lcm-buftest-receiver.dir/buftest-receiver.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/lcm-buftest-receiver.dir/buftest-receiver.c.i"
	cd /home/zakhar/work/SI/lcm-1.4.0/build/liblcm-test && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/zakhar/work/SI/lcm-1.4.0/liblcm-test/buftest-receiver.c > CMakeFiles/lcm-buftest-receiver.dir/buftest-receiver.c.i

liblcm-test/CMakeFiles/lcm-buftest-receiver.dir/buftest-receiver.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/lcm-buftest-receiver.dir/buftest-receiver.c.s"
	cd /home/zakhar/work/SI/lcm-1.4.0/build/liblcm-test && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/zakhar/work/SI/lcm-1.4.0/liblcm-test/buftest-receiver.c -o CMakeFiles/lcm-buftest-receiver.dir/buftest-receiver.c.s

# Object files for target lcm-buftest-receiver
lcm__buftest__receiver_OBJECTS = \
"CMakeFiles/lcm-buftest-receiver.dir/buftest-receiver.c.o"

# External object files for target lcm-buftest-receiver
lcm__buftest__receiver_EXTERNAL_OBJECTS =

liblcm-test/lcm-buftest-receiver: liblcm-test/CMakeFiles/lcm-buftest-receiver.dir/buftest-receiver.c.o
liblcm-test/lcm-buftest-receiver: liblcm-test/CMakeFiles/lcm-buftest-receiver.dir/build.make
liblcm-test/lcm-buftest-receiver: lcm/liblcm.so.1.4.0
liblcm-test/lcm-buftest-receiver: /usr/lib/x86_64-linux-gnu/libglib-2.0.so
liblcm-test/lcm-buftest-receiver: liblcm-test/CMakeFiles/lcm-buftest-receiver.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zakhar/work/SI/lcm-1.4.0/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C executable lcm-buftest-receiver"
	cd /home/zakhar/work/SI/lcm-1.4.0/build/liblcm-test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/lcm-buftest-receiver.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
liblcm-test/CMakeFiles/lcm-buftest-receiver.dir/build: liblcm-test/lcm-buftest-receiver
.PHONY : liblcm-test/CMakeFiles/lcm-buftest-receiver.dir/build

liblcm-test/CMakeFiles/lcm-buftest-receiver.dir/clean:
	cd /home/zakhar/work/SI/lcm-1.4.0/build/liblcm-test && $(CMAKE_COMMAND) -P CMakeFiles/lcm-buftest-receiver.dir/cmake_clean.cmake
.PHONY : liblcm-test/CMakeFiles/lcm-buftest-receiver.dir/clean

liblcm-test/CMakeFiles/lcm-buftest-receiver.dir/depend:
	cd /home/zakhar/work/SI/lcm-1.4.0/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zakhar/work/SI/lcm-1.4.0 /home/zakhar/work/SI/lcm-1.4.0/liblcm-test /home/zakhar/work/SI/lcm-1.4.0/build /home/zakhar/work/SI/lcm-1.4.0/build/liblcm-test /home/zakhar/work/SI/lcm-1.4.0/build/liblcm-test/CMakeFiles/lcm-buftest-receiver.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : liblcm-test/CMakeFiles/lcm-buftest-receiver.dir/depend

