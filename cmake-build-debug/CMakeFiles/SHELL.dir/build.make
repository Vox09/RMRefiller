# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.8

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
CMAKE_SOURCE_DIR = /Users/David/Documents/GitHub/Robomaster/Refiller_Clean

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/David/Documents/GitHub/Robomaster/Refiller_Clean/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/SHELL.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/SHELL.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/SHELL.dir/flags.make

CMakeFiles/SHELL.dir/os/various/shell.c.o: CMakeFiles/SHELL.dir/flags.make
CMakeFiles/SHELL.dir/os/various/shell.c.o: ../os/various/shell.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/David/Documents/GitHub/Robomaster/Refiller_Clean/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/SHELL.dir/os/various/shell.c.o"
	/Library/Developer/CommandLineTools/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/SHELL.dir/os/various/shell.c.o   -c /Users/David/Documents/GitHub/Robomaster/Refiller_Clean/os/various/shell.c

CMakeFiles/SHELL.dir/os/various/shell.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/SHELL.dir/os/various/shell.c.i"
	/Library/Developer/CommandLineTools/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /Users/David/Documents/GitHub/Robomaster/Refiller_Clean/os/various/shell.c > CMakeFiles/SHELL.dir/os/various/shell.c.i

CMakeFiles/SHELL.dir/os/various/shell.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/SHELL.dir/os/various/shell.c.s"
	/Library/Developer/CommandLineTools/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /Users/David/Documents/GitHub/Robomaster/Refiller_Clean/os/various/shell.c -o CMakeFiles/SHELL.dir/os/various/shell.c.s

CMakeFiles/SHELL.dir/os/various/shell.c.o.requires:

.PHONY : CMakeFiles/SHELL.dir/os/various/shell.c.o.requires

CMakeFiles/SHELL.dir/os/various/shell.c.o.provides: CMakeFiles/SHELL.dir/os/various/shell.c.o.requires
	$(MAKE) -f CMakeFiles/SHELL.dir/build.make CMakeFiles/SHELL.dir/os/various/shell.c.o.provides.build
.PHONY : CMakeFiles/SHELL.dir/os/various/shell.c.o.provides

CMakeFiles/SHELL.dir/os/various/shell.c.o.provides.build: CMakeFiles/SHELL.dir/os/various/shell.c.o


CMakeFiles/SHELL.dir/os/various/evtimer.c.o: CMakeFiles/SHELL.dir/flags.make
CMakeFiles/SHELL.dir/os/various/evtimer.c.o: ../os/various/evtimer.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/David/Documents/GitHub/Robomaster/Refiller_Clean/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object CMakeFiles/SHELL.dir/os/various/evtimer.c.o"
	/Library/Developer/CommandLineTools/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/SHELL.dir/os/various/evtimer.c.o   -c /Users/David/Documents/GitHub/Robomaster/Refiller_Clean/os/various/evtimer.c

CMakeFiles/SHELL.dir/os/various/evtimer.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/SHELL.dir/os/various/evtimer.c.i"
	/Library/Developer/CommandLineTools/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /Users/David/Documents/GitHub/Robomaster/Refiller_Clean/os/various/evtimer.c > CMakeFiles/SHELL.dir/os/various/evtimer.c.i

CMakeFiles/SHELL.dir/os/various/evtimer.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/SHELL.dir/os/various/evtimer.c.s"
	/Library/Developer/CommandLineTools/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /Users/David/Documents/GitHub/Robomaster/Refiller_Clean/os/various/evtimer.c -o CMakeFiles/SHELL.dir/os/various/evtimer.c.s

CMakeFiles/SHELL.dir/os/various/evtimer.c.o.requires:

.PHONY : CMakeFiles/SHELL.dir/os/various/evtimer.c.o.requires

CMakeFiles/SHELL.dir/os/various/evtimer.c.o.provides: CMakeFiles/SHELL.dir/os/various/evtimer.c.o.requires
	$(MAKE) -f CMakeFiles/SHELL.dir/build.make CMakeFiles/SHELL.dir/os/various/evtimer.c.o.provides.build
.PHONY : CMakeFiles/SHELL.dir/os/various/evtimer.c.o.provides

CMakeFiles/SHELL.dir/os/various/evtimer.c.o.provides.build: CMakeFiles/SHELL.dir/os/various/evtimer.c.o


CMakeFiles/SHELL.dir/os/various/syscalls.c.o: CMakeFiles/SHELL.dir/flags.make
CMakeFiles/SHELL.dir/os/various/syscalls.c.o: ../os/various/syscalls.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/David/Documents/GitHub/Robomaster/Refiller_Clean/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object CMakeFiles/SHELL.dir/os/various/syscalls.c.o"
	/Library/Developer/CommandLineTools/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/SHELL.dir/os/various/syscalls.c.o   -c /Users/David/Documents/GitHub/Robomaster/Refiller_Clean/os/various/syscalls.c

CMakeFiles/SHELL.dir/os/various/syscalls.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/SHELL.dir/os/various/syscalls.c.i"
	/Library/Developer/CommandLineTools/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /Users/David/Documents/GitHub/Robomaster/Refiller_Clean/os/various/syscalls.c > CMakeFiles/SHELL.dir/os/various/syscalls.c.i

CMakeFiles/SHELL.dir/os/various/syscalls.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/SHELL.dir/os/various/syscalls.c.s"
	/Library/Developer/CommandLineTools/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /Users/David/Documents/GitHub/Robomaster/Refiller_Clean/os/various/syscalls.c -o CMakeFiles/SHELL.dir/os/various/syscalls.c.s

CMakeFiles/SHELL.dir/os/various/syscalls.c.o.requires:

.PHONY : CMakeFiles/SHELL.dir/os/various/syscalls.c.o.requires

CMakeFiles/SHELL.dir/os/various/syscalls.c.o.provides: CMakeFiles/SHELL.dir/os/various/syscalls.c.o.requires
	$(MAKE) -f CMakeFiles/SHELL.dir/build.make CMakeFiles/SHELL.dir/os/various/syscalls.c.o.provides.build
.PHONY : CMakeFiles/SHELL.dir/os/various/syscalls.c.o.provides

CMakeFiles/SHELL.dir/os/various/syscalls.c.o.provides.build: CMakeFiles/SHELL.dir/os/various/syscalls.c.o


CMakeFiles/SHELL.dir/os/hal/lib/streams/memstreams.c.o: CMakeFiles/SHELL.dir/flags.make
CMakeFiles/SHELL.dir/os/hal/lib/streams/memstreams.c.o: ../os/hal/lib/streams/memstreams.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/David/Documents/GitHub/Robomaster/Refiller_Clean/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object CMakeFiles/SHELL.dir/os/hal/lib/streams/memstreams.c.o"
	/Library/Developer/CommandLineTools/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/SHELL.dir/os/hal/lib/streams/memstreams.c.o   -c /Users/David/Documents/GitHub/Robomaster/Refiller_Clean/os/hal/lib/streams/memstreams.c

CMakeFiles/SHELL.dir/os/hal/lib/streams/memstreams.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/SHELL.dir/os/hal/lib/streams/memstreams.c.i"
	/Library/Developer/CommandLineTools/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /Users/David/Documents/GitHub/Robomaster/Refiller_Clean/os/hal/lib/streams/memstreams.c > CMakeFiles/SHELL.dir/os/hal/lib/streams/memstreams.c.i

CMakeFiles/SHELL.dir/os/hal/lib/streams/memstreams.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/SHELL.dir/os/hal/lib/streams/memstreams.c.s"
	/Library/Developer/CommandLineTools/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /Users/David/Documents/GitHub/Robomaster/Refiller_Clean/os/hal/lib/streams/memstreams.c -o CMakeFiles/SHELL.dir/os/hal/lib/streams/memstreams.c.s

CMakeFiles/SHELL.dir/os/hal/lib/streams/memstreams.c.o.requires:

.PHONY : CMakeFiles/SHELL.dir/os/hal/lib/streams/memstreams.c.o.requires

CMakeFiles/SHELL.dir/os/hal/lib/streams/memstreams.c.o.provides: CMakeFiles/SHELL.dir/os/hal/lib/streams/memstreams.c.o.requires
	$(MAKE) -f CMakeFiles/SHELL.dir/build.make CMakeFiles/SHELL.dir/os/hal/lib/streams/memstreams.c.o.provides.build
.PHONY : CMakeFiles/SHELL.dir/os/hal/lib/streams/memstreams.c.o.provides

CMakeFiles/SHELL.dir/os/hal/lib/streams/memstreams.c.o.provides.build: CMakeFiles/SHELL.dir/os/hal/lib/streams/memstreams.c.o


CMakeFiles/SHELL.dir/os/hal/lib/streams/chprintf.c.o: CMakeFiles/SHELL.dir/flags.make
CMakeFiles/SHELL.dir/os/hal/lib/streams/chprintf.c.o: ../os/hal/lib/streams/chprintf.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/David/Documents/GitHub/Robomaster/Refiller_Clean/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building C object CMakeFiles/SHELL.dir/os/hal/lib/streams/chprintf.c.o"
	/Library/Developer/CommandLineTools/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/SHELL.dir/os/hal/lib/streams/chprintf.c.o   -c /Users/David/Documents/GitHub/Robomaster/Refiller_Clean/os/hal/lib/streams/chprintf.c

CMakeFiles/SHELL.dir/os/hal/lib/streams/chprintf.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/SHELL.dir/os/hal/lib/streams/chprintf.c.i"
	/Library/Developer/CommandLineTools/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /Users/David/Documents/GitHub/Robomaster/Refiller_Clean/os/hal/lib/streams/chprintf.c > CMakeFiles/SHELL.dir/os/hal/lib/streams/chprintf.c.i

CMakeFiles/SHELL.dir/os/hal/lib/streams/chprintf.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/SHELL.dir/os/hal/lib/streams/chprintf.c.s"
	/Library/Developer/CommandLineTools/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /Users/David/Documents/GitHub/Robomaster/Refiller_Clean/os/hal/lib/streams/chprintf.c -o CMakeFiles/SHELL.dir/os/hal/lib/streams/chprintf.c.s

CMakeFiles/SHELL.dir/os/hal/lib/streams/chprintf.c.o.requires:

.PHONY : CMakeFiles/SHELL.dir/os/hal/lib/streams/chprintf.c.o.requires

CMakeFiles/SHELL.dir/os/hal/lib/streams/chprintf.c.o.provides: CMakeFiles/SHELL.dir/os/hal/lib/streams/chprintf.c.o.requires
	$(MAKE) -f CMakeFiles/SHELL.dir/build.make CMakeFiles/SHELL.dir/os/hal/lib/streams/chprintf.c.o.provides.build
.PHONY : CMakeFiles/SHELL.dir/os/hal/lib/streams/chprintf.c.o.provides

CMakeFiles/SHELL.dir/os/hal/lib/streams/chprintf.c.o.provides.build: CMakeFiles/SHELL.dir/os/hal/lib/streams/chprintf.c.o


# Object files for target SHELL
SHELL_OBJECTS = \
"CMakeFiles/SHELL.dir/os/various/shell.c.o" \
"CMakeFiles/SHELL.dir/os/various/evtimer.c.o" \
"CMakeFiles/SHELL.dir/os/various/syscalls.c.o" \
"CMakeFiles/SHELL.dir/os/hal/lib/streams/memstreams.c.o" \
"CMakeFiles/SHELL.dir/os/hal/lib/streams/chprintf.c.o"

# External object files for target SHELL
SHELL_EXTERNAL_OBJECTS =

libSHELL.a: CMakeFiles/SHELL.dir/os/various/shell.c.o
libSHELL.a: CMakeFiles/SHELL.dir/os/various/evtimer.c.o
libSHELL.a: CMakeFiles/SHELL.dir/os/various/syscalls.c.o
libSHELL.a: CMakeFiles/SHELL.dir/os/hal/lib/streams/memstreams.c.o
libSHELL.a: CMakeFiles/SHELL.dir/os/hal/lib/streams/chprintf.c.o
libSHELL.a: CMakeFiles/SHELL.dir/build.make
libSHELL.a: CMakeFiles/SHELL.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/David/Documents/GitHub/Robomaster/Refiller_Clean/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking C static library libSHELL.a"
	$(CMAKE_COMMAND) -P CMakeFiles/SHELL.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/SHELL.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/SHELL.dir/build: libSHELL.a

.PHONY : CMakeFiles/SHELL.dir/build

CMakeFiles/SHELL.dir/requires: CMakeFiles/SHELL.dir/os/various/shell.c.o.requires
CMakeFiles/SHELL.dir/requires: CMakeFiles/SHELL.dir/os/various/evtimer.c.o.requires
CMakeFiles/SHELL.dir/requires: CMakeFiles/SHELL.dir/os/various/syscalls.c.o.requires
CMakeFiles/SHELL.dir/requires: CMakeFiles/SHELL.dir/os/hal/lib/streams/memstreams.c.o.requires
CMakeFiles/SHELL.dir/requires: CMakeFiles/SHELL.dir/os/hal/lib/streams/chprintf.c.o.requires

.PHONY : CMakeFiles/SHELL.dir/requires

CMakeFiles/SHELL.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/SHELL.dir/cmake_clean.cmake
.PHONY : CMakeFiles/SHELL.dir/clean

CMakeFiles/SHELL.dir/depend:
	cd /Users/David/Documents/GitHub/Robomaster/Refiller_Clean/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/David/Documents/GitHub/Robomaster/Refiller_Clean /Users/David/Documents/GitHub/Robomaster/Refiller_Clean /Users/David/Documents/GitHub/Robomaster/Refiller_Clean/cmake-build-debug /Users/David/Documents/GitHub/Robomaster/Refiller_Clean/cmake-build-debug /Users/David/Documents/GitHub/Robomaster/Refiller_Clean/cmake-build-debug/CMakeFiles/SHELL.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/SHELL.dir/depend

