# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.9

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
CMAKE_COMMAND = /home/destinxxy/Program/clion-2017.3.2/bin/cmake/bin/cmake

# The command to remove a file.
RM = /home/destinxxy/Program/clion-2017.3.2/bin/cmake/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/destinxxy/Documents/RM2018/RMRefiller

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/destinxxy/Documents/RM2018/RMRefiller/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/PLATFORM.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/PLATFORM.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/PLATFORM.dir/flags.make

CMakeFiles/PLATFORM.dir/os/hal/ports/STM32/LLD/USARTv1/serial_lld.c.o: CMakeFiles/PLATFORM.dir/flags.make
CMakeFiles/PLATFORM.dir/os/hal/ports/STM32/LLD/USARTv1/serial_lld.c.o: ../os/hal/ports/STM32/LLD/USARTv1/serial_lld.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/destinxxy/Documents/RM2018/RMRefiller/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/PLATFORM.dir/os/hal/ports/STM32/LLD/USARTv1/serial_lld.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/PLATFORM.dir/os/hal/ports/STM32/LLD/USARTv1/serial_lld.c.o   -c /home/destinxxy/Documents/RM2018/RMRefiller/os/hal/ports/STM32/LLD/USARTv1/serial_lld.c

CMakeFiles/PLATFORM.dir/os/hal/ports/STM32/LLD/USARTv1/serial_lld.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/PLATFORM.dir/os/hal/ports/STM32/LLD/USARTv1/serial_lld.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/destinxxy/Documents/RM2018/RMRefiller/os/hal/ports/STM32/LLD/USARTv1/serial_lld.c > CMakeFiles/PLATFORM.dir/os/hal/ports/STM32/LLD/USARTv1/serial_lld.c.i

CMakeFiles/PLATFORM.dir/os/hal/ports/STM32/LLD/USARTv1/serial_lld.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/PLATFORM.dir/os/hal/ports/STM32/LLD/USARTv1/serial_lld.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/destinxxy/Documents/RM2018/RMRefiller/os/hal/ports/STM32/LLD/USARTv1/serial_lld.c -o CMakeFiles/PLATFORM.dir/os/hal/ports/STM32/LLD/USARTv1/serial_lld.c.s

CMakeFiles/PLATFORM.dir/os/hal/ports/STM32/LLD/USARTv1/serial_lld.c.o.requires:

.PHONY : CMakeFiles/PLATFORM.dir/os/hal/ports/STM32/LLD/USARTv1/serial_lld.c.o.requires

CMakeFiles/PLATFORM.dir/os/hal/ports/STM32/LLD/USARTv1/serial_lld.c.o.provides: CMakeFiles/PLATFORM.dir/os/hal/ports/STM32/LLD/USARTv1/serial_lld.c.o.requires
	$(MAKE) -f CMakeFiles/PLATFORM.dir/build.make CMakeFiles/PLATFORM.dir/os/hal/ports/STM32/LLD/USARTv1/serial_lld.c.o.provides.build
.PHONY : CMakeFiles/PLATFORM.dir/os/hal/ports/STM32/LLD/USARTv1/serial_lld.c.o.provides

CMakeFiles/PLATFORM.dir/os/hal/ports/STM32/LLD/USARTv1/serial_lld.c.o.provides.build: CMakeFiles/PLATFORM.dir/os/hal/ports/STM32/LLD/USARTv1/serial_lld.c.o


# Object files for target PLATFORM
PLATFORM_OBJECTS = \
"CMakeFiles/PLATFORM.dir/os/hal/ports/STM32/LLD/USARTv1/serial_lld.c.o"

# External object files for target PLATFORM
PLATFORM_EXTERNAL_OBJECTS =

libPLATFORM.a: CMakeFiles/PLATFORM.dir/os/hal/ports/STM32/LLD/USARTv1/serial_lld.c.o
libPLATFORM.a: CMakeFiles/PLATFORM.dir/build.make
libPLATFORM.a: CMakeFiles/PLATFORM.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/destinxxy/Documents/RM2018/RMRefiller/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C static library libPLATFORM.a"
	$(CMAKE_COMMAND) -P CMakeFiles/PLATFORM.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/PLATFORM.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/PLATFORM.dir/build: libPLATFORM.a

.PHONY : CMakeFiles/PLATFORM.dir/build

CMakeFiles/PLATFORM.dir/requires: CMakeFiles/PLATFORM.dir/os/hal/ports/STM32/LLD/USARTv1/serial_lld.c.o.requires

.PHONY : CMakeFiles/PLATFORM.dir/requires

CMakeFiles/PLATFORM.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/PLATFORM.dir/cmake_clean.cmake
.PHONY : CMakeFiles/PLATFORM.dir/clean

CMakeFiles/PLATFORM.dir/depend:
	cd /home/destinxxy/Documents/RM2018/RMRefiller/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/destinxxy/Documents/RM2018/RMRefiller /home/destinxxy/Documents/RM2018/RMRefiller /home/destinxxy/Documents/RM2018/RMRefiller/cmake-build-debug /home/destinxxy/Documents/RM2018/RMRefiller/cmake-build-debug /home/destinxxy/Documents/RM2018/RMRefiller/cmake-build-debug/CMakeFiles/PLATFORM.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/PLATFORM.dir/depend

