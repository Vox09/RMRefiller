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

# Utility rule file for STLink.

# Include the progress variables for this target.
include CMakeFiles/STLink.dir/progress.make

CMakeFiles/STLink:
	make upload -C /home/destinxxy/Documents/RM2018/RMRefiller/dev CLION_EXE_DIR=/home/destinxxy/Documents/RM2018/RMRefiller/cmake-build-debug

STLink: CMakeFiles/STLink
STLink: CMakeFiles/STLink.dir/build.make

.PHONY : STLink

# Rule to build all files generated by this target.
CMakeFiles/STLink.dir/build: STLink

.PHONY : CMakeFiles/STLink.dir/build

CMakeFiles/STLink.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/STLink.dir/cmake_clean.cmake
.PHONY : CMakeFiles/STLink.dir/clean

CMakeFiles/STLink.dir/depend:
	cd /home/destinxxy/Documents/RM2018/RMRefiller/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/destinxxy/Documents/RM2018/RMRefiller /home/destinxxy/Documents/RM2018/RMRefiller /home/destinxxy/Documents/RM2018/RMRefiller/cmake-build-debug /home/destinxxy/Documents/RM2018/RMRefiller/cmake-build-debug /home/destinxxy/Documents/RM2018/RMRefiller/cmake-build-debug/CMakeFiles/STLink.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/STLink.dir/depend

