# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.15

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
CMAKE_COMMAND = /usr/local/Cellar/cmake/3.15.3/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/3.15.3/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/julia/Documents/GeorgeMason/Fall_2019/CS633/HW/pa01_rotating_caliper

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/julia/Documents/GeorgeMason/Fall_2019/CS633/HW/pa01_rotating_caliper/build

# Include any dependencies generated for this target.
include src/CMakeFiles/PA01.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/PA01.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/PA01.dir/flags.make

src/CMakeFiles/PA01.dir/main.cpp.o: src/CMakeFiles/PA01.dir/flags.make
src/CMakeFiles/PA01.dir/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/julia/Documents/GeorgeMason/Fall_2019/CS633/HW/pa01_rotating_caliper/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/PA01.dir/main.cpp.o"
	cd /Users/julia/Documents/GeorgeMason/Fall_2019/CS633/HW/pa01_rotating_caliper/build/src && /Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/PA01.dir/main.cpp.o -c /Users/julia/Documents/GeorgeMason/Fall_2019/CS633/HW/pa01_rotating_caliper/src/main.cpp

src/CMakeFiles/PA01.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/PA01.dir/main.cpp.i"
	cd /Users/julia/Documents/GeorgeMason/Fall_2019/CS633/HW/pa01_rotating_caliper/build/src && /Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/julia/Documents/GeorgeMason/Fall_2019/CS633/HW/pa01_rotating_caliper/src/main.cpp > CMakeFiles/PA01.dir/main.cpp.i

src/CMakeFiles/PA01.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/PA01.dir/main.cpp.s"
	cd /Users/julia/Documents/GeorgeMason/Fall_2019/CS633/HW/pa01_rotating_caliper/build/src && /Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/julia/Documents/GeorgeMason/Fall_2019/CS633/HW/pa01_rotating_caliper/src/main.cpp -o CMakeFiles/PA01.dir/main.cpp.s

# Object files for target PA01
PA01_OBJECTS = \
"CMakeFiles/PA01.dir/main.cpp.o"

# External object files for target PA01
PA01_EXTERNAL_OBJECTS =

PA01: src/CMakeFiles/PA01.dir/main.cpp.o
PA01: src/CMakeFiles/PA01.dir/build.make
PA01: src/mathtool/libmathtool.a
PA01: src/polygon/libpolygon.a
PA01: src/libbbox2dlib.a
PA01: src/mathtool/libmathtool.a
PA01: src/polygon/libpolygon.a
PA01: src/CMakeFiles/PA01.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/julia/Documents/GeorgeMason/Fall_2019/CS633/HW/pa01_rotating_caliper/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../PA01"
	cd /Users/julia/Documents/GeorgeMason/Fall_2019/CS633/HW/pa01_rotating_caliper/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/PA01.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/PA01.dir/build: PA01

.PHONY : src/CMakeFiles/PA01.dir/build

src/CMakeFiles/PA01.dir/clean:
	cd /Users/julia/Documents/GeorgeMason/Fall_2019/CS633/HW/pa01_rotating_caliper/build/src && $(CMAKE_COMMAND) -P CMakeFiles/PA01.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/PA01.dir/clean

src/CMakeFiles/PA01.dir/depend:
	cd /Users/julia/Documents/GeorgeMason/Fall_2019/CS633/HW/pa01_rotating_caliper/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/julia/Documents/GeorgeMason/Fall_2019/CS633/HW/pa01_rotating_caliper /Users/julia/Documents/GeorgeMason/Fall_2019/CS633/HW/pa01_rotating_caliper/src /Users/julia/Documents/GeorgeMason/Fall_2019/CS633/HW/pa01_rotating_caliper/build /Users/julia/Documents/GeorgeMason/Fall_2019/CS633/HW/pa01_rotating_caliper/build/src /Users/julia/Documents/GeorgeMason/Fall_2019/CS633/HW/pa01_rotating_caliper/build/src/CMakeFiles/PA01.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/PA01.dir/depend
