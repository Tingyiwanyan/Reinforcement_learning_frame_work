# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/tingyi/Research_Frame_work

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/tingyi/Research_Frame_work/build

# Include any dependencies generated for this target.
include CMakeFiles/rtdp.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/rtdp.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/rtdp.dir/flags.make

CMakeFiles/rtdp.dir/src/Hello.cpp.o: CMakeFiles/rtdp.dir/flags.make
CMakeFiles/rtdp.dir/src/Hello.cpp.o: ../src/Hello.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tingyi/Research_Frame_work/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/rtdp.dir/src/Hello.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/rtdp.dir/src/Hello.cpp.o -c /home/tingyi/Research_Frame_work/src/Hello.cpp

CMakeFiles/rtdp.dir/src/Hello.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rtdp.dir/src/Hello.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tingyi/Research_Frame_work/src/Hello.cpp > CMakeFiles/rtdp.dir/src/Hello.cpp.i

CMakeFiles/rtdp.dir/src/Hello.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rtdp.dir/src/Hello.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tingyi/Research_Frame_work/src/Hello.cpp -o CMakeFiles/rtdp.dir/src/Hello.cpp.s

CMakeFiles/rtdp.dir/src/Hello.cpp.o.requires:

.PHONY : CMakeFiles/rtdp.dir/src/Hello.cpp.o.requires

CMakeFiles/rtdp.dir/src/Hello.cpp.o.provides: CMakeFiles/rtdp.dir/src/Hello.cpp.o.requires
	$(MAKE) -f CMakeFiles/rtdp.dir/build.make CMakeFiles/rtdp.dir/src/Hello.cpp.o.provides.build
.PHONY : CMakeFiles/rtdp.dir/src/Hello.cpp.o.provides

CMakeFiles/rtdp.dir/src/Hello.cpp.o.provides.build: CMakeFiles/rtdp.dir/src/Hello.cpp.o


CMakeFiles/rtdp.dir/src/main.cpp.o: CMakeFiles/rtdp.dir/flags.make
CMakeFiles/rtdp.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tingyi/Research_Frame_work/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/rtdp.dir/src/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/rtdp.dir/src/main.cpp.o -c /home/tingyi/Research_Frame_work/src/main.cpp

CMakeFiles/rtdp.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rtdp.dir/src/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tingyi/Research_Frame_work/src/main.cpp > CMakeFiles/rtdp.dir/src/main.cpp.i

CMakeFiles/rtdp.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rtdp.dir/src/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tingyi/Research_Frame_work/src/main.cpp -o CMakeFiles/rtdp.dir/src/main.cpp.s

CMakeFiles/rtdp.dir/src/main.cpp.o.requires:

.PHONY : CMakeFiles/rtdp.dir/src/main.cpp.o.requires

CMakeFiles/rtdp.dir/src/main.cpp.o.provides: CMakeFiles/rtdp.dir/src/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/rtdp.dir/build.make CMakeFiles/rtdp.dir/src/main.cpp.o.provides.build
.PHONY : CMakeFiles/rtdp.dir/src/main.cpp.o.provides

CMakeFiles/rtdp.dir/src/main.cpp.o.provides.build: CMakeFiles/rtdp.dir/src/main.cpp.o


CMakeFiles/rtdp.dir/src/rtdp.cpp.o: CMakeFiles/rtdp.dir/flags.make
CMakeFiles/rtdp.dir/src/rtdp.cpp.o: ../src/rtdp.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tingyi/Research_Frame_work/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/rtdp.dir/src/rtdp.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/rtdp.dir/src/rtdp.cpp.o -c /home/tingyi/Research_Frame_work/src/rtdp.cpp

CMakeFiles/rtdp.dir/src/rtdp.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rtdp.dir/src/rtdp.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tingyi/Research_Frame_work/src/rtdp.cpp > CMakeFiles/rtdp.dir/src/rtdp.cpp.i

CMakeFiles/rtdp.dir/src/rtdp.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rtdp.dir/src/rtdp.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tingyi/Research_Frame_work/src/rtdp.cpp -o CMakeFiles/rtdp.dir/src/rtdp.cpp.s

CMakeFiles/rtdp.dir/src/rtdp.cpp.o.requires:

.PHONY : CMakeFiles/rtdp.dir/src/rtdp.cpp.o.requires

CMakeFiles/rtdp.dir/src/rtdp.cpp.o.provides: CMakeFiles/rtdp.dir/src/rtdp.cpp.o.requires
	$(MAKE) -f CMakeFiles/rtdp.dir/build.make CMakeFiles/rtdp.dir/src/rtdp.cpp.o.provides.build
.PHONY : CMakeFiles/rtdp.dir/src/rtdp.cpp.o.provides

CMakeFiles/rtdp.dir/src/rtdp.cpp.o.provides.build: CMakeFiles/rtdp.dir/src/rtdp.cpp.o


CMakeFiles/rtdp.dir/src/ssp.cpp.o: CMakeFiles/rtdp.dir/flags.make
CMakeFiles/rtdp.dir/src/ssp.cpp.o: ../src/ssp.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tingyi/Research_Frame_work/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/rtdp.dir/src/ssp.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/rtdp.dir/src/ssp.cpp.o -c /home/tingyi/Research_Frame_work/src/ssp.cpp

CMakeFiles/rtdp.dir/src/ssp.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rtdp.dir/src/ssp.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tingyi/Research_Frame_work/src/ssp.cpp > CMakeFiles/rtdp.dir/src/ssp.cpp.i

CMakeFiles/rtdp.dir/src/ssp.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rtdp.dir/src/ssp.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tingyi/Research_Frame_work/src/ssp.cpp -o CMakeFiles/rtdp.dir/src/ssp.cpp.s

CMakeFiles/rtdp.dir/src/ssp.cpp.o.requires:

.PHONY : CMakeFiles/rtdp.dir/src/ssp.cpp.o.requires

CMakeFiles/rtdp.dir/src/ssp.cpp.o.provides: CMakeFiles/rtdp.dir/src/ssp.cpp.o.requires
	$(MAKE) -f CMakeFiles/rtdp.dir/build.make CMakeFiles/rtdp.dir/src/ssp.cpp.o.provides.build
.PHONY : CMakeFiles/rtdp.dir/src/ssp.cpp.o.provides

CMakeFiles/rtdp.dir/src/ssp.cpp.o.provides.build: CMakeFiles/rtdp.dir/src/ssp.cpp.o


# Object files for target rtdp
rtdp_OBJECTS = \
"CMakeFiles/rtdp.dir/src/Hello.cpp.o" \
"CMakeFiles/rtdp.dir/src/main.cpp.o" \
"CMakeFiles/rtdp.dir/src/rtdp.cpp.o" \
"CMakeFiles/rtdp.dir/src/ssp.cpp.o"

# External object files for target rtdp
rtdp_EXTERNAL_OBJECTS =

rtdp: CMakeFiles/rtdp.dir/src/Hello.cpp.o
rtdp: CMakeFiles/rtdp.dir/src/main.cpp.o
rtdp: CMakeFiles/rtdp.dir/src/rtdp.cpp.o
rtdp: CMakeFiles/rtdp.dir/src/ssp.cpp.o
rtdp: CMakeFiles/rtdp.dir/build.make
rtdp: CMakeFiles/rtdp.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/tingyi/Research_Frame_work/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX executable rtdp"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/rtdp.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/rtdp.dir/build: rtdp

.PHONY : CMakeFiles/rtdp.dir/build

CMakeFiles/rtdp.dir/requires: CMakeFiles/rtdp.dir/src/Hello.cpp.o.requires
CMakeFiles/rtdp.dir/requires: CMakeFiles/rtdp.dir/src/main.cpp.o.requires
CMakeFiles/rtdp.dir/requires: CMakeFiles/rtdp.dir/src/rtdp.cpp.o.requires
CMakeFiles/rtdp.dir/requires: CMakeFiles/rtdp.dir/src/ssp.cpp.o.requires

.PHONY : CMakeFiles/rtdp.dir/requires

CMakeFiles/rtdp.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/rtdp.dir/cmake_clean.cmake
.PHONY : CMakeFiles/rtdp.dir/clean

CMakeFiles/rtdp.dir/depend:
	cd /home/tingyi/Research_Frame_work/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/tingyi/Research_Frame_work /home/tingyi/Research_Frame_work /home/tingyi/Research_Frame_work/build /home/tingyi/Research_Frame_work/build /home/tingyi/Research_Frame_work/build/CMakeFiles/rtdp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/rtdp.dir/depend
