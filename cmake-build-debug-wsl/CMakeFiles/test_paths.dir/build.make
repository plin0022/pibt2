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
CMAKE_SOURCE_DIR = /mnt/c/Users/lpq66/OneDrive/Desktop/project_test/pibt_test_1

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /mnt/c/Users/lpq66/OneDrive/Desktop/project_test/pibt_test_1/cmake-build-debug-wsl

# Include any dependencies generated for this target.
include CMakeFiles/test_paths.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/test_paths.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/test_paths.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/test_paths.dir/flags.make

CMakeFiles/test_paths.dir/tests/test_paths.cpp.o: CMakeFiles/test_paths.dir/flags.make
CMakeFiles/test_paths.dir/tests/test_paths.cpp.o: ../tests/test_paths.cpp
CMakeFiles/test_paths.dir/tests/test_paths.cpp.o: CMakeFiles/test_paths.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/mnt/c/Users/lpq66/OneDrive/Desktop/project_test/pibt_test_1/cmake-build-debug-wsl/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/test_paths.dir/tests/test_paths.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/test_paths.dir/tests/test_paths.cpp.o -MF CMakeFiles/test_paths.dir/tests/test_paths.cpp.o.d -o CMakeFiles/test_paths.dir/tests/test_paths.cpp.o -c /mnt/c/Users/lpq66/OneDrive/Desktop/project_test/pibt_test_1/tests/test_paths.cpp

CMakeFiles/test_paths.dir/tests/test_paths.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_paths.dir/tests/test_paths.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/c/Users/lpq66/OneDrive/Desktop/project_test/pibt_test_1/tests/test_paths.cpp > CMakeFiles/test_paths.dir/tests/test_paths.cpp.i

CMakeFiles/test_paths.dir/tests/test_paths.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_paths.dir/tests/test_paths.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/c/Users/lpq66/OneDrive/Desktop/project_test/pibt_test_1/tests/test_paths.cpp -o CMakeFiles/test_paths.dir/tests/test_paths.cpp.s

CMakeFiles/test_paths.dir/third_party/googletest/googletest/src/gtest_main.cc.o: CMakeFiles/test_paths.dir/flags.make
CMakeFiles/test_paths.dir/third_party/googletest/googletest/src/gtest_main.cc.o: ../third_party/googletest/googletest/src/gtest_main.cc
CMakeFiles/test_paths.dir/third_party/googletest/googletest/src/gtest_main.cc.o: CMakeFiles/test_paths.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/mnt/c/Users/lpq66/OneDrive/Desktop/project_test/pibt_test_1/cmake-build-debug-wsl/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/test_paths.dir/third_party/googletest/googletest/src/gtest_main.cc.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/test_paths.dir/third_party/googletest/googletest/src/gtest_main.cc.o -MF CMakeFiles/test_paths.dir/third_party/googletest/googletest/src/gtest_main.cc.o.d -o CMakeFiles/test_paths.dir/third_party/googletest/googletest/src/gtest_main.cc.o -c /mnt/c/Users/lpq66/OneDrive/Desktop/project_test/pibt_test_1/third_party/googletest/googletest/src/gtest_main.cc

CMakeFiles/test_paths.dir/third_party/googletest/googletest/src/gtest_main.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_paths.dir/third_party/googletest/googletest/src/gtest_main.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/c/Users/lpq66/OneDrive/Desktop/project_test/pibt_test_1/third_party/googletest/googletest/src/gtest_main.cc > CMakeFiles/test_paths.dir/third_party/googletest/googletest/src/gtest_main.cc.i

CMakeFiles/test_paths.dir/third_party/googletest/googletest/src/gtest_main.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_paths.dir/third_party/googletest/googletest/src/gtest_main.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/c/Users/lpq66/OneDrive/Desktop/project_test/pibt_test_1/third_party/googletest/googletest/src/gtest_main.cc -o CMakeFiles/test_paths.dir/third_party/googletest/googletest/src/gtest_main.cc.s

# Object files for target test_paths
test_paths_OBJECTS = \
"CMakeFiles/test_paths.dir/tests/test_paths.cpp.o" \
"CMakeFiles/test_paths.dir/third_party/googletest/googletest/src/gtest_main.cc.o"

# External object files for target test_paths
test_paths_EXTERNAL_OBJECTS =

test_paths: CMakeFiles/test_paths.dir/tests/test_paths.cpp.o
test_paths: CMakeFiles/test_paths.dir/third_party/googletest/googletest/src/gtest_main.cc.o
test_paths: CMakeFiles/test_paths.dir/build.make
test_paths: pibt2/liblib-mapf.a
test_paths: lib/libgtestd.a
test_paths: pibt2/graph/liblib-graph.a
test_paths: CMakeFiles/test_paths.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/mnt/c/Users/lpq66/OneDrive/Desktop/project_test/pibt_test_1/cmake-build-debug-wsl/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable test_paths"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_paths.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/test_paths.dir/build: test_paths
.PHONY : CMakeFiles/test_paths.dir/build

CMakeFiles/test_paths.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/test_paths.dir/cmake_clean.cmake
.PHONY : CMakeFiles/test_paths.dir/clean

CMakeFiles/test_paths.dir/depend:
	cd /mnt/c/Users/lpq66/OneDrive/Desktop/project_test/pibt_test_1/cmake-build-debug-wsl && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /mnt/c/Users/lpq66/OneDrive/Desktop/project_test/pibt_test_1 /mnt/c/Users/lpq66/OneDrive/Desktop/project_test/pibt_test_1 /mnt/c/Users/lpq66/OneDrive/Desktop/project_test/pibt_test_1/cmake-build-debug-wsl /mnt/c/Users/lpq66/OneDrive/Desktop/project_test/pibt_test_1/cmake-build-debug-wsl /mnt/c/Users/lpq66/OneDrive/Desktop/project_test/pibt_test_1/cmake-build-debug-wsl/CMakeFiles/test_paths.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/test_paths.dir/depend

