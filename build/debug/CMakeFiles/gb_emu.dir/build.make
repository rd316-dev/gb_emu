# CMAKE generated file: DO NOT EDIT!
# Generated by "MinGW Makefiles" Generator, CMake Version 3.23

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

SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = C:\Files\cmake-3.23.2-windows-x86_64\bin\cmake.exe

# The command to remove a file.
RM = C:\Files\cmake-3.23.2-windows-x86_64\bin\cmake.exe -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = C:\Projects\gb_emu

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = C:\Projects\gb_emu\build\debug

# Include any dependencies generated for this target.
include CMakeFiles/gb_emu.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/gb_emu.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/gb_emu.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/gb_emu.dir/flags.make

CMakeFiles/gb_emu.dir/src/main.cpp.obj: CMakeFiles/gb_emu.dir/flags.make
CMakeFiles/gb_emu.dir/src/main.cpp.obj: CMakeFiles/gb_emu.dir/includes_CXX.rsp
CMakeFiles/gb_emu.dir/src/main.cpp.obj: ../../src/main.cpp
CMakeFiles/gb_emu.dir/src/main.cpp.obj: CMakeFiles/gb_emu.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\Projects\gb_emu\build\debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/gb_emu.dir/src/main.cpp.obj"
	C:\Files\Qt\Tools\mingw810_64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/gb_emu.dir/src/main.cpp.obj -MF CMakeFiles\gb_emu.dir\src\main.cpp.obj.d -o CMakeFiles\gb_emu.dir\src\main.cpp.obj -c C:\Projects\gb_emu\src\main.cpp

CMakeFiles/gb_emu.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gb_emu.dir/src/main.cpp.i"
	C:\Files\Qt\Tools\mingw810_64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\Projects\gb_emu\src\main.cpp > CMakeFiles\gb_emu.dir\src\main.cpp.i

CMakeFiles/gb_emu.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gb_emu.dir/src/main.cpp.s"
	C:\Files\Qt\Tools\mingw810_64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\Projects\gb_emu\src\main.cpp -o CMakeFiles\gb_emu.dir\src\main.cpp.s

CMakeFiles/gb_emu.dir/src/gameboyemulator.cpp.obj: CMakeFiles/gb_emu.dir/flags.make
CMakeFiles/gb_emu.dir/src/gameboyemulator.cpp.obj: CMakeFiles/gb_emu.dir/includes_CXX.rsp
CMakeFiles/gb_emu.dir/src/gameboyemulator.cpp.obj: ../../src/gameboyemulator.cpp
CMakeFiles/gb_emu.dir/src/gameboyemulator.cpp.obj: CMakeFiles/gb_emu.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\Projects\gb_emu\build\debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/gb_emu.dir/src/gameboyemulator.cpp.obj"
	C:\Files\Qt\Tools\mingw810_64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/gb_emu.dir/src/gameboyemulator.cpp.obj -MF CMakeFiles\gb_emu.dir\src\gameboyemulator.cpp.obj.d -o CMakeFiles\gb_emu.dir\src\gameboyemulator.cpp.obj -c C:\Projects\gb_emu\src\gameboyemulator.cpp

CMakeFiles/gb_emu.dir/src/gameboyemulator.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gb_emu.dir/src/gameboyemulator.cpp.i"
	C:\Files\Qt\Tools\mingw810_64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\Projects\gb_emu\src\gameboyemulator.cpp > CMakeFiles\gb_emu.dir\src\gameboyemulator.cpp.i

CMakeFiles/gb_emu.dir/src/gameboyemulator.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gb_emu.dir/src/gameboyemulator.cpp.s"
	C:\Files\Qt\Tools\mingw810_64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\Projects\gb_emu\src\gameboyemulator.cpp -o CMakeFiles\gb_emu.dir\src\gameboyemulator.cpp.s

CMakeFiles/gb_emu.dir/src/imgui_impl_sdlrenderer.cpp.obj: CMakeFiles/gb_emu.dir/flags.make
CMakeFiles/gb_emu.dir/src/imgui_impl_sdlrenderer.cpp.obj: CMakeFiles/gb_emu.dir/includes_CXX.rsp
CMakeFiles/gb_emu.dir/src/imgui_impl_sdlrenderer.cpp.obj: ../../src/imgui_impl_sdlrenderer.cpp
CMakeFiles/gb_emu.dir/src/imgui_impl_sdlrenderer.cpp.obj: CMakeFiles/gb_emu.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\Projects\gb_emu\build\debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/gb_emu.dir/src/imgui_impl_sdlrenderer.cpp.obj"
	C:\Files\Qt\Tools\mingw810_64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/gb_emu.dir/src/imgui_impl_sdlrenderer.cpp.obj -MF CMakeFiles\gb_emu.dir\src\imgui_impl_sdlrenderer.cpp.obj.d -o CMakeFiles\gb_emu.dir\src\imgui_impl_sdlrenderer.cpp.obj -c C:\Projects\gb_emu\src\imgui_impl_sdlrenderer.cpp

CMakeFiles/gb_emu.dir/src/imgui_impl_sdlrenderer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gb_emu.dir/src/imgui_impl_sdlrenderer.cpp.i"
	C:\Files\Qt\Tools\mingw810_64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\Projects\gb_emu\src\imgui_impl_sdlrenderer.cpp > CMakeFiles\gb_emu.dir\src\imgui_impl_sdlrenderer.cpp.i

CMakeFiles/gb_emu.dir/src/imgui_impl_sdlrenderer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gb_emu.dir/src/imgui_impl_sdlrenderer.cpp.s"
	C:\Files\Qt\Tools\mingw810_64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\Projects\gb_emu\src\imgui_impl_sdlrenderer.cpp -o CMakeFiles\gb_emu.dir\src\imgui_impl_sdlrenderer.cpp.s

CMakeFiles/gb_emu.dir/src/imgui_impl_sdl.cpp.obj: CMakeFiles/gb_emu.dir/flags.make
CMakeFiles/gb_emu.dir/src/imgui_impl_sdl.cpp.obj: CMakeFiles/gb_emu.dir/includes_CXX.rsp
CMakeFiles/gb_emu.dir/src/imgui_impl_sdl.cpp.obj: ../../src/imgui_impl_sdl.cpp
CMakeFiles/gb_emu.dir/src/imgui_impl_sdl.cpp.obj: CMakeFiles/gb_emu.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\Projects\gb_emu\build\debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/gb_emu.dir/src/imgui_impl_sdl.cpp.obj"
	C:\Files\Qt\Tools\mingw810_64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/gb_emu.dir/src/imgui_impl_sdl.cpp.obj -MF CMakeFiles\gb_emu.dir\src\imgui_impl_sdl.cpp.obj.d -o CMakeFiles\gb_emu.dir\src\imgui_impl_sdl.cpp.obj -c C:\Projects\gb_emu\src\imgui_impl_sdl.cpp

CMakeFiles/gb_emu.dir/src/imgui_impl_sdl.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gb_emu.dir/src/imgui_impl_sdl.cpp.i"
	C:\Files\Qt\Tools\mingw810_64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\Projects\gb_emu\src\imgui_impl_sdl.cpp > CMakeFiles\gb_emu.dir\src\imgui_impl_sdl.cpp.i

CMakeFiles/gb_emu.dir/src/imgui_impl_sdl.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gb_emu.dir/src/imgui_impl_sdl.cpp.s"
	C:\Files\Qt\Tools\mingw810_64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\Projects\gb_emu\src\imgui_impl_sdl.cpp -o CMakeFiles\gb_emu.dir\src\imgui_impl_sdl.cpp.s

# Object files for target gb_emu
gb_emu_OBJECTS = \
"CMakeFiles/gb_emu.dir/src/main.cpp.obj" \
"CMakeFiles/gb_emu.dir/src/gameboyemulator.cpp.obj" \
"CMakeFiles/gb_emu.dir/src/imgui_impl_sdlrenderer.cpp.obj" \
"CMakeFiles/gb_emu.dir/src/imgui_impl_sdl.cpp.obj"

# External object files for target gb_emu
gb_emu_EXTERNAL_OBJECTS =

gb_emu.exe: CMakeFiles/gb_emu.dir/src/main.cpp.obj
gb_emu.exe: CMakeFiles/gb_emu.dir/src/gameboyemulator.cpp.obj
gb_emu.exe: CMakeFiles/gb_emu.dir/src/imgui_impl_sdlrenderer.cpp.obj
gb_emu.exe: CMakeFiles/gb_emu.dir/src/imgui_impl_sdl.cpp.obj
gb_emu.exe: CMakeFiles/gb_emu.dir/build.make
gb_emu.exe: ../../imgui/imgui.cpp
gb_emu.exe: ../../imgui/imgui_demo.cpp
gb_emu.exe: ../../imgui/imgui_draw.cpp
gb_emu.exe: ../../imgui/imgui_tables.cpp
gb_emu.exe: ../../imgui/imgui_widgets.cpp
gb_emu.exe: CMakeFiles/gb_emu.dir/linklibs.rsp
gb_emu.exe: CMakeFiles/gb_emu.dir/objects1.rsp
gb_emu.exe: CMakeFiles/gb_emu.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=C:\Projects\gb_emu\build\debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX executable gb_emu.exe"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\gb_emu.dir\link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/gb_emu.dir/build: gb_emu.exe
.PHONY : CMakeFiles/gb_emu.dir/build

CMakeFiles/gb_emu.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles\gb_emu.dir\cmake_clean.cmake
.PHONY : CMakeFiles/gb_emu.dir/clean

CMakeFiles/gb_emu.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" C:\Projects\gb_emu C:\Projects\gb_emu C:\Projects\gb_emu\build\debug C:\Projects\gb_emu\build\debug C:\Projects\gb_emu\build\debug\CMakeFiles\gb_emu.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/gb_emu.dir/depend

