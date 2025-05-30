# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /unitree/module/graph_pid_ws/src/HesaiLidar_General_ROS-ROS2

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /unitree/module/graph_pid_ws/build/hesai_lidar

# Include any dependencies generated for this target.
include CMakeFiles/PandarGeneralSDK.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/PandarGeneralSDK.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/PandarGeneralSDK.dir/flags.make

CMakeFiles/PandarGeneralSDK.dir/src/HesaiLidar_General_SDK/src/pandarGeneral_sdk.cc.o: CMakeFiles/PandarGeneralSDK.dir/flags.make
CMakeFiles/PandarGeneralSDK.dir/src/HesaiLidar_General_SDK/src/pandarGeneral_sdk.cc.o: /unitree/module/graph_pid_ws/src/HesaiLidar_General_ROS-ROS2/src/HesaiLidar_General_SDK/src/pandarGeneral_sdk.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/unitree/module/graph_pid_ws/build/hesai_lidar/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/PandarGeneralSDK.dir/src/HesaiLidar_General_SDK/src/pandarGeneral_sdk.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/PandarGeneralSDK.dir/src/HesaiLidar_General_SDK/src/pandarGeneral_sdk.cc.o -c /unitree/module/graph_pid_ws/src/HesaiLidar_General_ROS-ROS2/src/HesaiLidar_General_SDK/src/pandarGeneral_sdk.cc

CMakeFiles/PandarGeneralSDK.dir/src/HesaiLidar_General_SDK/src/pandarGeneral_sdk.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/PandarGeneralSDK.dir/src/HesaiLidar_General_SDK/src/pandarGeneral_sdk.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /unitree/module/graph_pid_ws/src/HesaiLidar_General_ROS-ROS2/src/HesaiLidar_General_SDK/src/pandarGeneral_sdk.cc > CMakeFiles/PandarGeneralSDK.dir/src/HesaiLidar_General_SDK/src/pandarGeneral_sdk.cc.i

CMakeFiles/PandarGeneralSDK.dir/src/HesaiLidar_General_SDK/src/pandarGeneral_sdk.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/PandarGeneralSDK.dir/src/HesaiLidar_General_SDK/src/pandarGeneral_sdk.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /unitree/module/graph_pid_ws/src/HesaiLidar_General_ROS-ROS2/src/HesaiLidar_General_SDK/src/pandarGeneral_sdk.cc -o CMakeFiles/PandarGeneralSDK.dir/src/HesaiLidar_General_SDK/src/pandarGeneral_sdk.cc.s

CMakeFiles/PandarGeneralSDK.dir/src/HesaiLidar_General_SDK/src/tcp_command_client.c.o: CMakeFiles/PandarGeneralSDK.dir/flags.make
CMakeFiles/PandarGeneralSDK.dir/src/HesaiLidar_General_SDK/src/tcp_command_client.c.o: /unitree/module/graph_pid_ws/src/HesaiLidar_General_ROS-ROS2/src/HesaiLidar_General_SDK/src/tcp_command_client.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/unitree/module/graph_pid_ws/build/hesai_lidar/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object CMakeFiles/PandarGeneralSDK.dir/src/HesaiLidar_General_SDK/src/tcp_command_client.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/PandarGeneralSDK.dir/src/HesaiLidar_General_SDK/src/tcp_command_client.c.o   -c /unitree/module/graph_pid_ws/src/HesaiLidar_General_ROS-ROS2/src/HesaiLidar_General_SDK/src/tcp_command_client.c

CMakeFiles/PandarGeneralSDK.dir/src/HesaiLidar_General_SDK/src/tcp_command_client.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/PandarGeneralSDK.dir/src/HesaiLidar_General_SDK/src/tcp_command_client.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /unitree/module/graph_pid_ws/src/HesaiLidar_General_ROS-ROS2/src/HesaiLidar_General_SDK/src/tcp_command_client.c > CMakeFiles/PandarGeneralSDK.dir/src/HesaiLidar_General_SDK/src/tcp_command_client.c.i

CMakeFiles/PandarGeneralSDK.dir/src/HesaiLidar_General_SDK/src/tcp_command_client.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/PandarGeneralSDK.dir/src/HesaiLidar_General_SDK/src/tcp_command_client.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /unitree/module/graph_pid_ws/src/HesaiLidar_General_ROS-ROS2/src/HesaiLidar_General_SDK/src/tcp_command_client.c -o CMakeFiles/PandarGeneralSDK.dir/src/HesaiLidar_General_SDK/src/tcp_command_client.c.s

CMakeFiles/PandarGeneralSDK.dir/src/HesaiLidar_General_SDK/src/util.c.o: CMakeFiles/PandarGeneralSDK.dir/flags.make
CMakeFiles/PandarGeneralSDK.dir/src/HesaiLidar_General_SDK/src/util.c.o: /unitree/module/graph_pid_ws/src/HesaiLidar_General_ROS-ROS2/src/HesaiLidar_General_SDK/src/util.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/unitree/module/graph_pid_ws/build/hesai_lidar/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object CMakeFiles/PandarGeneralSDK.dir/src/HesaiLidar_General_SDK/src/util.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/PandarGeneralSDK.dir/src/HesaiLidar_General_SDK/src/util.c.o   -c /unitree/module/graph_pid_ws/src/HesaiLidar_General_ROS-ROS2/src/HesaiLidar_General_SDK/src/util.c

CMakeFiles/PandarGeneralSDK.dir/src/HesaiLidar_General_SDK/src/util.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/PandarGeneralSDK.dir/src/HesaiLidar_General_SDK/src/util.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /unitree/module/graph_pid_ws/src/HesaiLidar_General_ROS-ROS2/src/HesaiLidar_General_SDK/src/util.c > CMakeFiles/PandarGeneralSDK.dir/src/HesaiLidar_General_SDK/src/util.c.i

CMakeFiles/PandarGeneralSDK.dir/src/HesaiLidar_General_SDK/src/util.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/PandarGeneralSDK.dir/src/HesaiLidar_General_SDK/src/util.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /unitree/module/graph_pid_ws/src/HesaiLidar_General_ROS-ROS2/src/HesaiLidar_General_SDK/src/util.c -o CMakeFiles/PandarGeneralSDK.dir/src/HesaiLidar_General_SDK/src/util.c.s

# Object files for target PandarGeneralSDK
PandarGeneralSDK_OBJECTS = \
"CMakeFiles/PandarGeneralSDK.dir/src/HesaiLidar_General_SDK/src/pandarGeneral_sdk.cc.o" \
"CMakeFiles/PandarGeneralSDK.dir/src/HesaiLidar_General_SDK/src/tcp_command_client.c.o" \
"CMakeFiles/PandarGeneralSDK.dir/src/HesaiLidar_General_SDK/src/util.c.o"

# External object files for target PandarGeneralSDK
PandarGeneralSDK_EXTERNAL_OBJECTS =

libPandarGeneralSDK.so: CMakeFiles/PandarGeneralSDK.dir/src/HesaiLidar_General_SDK/src/pandarGeneral_sdk.cc.o
libPandarGeneralSDK.so: CMakeFiles/PandarGeneralSDK.dir/src/HesaiLidar_General_SDK/src/tcp_command_client.c.o
libPandarGeneralSDK.so: CMakeFiles/PandarGeneralSDK.dir/src/HesaiLidar_General_SDK/src/util.c.o
libPandarGeneralSDK.so: CMakeFiles/PandarGeneralSDK.dir/build.make
libPandarGeneralSDK.so: libPandarGeneral.so
libPandarGeneralSDK.so: /usr/lib/aarch64-linux-gnu/libboost_system.so.1.71.0
libPandarGeneralSDK.so: /usr/lib/aarch64-linux-gnu/libboost_filesystem.so
libPandarGeneralSDK.so: /usr/lib/aarch64-linux-gnu/libboost_date_time.so
libPandarGeneralSDK.so: /usr/lib/aarch64-linux-gnu/libboost_iostreams.so
libPandarGeneralSDK.so: /usr/lib/aarch64-linux-gnu/libboost_regex.so
libPandarGeneralSDK.so: /usr/lib/aarch64-linux-gnu/libpcl_io.so
libPandarGeneralSDK.so: /usr/lib/aarch64-linux-gnu/libboost_thread.so.1.71.0
libPandarGeneralSDK.so: libhesai_lidar__rosidl_typesupport_cpp.so
libPandarGeneralSDK.so: /usr/lib/aarch64-linux-gnu/libpcl_octree.so
libPandarGeneralSDK.so: /usr/lib/aarch64-linux-gnu/libpcl_common.so
libPandarGeneralSDK.so: /usr/lib/aarch64-linux-gnu/libboost_atomic.so.1.71.0
libPandarGeneralSDK.so: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_c.so
libPandarGeneralSDK.so: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_generator_c.so
libPandarGeneralSDK.so: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_c.so
libPandarGeneralSDK.so: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_cpp.so
libPandarGeneralSDK.so: /opt/ros/foxy/lib/librosidl_typesupport_introspection_cpp.so
libPandarGeneralSDK.so: /opt/ros/foxy/lib/librosidl_typesupport_introspection_c.so
libPandarGeneralSDK.so: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_cpp.so
libPandarGeneralSDK.so: /opt/ros/foxy/lib/librosidl_typesupport_cpp.so
libPandarGeneralSDK.so: /opt/ros/foxy/lib/librosidl_typesupport_c.so
libPandarGeneralSDK.so: /opt/ros/foxy/lib/librosidl_runtime_c.so
libPandarGeneralSDK.so: /opt/ros/foxy/lib/librcpputils.so
libPandarGeneralSDK.so: /opt/ros/foxy/lib/librcutils.so
libPandarGeneralSDK.so: CMakeFiles/PandarGeneralSDK.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/unitree/module/graph_pid_ws/build/hesai_lidar/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX shared library libPandarGeneralSDK.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/PandarGeneralSDK.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/PandarGeneralSDK.dir/build: libPandarGeneralSDK.so

.PHONY : CMakeFiles/PandarGeneralSDK.dir/build

CMakeFiles/PandarGeneralSDK.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/PandarGeneralSDK.dir/cmake_clean.cmake
.PHONY : CMakeFiles/PandarGeneralSDK.dir/clean

CMakeFiles/PandarGeneralSDK.dir/depend:
	cd /unitree/module/graph_pid_ws/build/hesai_lidar && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /unitree/module/graph_pid_ws/src/HesaiLidar_General_ROS-ROS2 /unitree/module/graph_pid_ws/src/HesaiLidar_General_ROS-ROS2 /unitree/module/graph_pid_ws/build/hesai_lidar /unitree/module/graph_pid_ws/build/hesai_lidar /unitree/module/graph_pid_ws/build/hesai_lidar/CMakeFiles/PandarGeneralSDK.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/PandarGeneralSDK.dir/depend

