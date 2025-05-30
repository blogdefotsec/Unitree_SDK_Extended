project(slam_example)
cmake_minimum_required(VERSION 3.16)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(unitree_sdk2 REQUIRED)
link_libraries(unitree_ros2_idl_cpp)

include_directories(/usr/local/include)

add_executable(start_mappping unitree_slam_example/start_mapping.cpp)
target_link_libraries(start_mappping unitree_sdk2)
add_executable(end_mappping unitree_slam_example/end_mapping.cpp)
target_link_libraries(end_mappping unitree_sdk2)

add_executable(add_node unitree_slam_example/add_node.cpp)
target_link_libraries(add_node unitree_sdk2)
add_executable(add_edge unitree_slam_example/add_edge.cpp)
target_link_libraries(add_edge unitree_sdk2)


add_executable(delete_node unitree_slam_example/delete_node.cpp)
target_link_libraries(delete_node unitree_sdk2)
add_executable(delete_edge unitree_slam_example/delete_edge.cpp)
target_link_libraries(delete_edge unitree_sdk2)

add_executable(query_edge unitree_slam_example/query_edge.cpp)
target_link_libraries(query_edge unitree_sdk2)
add_executable(query_node unitree_slam_example/query_node.cpp)
target_link_libraries(query_node unitree_sdk2)

add_executable(start_relocation unitree_slam_example/start_relocation.cpp)
target_link_libraries(start_relocation unitree_sdk2)
add_executable(pose_init unitree_slam_example/pose_init.cpp)
target_link_libraries(pose_init unitree_sdk2)


add_executable(start_nav unitree_slam_example/start_nav.cpp)
target_link_libraries(start_nav unitree_sdk2)
add_executable(single_nav unitree_slam_example/single_nav.cpp)
target_link_libraries(single_nav unitree_sdk2)
add_executable(set_target_nav unitree_slam_example/set_target_nav.cpp)
target_link_libraries(set_target_nav unitree_sdk2)



add_executable(multiple_nav_default unitree_slam_example/multiple_nav_default.cpp)
target_link_libraries(multiple_nav_default unitree_sdk2)
add_executable(multiple_nav_set unitree_slam_example/multiple_nav_set.cpp)
target_link_libraries(multiple_nav_set unitree_sdk2)
add_executable(pause_nav unitree_slam_example/pause_nav.cpp)
target_link_libraries(pause_nav unitree_sdk2)
add_executable(recover_nav unitree_slam_example/recover_nav.cpp)
target_link_libraries(recover_nav unitree_sdk2)

add_executable(return_origin unitree_slam_example/return_origin.cpp)
target_link_libraries(return_origin unitree_sdk2)
add_executable(close_all_node unitree_slam_example/close_all_node.cpp)
target_link_libraries(close_all_node unitree_sdk2)

add_executable(rslidar_dds_test unitree_slam_example/rslidar_dds_test.cpp)
target_link_libraries(rslidar_dds_test unitree_sdk2)

add_executable(demo_b2 unitree_slam_example/demo_b2.cpp)
target_link_libraries(demo_b2 unitree_sdk2)
add_executable(demo_xt16 unitree_slam_example/demo_xt16.cpp)
target_link_libraries(demo_xt16 unitree_sdk2)
add_executable(demo_mid360 unitree_slam_example/demo_mid360.cpp)
target_link_libraries(demo_mid360 unitree_sdk2)


