# CMake generated Testfile for 
# Source directory: /unitree/module/Odometer_service/src/rpg_svo_pro_open/svo
# Build directory: /unitree/module/Odometer_service/build/svo
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_svo_gtest_svo-test-frame "/unitree/module/Odometer_service/build/svo/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/unitree/module/Odometer_service/build/svo/test_results/svo/gtest-svo-test-frame.xml" "--return-code" "/unitree/module/Odometer_service/devel/.private/svo/lib/svo/svo-test-frame --gtest_output=xml:/unitree/module/Odometer_service/build/svo/test_results/svo/gtest-svo-test-frame.xml")
set_tests_properties(_ctest_svo_gtest_svo-test-frame PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/catkin/cmake/test/gtest.cmake;98;catkin_run_tests_target;/opt/ros/noetic/share/catkin/cmake/test/gtest.cmake;37;_catkin_add_google_test;/unitree/module/Odometer_service/src/rpg_svo_pro_open/svo/CMakeLists.txt;51;catkin_add_gtest;/unitree/module/Odometer_service/src/rpg_svo_pro_open/svo/CMakeLists.txt;0;")
subdirs("gtest")
