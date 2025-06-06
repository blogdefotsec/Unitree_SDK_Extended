# Install script for directory: /unitree/module/Odometer_service/src/rpg_svo_pro_open/svo_ros

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/unitree/module/Odometer_service/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/unitree/module/Odometer_service/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/unitree/module/Odometer_service/install" TYPE PROGRAM FILES "/unitree/module/Odometer_service/build/svo_ros/catkin_generated/installspace/_setup_util.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/unitree/module/Odometer_service/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/unitree/module/Odometer_service/install" TYPE PROGRAM FILES "/unitree/module/Odometer_service/build/svo_ros/catkin_generated/installspace/env.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/unitree/module/Odometer_service/install/setup.bash;/unitree/module/Odometer_service/install/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/unitree/module/Odometer_service/install" TYPE FILE FILES
    "/unitree/module/Odometer_service/build/svo_ros/catkin_generated/installspace/setup.bash"
    "/unitree/module/Odometer_service/build/svo_ros/catkin_generated/installspace/local_setup.bash"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/unitree/module/Odometer_service/install/setup.sh;/unitree/module/Odometer_service/install/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/unitree/module/Odometer_service/install" TYPE FILE FILES
    "/unitree/module/Odometer_service/build/svo_ros/catkin_generated/installspace/setup.sh"
    "/unitree/module/Odometer_service/build/svo_ros/catkin_generated/installspace/local_setup.sh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/unitree/module/Odometer_service/install/setup.zsh;/unitree/module/Odometer_service/install/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/unitree/module/Odometer_service/install" TYPE FILE FILES
    "/unitree/module/Odometer_service/build/svo_ros/catkin_generated/installspace/setup.zsh"
    "/unitree/module/Odometer_service/build/svo_ros/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/unitree/module/Odometer_service/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/unitree/module/Odometer_service/install" TYPE FILE FILES "/unitree/module/Odometer_service/build/svo_ros/catkin_generated/installspace/.rosinstall")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libsvo_ros.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libsvo_ros.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libsvo_ros.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/unitree/module/Odometer_service/devel/.private/svo_ros/lib/libsvo_ros.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libsvo_ros.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libsvo_ros.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libsvo_ros.so"
         OLD_RPATH "/opt/ros/noetic/lib:/unitree/module/Odometer_service/devel/.private/svo_ceres_backend/lib:/unitree/module/Odometer_service/devel/.private/svo/lib:/unitree/module/Odometer_service/devel/.private/svo_tracker/lib:/unitree/module/Odometer_service/devel/.private/svo_img_align/lib:/unitree/module/Odometer_service/devel/.private/svo_direct/lib:/unitree/module/Odometer_service/devel/.private/svo_test_utils/lib:/unitree/module/Odometer_service/devel/.private/svo_vio_common/lib:/unitree/module/Odometer_service/devel/.private/vikit_solver/lib:/unitree/module/Odometer_service/devel/.private/svo_online_loopclosing/lib:/unitree/module/Odometer_service/devel/.private/rpg_common/lib:/unitree/module/Odometer_service/devel/.private/dbow2_catkin/lib:/unitree/module/Odometer_service/devel/.private/svo_pgo/lib:/unitree/module/Odometer_service/devel/.private/svo_common/lib:/unitree/module/Odometer_service/devel/.private/vikit_cameras/lib:/unitree/module/Odometer_service/devel/.private/fast/lib:/unitree/module/Odometer_service/devel/.private/eigen_checks/lib:/unitree/module/Odometer_service/devel/.private/ceres_catkin/lib:/unitree/module/Odometer_service/devel/.private/opengv/lib:/unitree/module/Odometer_service/devel/.private/vikit_ros/lib:/unitree/module/Odometer_service/devel/.private/vikit_common/lib:/unitree/module/Odometer_service/devel/.private/gflags_catkin/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libsvo_ros.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/svo_ros/svo_benchmark" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/svo_ros/svo_benchmark")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/svo_ros/svo_benchmark"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/svo_ros" TYPE EXECUTABLE FILES "/unitree/module/Odometer_service/devel/.private/svo_ros/lib/svo_ros/svo_benchmark")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/svo_ros/svo_benchmark" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/svo_ros/svo_benchmark")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/svo_ros/svo_benchmark"
         OLD_RPATH "/opt/ros/noetic/lib:/unitree/module/Odometer_service/devel/.private/svo_ceres_backend/lib:/unitree/module/Odometer_service/devel/.private/svo/lib:/unitree/module/Odometer_service/devel/.private/svo_tracker/lib:/unitree/module/Odometer_service/devel/.private/svo_img_align/lib:/unitree/module/Odometer_service/devel/.private/svo_direct/lib:/unitree/module/Odometer_service/devel/.private/svo_test_utils/lib:/unitree/module/Odometer_service/devel/.private/svo_vio_common/lib:/unitree/module/Odometer_service/devel/.private/vikit_solver/lib:/unitree/module/Odometer_service/devel/.private/svo_online_loopclosing/lib:/unitree/module/Odometer_service/devel/.private/rpg_common/lib:/unitree/module/Odometer_service/devel/.private/dbow2_catkin/lib:/unitree/module/Odometer_service/devel/.private/svo_pgo/lib:/unitree/module/Odometer_service/devel/.private/svo_common/lib:/unitree/module/Odometer_service/devel/.private/vikit_cameras/lib:/unitree/module/Odometer_service/devel/.private/fast/lib:/unitree/module/Odometer_service/devel/.private/eigen_checks/lib:/unitree/module/Odometer_service/devel/.private/ceres_catkin/lib:/unitree/module/Odometer_service/devel/.private/opengv/lib:/unitree/module/Odometer_service/devel/.private/vikit_ros/lib:/unitree/module/Odometer_service/devel/.private/vikit_common/lib:/unitree/module/Odometer_service/devel/.private/gflags_catkin/lib:/unitree/module/Odometer_service/devel/.private/svo_ros/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/svo_ros/svo_benchmark")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/svo_ros/svo_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/svo_ros/svo_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/svo_ros/svo_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/svo_ros" TYPE EXECUTABLE FILES "/unitree/module/Odometer_service/devel/.private/svo_ros/lib/svo_ros/svo_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/svo_ros/svo_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/svo_ros/svo_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/svo_ros/svo_node"
         OLD_RPATH "/opt/ros/noetic/lib:/unitree/module/Odometer_service/devel/.private/svo_ceres_backend/lib:/unitree/module/Odometer_service/devel/.private/svo/lib:/unitree/module/Odometer_service/devel/.private/svo_tracker/lib:/unitree/module/Odometer_service/devel/.private/svo_img_align/lib:/unitree/module/Odometer_service/devel/.private/svo_direct/lib:/unitree/module/Odometer_service/devel/.private/svo_test_utils/lib:/unitree/module/Odometer_service/devel/.private/svo_vio_common/lib:/unitree/module/Odometer_service/devel/.private/vikit_solver/lib:/unitree/module/Odometer_service/devel/.private/svo_online_loopclosing/lib:/unitree/module/Odometer_service/devel/.private/rpg_common/lib:/unitree/module/Odometer_service/devel/.private/dbow2_catkin/lib:/unitree/module/Odometer_service/devel/.private/svo_pgo/lib:/unitree/module/Odometer_service/devel/.private/svo_common/lib:/unitree/module/Odometer_service/devel/.private/vikit_cameras/lib:/unitree/module/Odometer_service/devel/.private/fast/lib:/unitree/module/Odometer_service/devel/.private/eigen_checks/lib:/unitree/module/Odometer_service/devel/.private/ceres_catkin/lib:/unitree/module/Odometer_service/devel/.private/opengv/lib:/unitree/module/Odometer_service/devel/.private/vikit_ros/lib:/unitree/module/Odometer_service/devel/.private/vikit_common/lib:/unitree/module/Odometer_service/devel/.private/gflags_catkin/lib:/unitree/module/Odometer_service/devel/.private/svo_ros/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/svo_ros/svo_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libsvo_nodelet.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libsvo_nodelet.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libsvo_nodelet.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/unitree/module/Odometer_service/devel/.private/svo_ros/lib/libsvo_nodelet.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libsvo_nodelet.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libsvo_nodelet.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libsvo_nodelet.so"
         OLD_RPATH "/opt/ros/noetic/lib:/unitree/module/Odometer_service/devel/.private/svo_ceres_backend/lib:/unitree/module/Odometer_service/devel/.private/svo/lib:/unitree/module/Odometer_service/devel/.private/svo_tracker/lib:/unitree/module/Odometer_service/devel/.private/svo_img_align/lib:/unitree/module/Odometer_service/devel/.private/svo_direct/lib:/unitree/module/Odometer_service/devel/.private/svo_test_utils/lib:/unitree/module/Odometer_service/devel/.private/svo_vio_common/lib:/unitree/module/Odometer_service/devel/.private/vikit_solver/lib:/unitree/module/Odometer_service/devel/.private/svo_online_loopclosing/lib:/unitree/module/Odometer_service/devel/.private/rpg_common/lib:/unitree/module/Odometer_service/devel/.private/dbow2_catkin/lib:/unitree/module/Odometer_service/devel/.private/svo_pgo/lib:/unitree/module/Odometer_service/devel/.private/svo_common/lib:/unitree/module/Odometer_service/devel/.private/vikit_cameras/lib:/unitree/module/Odometer_service/devel/.private/fast/lib:/unitree/module/Odometer_service/devel/.private/eigen_checks/lib:/unitree/module/Odometer_service/devel/.private/ceres_catkin/lib:/unitree/module/Odometer_service/devel/.private/opengv/lib:/unitree/module/Odometer_service/devel/.private/vikit_ros/lib:/unitree/module/Odometer_service/devel/.private/vikit_common/lib:/unitree/module/Odometer_service/devel/.private/gflags_catkin/lib:/unitree/module/Odometer_service/devel/.private/svo_ros/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libsvo_nodelet.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/unitree/module/Odometer_service/src/rpg_svo_pro_open/svo_ros/include/" FILES_MATCHING REGEX "/[^/]*\\.h$" REGEX "/[^/]*\\.hpp$" REGEX "/\\.svn$" EXCLUDE)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/svo_ros/launch" TYPE DIRECTORY FILES "/unitree/module/Odometer_service/src/rpg_svo_pro_open/svo_ros/launch/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/svo_ros/param" TYPE DIRECTORY FILES "/unitree/module/Odometer_service/src/rpg_svo_pro_open/svo_ros/param/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/svo_ros" TYPE FILE FILES "/unitree/module/Odometer_service/src/rpg_svo_pro_open/svo_ros/svo_nodelet.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/unitree/module/Odometer_service/build/svo_ros/catkin_generated/installspace/svo_ros.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/svo_ros/cmake" TYPE FILE FILES
    "/unitree/module/Odometer_service/build/svo_ros/catkin_generated/installspace/svo_rosConfig.cmake"
    "/unitree/module/Odometer_service/build/svo_ros/catkin_generated/installspace/svo_rosConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/svo_ros" TYPE FILE FILES "/unitree/module/Odometer_service/src/rpg_svo_pro_open/svo_ros/package.xml")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/unitree/module/Odometer_service/build/svo_ros/gtest/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/unitree/module/Odometer_service/build/svo_ros/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
