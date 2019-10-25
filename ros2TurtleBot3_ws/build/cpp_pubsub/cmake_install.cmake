<<<<<<< HEAD
# Install script for directory: /home/toby/ThirdYearProject/ros2TurtleBot3_ws/src/cpp_pubsub

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/toby/ThirdYearProject/ros2TurtleBot3_ws/install/cpp_pubsub")
=======
# Install script for directory: /home/portia/ThirdYearProject/ros2TurtleBot3_ws/src/cpp_pubsub

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/portia/ThirdYearProject/ros2TurtleBot3_ws/install/cpp_pubsub")
>>>>>>> 2bfd020d511dce9e8de31ce95719fda3d0662758
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
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
<<<<<<< HEAD
  include("/home/toby/ThirdYearProject/ros2TurtleBot3_ws/build/cpp_pubsub/ament_cmake_symlink_install/ament_cmake_symlink_install.cmake")
=======
  include("/home/portia/ThirdYearProject/ros2TurtleBot3_ws/build/cpp_pubsub/ament_cmake_symlink_install/ament_cmake_symlink_install.cmake")
>>>>>>> 2bfd020d511dce9e8de31ce95719fda3d0662758
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
<<<<<<< HEAD
file(WRITE "/home/toby/ThirdYearProject/ros2TurtleBot3_ws/build/cpp_pubsub/${CMAKE_INSTALL_MANIFEST}"
=======
file(WRITE "/home/portia/ThirdYearProject/ros2TurtleBot3_ws/build/cpp_pubsub/${CMAKE_INSTALL_MANIFEST}"
>>>>>>> 2bfd020d511dce9e8de31ce95719fda3d0662758
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
