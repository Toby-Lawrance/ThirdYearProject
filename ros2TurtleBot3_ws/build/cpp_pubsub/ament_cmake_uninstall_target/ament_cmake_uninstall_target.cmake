# generated from
# ament_cmake_core/cmake/uninstall_target/ament_cmake_uninstall_target.cmake.in

function(ament_cmake_uninstall_target_remove_empty_directories path)
<<<<<<< HEAD
  set(install_space "/home/toby/ThirdYearProject/ros2TurtleBot3_ws/install/cpp_pubsub")
=======
  set(install_space "/home/portia/ThirdYearProject/ros2TurtleBot3_ws/install/cpp_pubsub")
>>>>>>> 2bfd020d511dce9e8de31ce95719fda3d0662758
  if(install_space STREQUAL "")
    message(FATAL_ERROR "The CMAKE_INSTALL_PREFIX variable must not be empty")
  endif()

  string(LENGTH "${install_space}" length)
  string(SUBSTRING "${path}" 0 ${length} path_prefix)
  if(NOT path_prefix STREQUAL install_space)
    message(FATAL_ERROR "The path '${path}' must be within the install space '${install_space}'")
  endif()
  if(path STREQUAL install_space)
    return()
  endif()

  # check if directory is empty
  file(GLOB files "${path}/*")
  list(LENGTH files length)
  if(length EQUAL 0)
    message(STATUS "Uninstalling: ${path}/")
    execute_process(COMMAND "/usr/bin/cmake" "-E" "remove_directory" "${path}")
    # recursively try to remove parent directories
    get_filename_component(parent_path "${path}" PATH)
    ament_cmake_uninstall_target_remove_empty_directories("${parent_path}")
  endif()
endfunction()

# uninstall files installed using the standard install() function
<<<<<<< HEAD
set(install_manifest "/home/toby/ThirdYearProject/ros2TurtleBot3_ws/build/cpp_pubsub/install_manifest.txt")
=======
set(install_manifest "/home/portia/ThirdYearProject/ros2TurtleBot3_ws/build/cpp_pubsub/install_manifest.txt")
>>>>>>> 2bfd020d511dce9e8de31ce95719fda3d0662758
if(NOT EXISTS "${install_manifest}")
  message(FATAL_ERROR "Cannot find install manifest: ${install_manifest}")
endif()

file(READ "${install_manifest}" installed_files)
string(REGEX REPLACE "\n" ";" installed_files "${installed_files}")
foreach(installed_file ${installed_files})
  if(EXISTS "${installed_file}" OR IS_SYMLINK "${installed_file}")
    message(STATUS "Uninstalling: ${installed_file}")
    file(REMOVE "${installed_file}")
    if(EXISTS "${installed_file}" OR IS_SYMLINK "${installed_file}")
      message(FATAL_ERROR "Failed to remove '${installed_file}'")
    endif()

    # remove empty parent folders
    get_filename_component(parent_path "${installed_file}" PATH)
    ament_cmake_uninstall_target_remove_empty_directories("${parent_path}")
  endif()
endforeach()

# end of template

message(STATUS "Execute custom uninstall script")

# begin of custom uninstall code

# uninstall files installed using the symlink install functions
<<<<<<< HEAD
include("/home/toby/ThirdYearProject/ros2TurtleBot3_ws/build/cpp_pubsub/ament_cmake_symlink_install/ament_cmake_symlink_install_uninstall_script.cmake")
=======
include("/home/portia/ThirdYearProject/ros2TurtleBot3_ws/build/cpp_pubsub/ament_cmake_symlink_install/ament_cmake_symlink_install_uninstall_script.cmake")
>>>>>>> 2bfd020d511dce9e8de31ce95719fda3d0662758