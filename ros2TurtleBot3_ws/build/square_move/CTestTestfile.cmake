# CMake generated Testfile for 
# Source directory: /home/toby/ThirdYearProject/ros2TurtleBot3_ws/src/square_move
# Build directory: /home/toby/ThirdYearProject/ros2TurtleBot3_ws/build/square_move
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(copyright "/usr/bin/python3" "-u" "/opt/ros/dashing/share/ament_cmake_test/cmake/run_test.py" "/home/toby/ThirdYearProject/ros2TurtleBot3_ws/build/square_move/test_results/square_move/copyright.xunit.xml" "--package-name" "square_move" "--output-file" "/home/toby/ThirdYearProject/ros2TurtleBot3_ws/build/square_move/ament_copyright/copyright.txt" "--command" "/opt/ros/dashing/bin/ament_copyright" "--xunit-file" "/home/toby/ThirdYearProject/ros2TurtleBot3_ws/build/square_move/test_results/square_move/copyright.xunit.xml")
set_tests_properties(copyright PROPERTIES  LABELS "copyright;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/toby/ThirdYearProject/ros2TurtleBot3_ws/src/square_move")
add_test(cppcheck "/usr/bin/python3" "-u" "/opt/ros/dashing/share/ament_cmake_test/cmake/run_test.py" "/home/toby/ThirdYearProject/ros2TurtleBot3_ws/build/square_move/test_results/square_move/cppcheck.xunit.xml" "--package-name" "square_move" "--output-file" "/home/toby/ThirdYearProject/ros2TurtleBot3_ws/build/square_move/ament_cppcheck/cppcheck.txt" "--command" "/opt/ros/dashing/bin/ament_cppcheck" "--xunit-file" "/home/toby/ThirdYearProject/ros2TurtleBot3_ws/build/square_move/test_results/square_move/cppcheck.xunit.xml" "--include_dirs" "/home/toby/ThirdYearProject/ros2TurtleBot3_ws/src/square_move/include")
set_tests_properties(cppcheck PROPERTIES  LABELS "cppcheck;linter" TIMEOUT "120" WORKING_DIRECTORY "/home/toby/ThirdYearProject/ros2TurtleBot3_ws/src/square_move")
add_test(cpplint "/usr/bin/python3" "-u" "/opt/ros/dashing/share/ament_cmake_test/cmake/run_test.py" "/home/toby/ThirdYearProject/ros2TurtleBot3_ws/build/square_move/test_results/square_move/cpplint.xunit.xml" "--package-name" "square_move" "--output-file" "/home/toby/ThirdYearProject/ros2TurtleBot3_ws/build/square_move/ament_cpplint/cpplint.txt" "--command" "/opt/ros/dashing/bin/ament_cpplint" "--xunit-file" "/home/toby/ThirdYearProject/ros2TurtleBot3_ws/build/square_move/test_results/square_move/cpplint.xunit.xml")
set_tests_properties(cpplint PROPERTIES  LABELS "cpplint;linter" TIMEOUT "120" WORKING_DIRECTORY "/home/toby/ThirdYearProject/ros2TurtleBot3_ws/src/square_move")
add_test(lint_cmake "/usr/bin/python3" "-u" "/opt/ros/dashing/share/ament_cmake_test/cmake/run_test.py" "/home/toby/ThirdYearProject/ros2TurtleBot3_ws/build/square_move/test_results/square_move/lint_cmake.xunit.xml" "--package-name" "square_move" "--output-file" "/home/toby/ThirdYearProject/ros2TurtleBot3_ws/build/square_move/ament_lint_cmake/lint_cmake.txt" "--command" "/opt/ros/dashing/bin/ament_lint_cmake" "--xunit-file" "/home/toby/ThirdYearProject/ros2TurtleBot3_ws/build/square_move/test_results/square_move/lint_cmake.xunit.xml")
set_tests_properties(lint_cmake PROPERTIES  LABELS "lint_cmake;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/toby/ThirdYearProject/ros2TurtleBot3_ws/src/square_move")
add_test(uncrustify "/usr/bin/python3" "-u" "/opt/ros/dashing/share/ament_cmake_test/cmake/run_test.py" "/home/toby/ThirdYearProject/ros2TurtleBot3_ws/build/square_move/test_results/square_move/uncrustify.xunit.xml" "--package-name" "square_move" "--output-file" "/home/toby/ThirdYearProject/ros2TurtleBot3_ws/build/square_move/ament_uncrustify/uncrustify.txt" "--command" "/opt/ros/dashing/bin/ament_uncrustify" "--xunit-file" "/home/toby/ThirdYearProject/ros2TurtleBot3_ws/build/square_move/test_results/square_move/uncrustify.xunit.xml")
set_tests_properties(uncrustify PROPERTIES  LABELS "uncrustify;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/toby/ThirdYearProject/ros2TurtleBot3_ws/src/square_move")
add_test(xmllint "/usr/bin/python3" "-u" "/opt/ros/dashing/share/ament_cmake_test/cmake/run_test.py" "/home/toby/ThirdYearProject/ros2TurtleBot3_ws/build/square_move/test_results/square_move/xmllint.xunit.xml" "--package-name" "square_move" "--output-file" "/home/toby/ThirdYearProject/ros2TurtleBot3_ws/build/square_move/ament_xmllint/xmllint.txt" "--command" "/opt/ros/dashing/bin/ament_xmllint" "--xunit-file" "/home/toby/ThirdYearProject/ros2TurtleBot3_ws/build/square_move/test_results/square_move/xmllint.xunit.xml")
set_tests_properties(xmllint PROPERTIES  LABELS "xmllint;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/toby/ThirdYearProject/ros2TurtleBot3_ws/src/square_move")
