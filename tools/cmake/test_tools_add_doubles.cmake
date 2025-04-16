# @file      test_tools_add_doubles.cmake
# @author    Sławomir Cielepak (sie@spyro-soft.com)
# @date      2024-11-25
# @copyright Copyright (c) 2024 Beam Limited. All rights reserved.

# @license   Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included
# in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE, AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT, OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

# @details This macro creates a test doubles library with mocked ROS 2 rclcpp layer.
#          It is used to create a library that can be used in integration tests
#          to replace the original implementation of a target with a test double.
#          The test double library is created with the same sources as the original
#          implementation, but with additional test doubles for the ROS 2 rclcpp layer.




# @brief Create and install a test doubles library for a given target.
#
# This macro creates a test doubles library with mocked ROS 2 rclcpp layer.
#
# @param target Name of the target to create test doubles for. Will create library
#               named <target>_test_doubles and alias target <project_name>::<target>_test_doubles
# @param SOURCES List of source files to be compiled into the test doubles library
# @param AMENT_DEPENDENCIES (optional) Additional dependencies to be linked with the test library
#
# @example
#   project(robot)
#   find_package(my_robot_test_doubles REQUIRED)
#
#   # Add original implementation target
#   add_library(my_robot
#     src/sensor.cpp
#     src/actuator.cpp)
#
#   set(AMENT_TARGET_DEPS Eigen3 rclcpp std_msgs)
#   ament_target_dependencies(my_robot ${AMENT_TARGET_DEPS})
#
#   # Create test doubles target for the original implementation
#   test_tools_add_doubles(my_robot
#     AMENT_DEPENDENCIES ${AMENT_TARGET_DEPS})
#
#   ---
#
#   # In another package that uses the test doubles
#   
#   add_executable(integration_test test/main.cpp)
#   target_link_libraries(integration_test
#     robot::my_robot_test_doubles
#   )
#
function(test_tools_add_doubles target)
  include(CMakePackageConfigHelpers)

  cmake_parse_arguments(arg "INTERFACE" "" "SOURCES;AMENT_DEPENDENCIES" ${ARGN})

  set(LIB_DOUBLES_NAME ${target}_test_doubles)

  if(arg_INTERFACE)
    add_library(${LIB_DOUBLES_NAME} INTERFACE)
    add_library(${PROJECT_NAME}::${LIB_DOUBLES_NAME} ALIAS ${LIB_DOUBLES_NAME})

    target_include_directories(${LIB_DOUBLES_NAME} INTERFACE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    )

    target_link_libraries(${LIB_DOUBLES_NAME} INTERFACE
      test_tools_ros::publisher_mock
      test_tools_ros::subscription_mock
      test_tools_ros::timer_mock
    )

    ament_target_dependencies(${LIB_DOUBLES_NAME} INTERFACE
      ${arg_AMENT_DEPENDENCIES}
    )
  else()
    get_target_property(TARGET_SOURCES ${target} SOURCES)

    add_library(${LIB_DOUBLES_NAME} ${TARGET_SOURCES} ${arg_SOURCES})
    add_library(${PROJECT_NAME}::${LIB_DOUBLES_NAME} ALIAS ${LIB_DOUBLES_NAME})
    set_property(TARGET ${LIB_DOUBLES_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)

    target_include_directories(${LIB_DOUBLES_NAME} PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    )

    target_link_libraries(${LIB_DOUBLES_NAME}
      test_tools_ros::publisher_mock
      test_tools_ros::subscription_mock
      test_tools_ros::timer_mock
      test_tools_ros::common
    )

    ament_target_dependencies(${LIB_DOUBLES_NAME}
      ${arg_AMENT_DEPENDENCIES}
    )
  endif()
  
  
  # set(TEST_TOOLS_ROS_CMAKE_DIR "${CMAKE_INSTALL_PREFIX}/../test_tools_ros/share/test_tools_ros/cmake/")
  set(TEST_TOOLS_ROS_CMAKE_DIR "${CMAKE_CURRENT_FUNCTION_LIST_DIR}")


  configure_package_config_file(
    ${TEST_TOOLS_ROS_CMAKE_DIR}/test-doublesConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/${LIB_DOUBLES_NAME}Config.cmake
    INSTALL_DESTINATION share/${LIB_DOUBLES_NAME}/cmake
  )

  install(TARGETS ${LIB_DOUBLES_NAME}
    EXPORT ${LIB_DOUBLES_NAME}_targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
  )

  install(EXPORT ${LIB_DOUBLES_NAME}_targets
    FILE ${LIB_DOUBLES_NAME}_targetsExport.cmake
    NAMESPACE ${target}::
    DESTINATION share/${LIB_DOUBLES_NAME}/cmake/
  )

  install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${LIB_DOUBLES_NAME}Config.cmake
    DESTINATION share/${LIB_DOUBLES_NAME}/cmake/
  )
endfunction()