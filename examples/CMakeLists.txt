cmake_minimum_required(VERSION 3.10.2)
project(test_tools_ros_examples 
  LANGUAGES CXX
  VERSION 0.1.0
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(GTest CONFIG REQUIRED COMPONENTS GTest GMock)


# Publisher component
add_library(publisher_component SHARED src/publisher.cc)
target_compile_definitions(publisher_component PRIVATE "COMPOSITION_BUILDING_DLL")

target_include_directories(publisher_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(publisher_component
  rclcpp
  rclcpp_components
  std_msgs
)
rclcpp_components_register_nodes(publisher_component "test_composition::Publisher")



# Subscriber component
add_library(subscriber_component SHARED src/subscriber.cc)
target_compile_definitions(subscriber_component PRIVATE "COMPOSITION_BUILDING_DLL")
target_include_directories(subscriber_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(subscriber_component
  rclcpp
  rclcpp_components
  std_msgs
)
rclcpp_components_register_nodes(subscriber_component "test_composition::Subscriber")



# Manual/static Node composition
add_executable(manual_composition src/manual_composition.cc)
target_link_libraries(manual_composition
  publisher_component
  subscriber_component
)
ament_target_dependencies(manual_composition
  rclcpp
)


install(TARGETS
  publisher_component
  subscriber_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  manual_composition
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(test_tools_ros REQUIRED)
  test_tools_add_doubles(publisher_component
    AMENT_DEPENDENCIES
      rclcpp
      rclcpp_components
      std_msgs
  )
  test_tools_add_doubles(subscriber_component
    AMENT_DEPENDENCIES
      rclcpp
      rclcpp_components
      std_msgs
  )
  
  add_subdirectory(test)
endif()

ament_package()
