cmake_minimum_required(VERSION 3.5)
project(quickplot)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wunused)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)

include(FetchContent)
FetchContent_Declare(quickplot GIT_REPOSITORY https://github.com/mahilab/mahi-gui.git) 
FetchContent_MakeAvailable(quickplot)

add_library(quickplot-utils src/typesupport_helpers.cpp)
target_include_directories(quickplot-utils PRIVATE include)
ament_target_dependencies(quickplot-utils
  rclcpp
  std_msgs
  rosidl_typesupport_cpp
  rosidl_typesupport_introspection_cpp)

add_executable(${PROJECT_NAME} src/plot.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE include)
target_link_libraries(${PROJECT_NAME} mahi::gui quickplot-utils)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  rosidl_typesupport_cpp
  rosidl_typesupport_introspection_cpp)
ament_export_targets(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)

  ament_add_gtest(test_parser test/test_parser.cpp)
  target_include_directories(test_parser PRIVATE include)
  target_link_libraries(test_parser quickplot-utils)
  ament_target_dependencies(test_parser
    rclcpp
    rosidl_typesupport_cpp
    rosidl_typesupport_introspection_cpp
    geometry_msgs
    nav_msgs)
endif()

install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
