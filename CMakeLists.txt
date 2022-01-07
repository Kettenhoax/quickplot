cmake_minimum_required(VERSION 3.5)
project(quickplot)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wunused)
endif()

# TODO shouldn't need to repeat it, imgui_vendor should export setting
set(OpenGL_GL_PREFERENCE GLVND)

find_package(Boost REQUIRED COMPONENTS system)

find_package(ament_cmake REQUIRED)
find_package(implot_vendor REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)

add_library(quickplot
  src/introspection.cpp
  src/message_parser.cpp
  src/config.cpp)
target_include_directories(quickplot PUBLIC include)
ament_target_dependencies(quickplot
  rclcpp
  rcpputils
  yaml_cpp_vendor
  std_msgs
  rosidl_typesupport_cpp
  rosidl_typesupport_introspection_cpp)

add_executable(quickplot_bin src/main.cpp)
set_target_properties(quickplot_bin PROPERTIES OUTPUT_NAME "quickplot")
target_link_libraries(quickplot_bin
  Boost::system
  quickplot)

ament_target_dependencies(quickplot_bin
  implot_vendor
  rclcpp
  std_msgs
  rosidl_typesupport_cpp
  rosidl_typesupport_introspection_cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gmock REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(vision_msgs REQUIRED)

  ament_add_gmock(test_introspection test/test_introspection.cpp)
  target_link_libraries(test_introspection quickplot)
  ament_target_dependencies(test_introspection
    rclcpp
    rosidl_typesupport_introspection_cpp)

  ament_add_gmock(test_message_parser test/test_message_parser.cpp)
  target_link_libraries(test_message_parser quickplot)
  ament_target_dependencies(test_message_parser
    rclcpp
    rosidl_typesupport_cpp
    rosidl_typesupport_introspection_cpp
    geometry_msgs
    vision_msgs)

  ament_add_gmock(test_config test/test_config.cpp WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
  target_link_libraries(test_config quickplot)
endif()

install(TARGETS quickplot quickplot_bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
