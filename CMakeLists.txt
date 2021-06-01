cmake_minimum_required(VERSION 3.5)
project(quickplot)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wunused)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(four_wheel_steering_msgs)

include(FetchContent)
FetchContent_Declare(quickplot GIT_REPOSITORY https://github.com/mahilab/mahi-gui.git) 
FetchContent_MakeAvailable(quickplot)

add_executable(${PROJECT_NAME} "plot.cpp")
target_link_libraries(${PROJECT_NAME} mahi::gui)

ament_target_dependencies(${PROJECT_NAME} rclcpp four_wheel_steering_msgs)
ament_export_targets(${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
