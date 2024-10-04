cmake_minimum_required(VERSION 3.8)
project(gps_nav_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(dependencies
  rclcpp
  sensor_msgs
  grid_map_msgs
  std_msgs
  grid_map_ros
)

include_directories(
  include
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/GridmapGpsCreator.cpp
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(gps_nav_tools_program
  src/gps_nav_tools_program.cpp
)
ament_target_dependencies(gps_nav_tools_program ${dependencies})
target_link_libraries(gps_nav_tools_program ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS gps_nav_tools_program
RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
DESTINATION include/
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)
ament_lint_auto_find_test_dependencies()

find_package(ament_cmake_gtest REQUIRED)
# add_subdirectory(tests)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()