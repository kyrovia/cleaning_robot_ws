cmake_minimum_required(VERSION 3.8)
project(point_extract)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters segmentation)

add_executable(table_extract_node src/table_extract_node.cpp)
target_compile_features(table_extract_node PUBLIC cxx_std_17)
target_include_directories(table_extract_node PRIVATE ${PCL_INCLUDE_DIRS})
target_compile_definitions(table_extract_node PRIVATE ${PCL_DEFINITIONS})
ament_target_dependencies(table_extract_node
  ament_index_cpp
  rclcpp
  sensor_msgs
  pcl_conversions
)
target_link_libraries(table_extract_node ${PCL_LIBRARIES})

install(TARGETS table_extract_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch data
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
