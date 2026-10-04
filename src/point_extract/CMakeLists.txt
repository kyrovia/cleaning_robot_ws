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
find_package(PCL REQUIRED COMPONENTS common io filters search segmentation)

function(point_extract_node target source)
  add_executable(${target} ${source})
  target_compile_features(${target} PUBLIC cxx_std_17)
  target_include_directories(${target} PRIVATE ${PCL_INCLUDE_DIRS})
  target_compile_definitions(${target} PRIVATE ${PCL_DEFINITIONS})
  ament_target_dependencies(${target}
    ament_index_cpp
    rclcpp
    sensor_msgs
    pcl_conversions
  )
  target_link_libraries(${target} ${PCL_LIBRARIES})
endfunction()

point_extract_node(table_extract_node src/table_extract_node.cpp)
point_extract_node(cluster_extract_node src/cluster_extract_node.cpp)

install(TARGETS table_extract_node cluster_extract_node
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
