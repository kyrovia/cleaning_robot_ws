cmake_minimum_required(VERSION 3.8)
project(traj_cover)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io sample_consensus segmentation)

add_library(coverage_planner
  src/coverage_planner.cpp
  src/visualizer.cpp
)
target_compile_features(coverage_planner PUBLIC cxx_std_17)
target_include_directories(coverage_planner
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)
target_compile_definitions(coverage_planner PRIVATE ${PCL_DEFINITIONS})
target_link_libraries(coverage_planner Eigen3::Eigen ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(cover_plan_node src/cover_plan_node.cpp)
target_compile_features(cover_plan_node PUBLIC cxx_std_17)
ament_target_dependencies(cover_plan_node
  ament_index_cpp
  geometry_msgs
  nav_msgs
  rclcpp
  visualization_msgs
)
target_link_libraries(cover_plan_node coverage_planner ${PCL_LIBRARIES})

install(TARGETS
  coverage_planner
  cover_plan_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch data viz DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_coverage_planner test/test_coverage_planner.cpp)
  if(TARGET test_coverage_planner)
    target_link_libraries(test_coverage_planner coverage_planner)
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(coverage_planner)
ament_package()
