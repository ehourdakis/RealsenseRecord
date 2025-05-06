cmake_minimum_required(VERSION 3.8)
project(realsense_record_publisher)

## Compile as C++17
add_compile_options(-std=c++17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall)
  #add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
#find_package(OpenCV REQUIRED COMPONENTS highgui imgcodecs imgproc videoio)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Find libraries
find_package(OpenCV REQUIRED)

find_package(Boost COMPONENTS system program_options iostreams regex filesystem python thread coroutine context REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(realsense_record_publisher src/realsense_record_publisher.cpp)
target_link_libraries(realsense_record_publisher -lboost_filesystem)
ament_target_dependencies(realsense_record_publisher
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

install(TARGETS realsense_record_publisher
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
