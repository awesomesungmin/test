cmake_minimum_required(VERSION 3.5)
project(camera)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(image_transport REQUIRED)

include_directories(include)

set(dependencies
  rclcpp
  std_msgs
  PCL
  pcl_conversions
  image_transport
  cv_bridge
  geometry_msgs
  visualization_msgs
  sensor_msgs
  vision_msgs
  OpenCV
)
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(simple_cam src/simple_cam.cpp)
ament_target_dependencies(simple_cam ${dependencies})

add_executable(multi_cam src/multi_cam.cpp)
ament_target_dependencies(multi_cam ${dependencies})

add_executable(yolo_traffic src/yolo_traffic.cpp)
ament_target_dependencies(yolo_traffic ${dependencies})

install(TARGETS
  simple_cam
  multi_cam
  yolo_traffic
DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
