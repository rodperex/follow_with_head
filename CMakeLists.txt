cmake_minimum_required(VERSION 3.8)
project(follow_with_head)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs)
find_package(control_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(image_geometry REQUIRED)
find_package(depth_image_proc REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(rclcpp REQUIRED)


set(dependencies
  vision_msgs
  geometry_msgs
  control_msgs
  image_transport
  cv_bridge
  OpenCV
  message_filters
  image_geometry
  depth_image_proc
  sensor_msgs
  rclcpp
  )

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/follow_with_head/HeadController.cpp
  src/follow_with_head/PID.cpp
  src/follow_with_head/HSVFilter.cpp
  src/follow_with_head/DepthEstimator.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(head_controller src/head_controller_main.cpp)
ament_target_dependencies(head_controller ${dependencies})
target_link_libraries(head_controller ${PROJECT_NAME})

add_executable(hsv_filter src/hsv_filter_main.cpp)
ament_target_dependencies(hsv_filter ${dependencies})
target_link_libraries(hsv_filter ${PROJECT_NAME})

add_executable(depth_estimator src/depth_main.cpp)
ament_target_dependencies(depth_estimator ${dependencies})
target_link_libraries(depth_estimator ${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  head_controller
  hsv_filter
  depth_estimator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
