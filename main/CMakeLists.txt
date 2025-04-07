cmake_minimum_required(VERSION 3.5)
project(p6_main)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(image_geometry REQUIRED)
find_package(depth_image_proc REQUIRED)
find_package(yolo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(OpenCV REQUIRED)



set(dependencies
    rclcpp
    sensor_msgs
    vision_msgs
    image_transport
    cv_bridge
    OpenCV
    message_filters
    pcl_conversions
    pcl_ros
    image_geometry
    depth_image_proc
    yolo_msgs
    geometry_msgs
    tf2_geometry_msgs
    tf2
    tf2_ros
    std_msgs
    visualization_msgs
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/p6/TfPublisher.cpp
  src/p6/PIDController.cpp

)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(main src/main.cpp)
ament_target_dependencies(main ${dependencies})
target_link_libraries(main ${PROJECT_NAME})


install(TARGETS
 ${PROJECT_NAME}
  main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
