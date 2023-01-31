cmake_minimum_required(VERSION 3.5)
project(mqtt_ros)

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

add_compile_options(-fPIC)

if (UNIX AND NOT APPLE)
    # add_compile_options(-std=c++1y)
    add_compile_options(-pthread)
endif(UNIX AND NOT APPLE)

# find_package(OpenCV REQUIRED)
# find_package(PCL 1.3 REQUIRED)
# find_package(PCL 1.3 REQUIRED COMPONENTS common)
# find_package(yaml-cpp REQUIRED)  
# find_package(Eigen3 3.3.7 REQUIRED NO_MODULE)

#link pcl
# include_directories(${PCL_INCLUDE_DIRS})
# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)
find_package(PahoMqttCpp REQUIRED)

add_library(mqtt_ros SHARED src/mqtt_ros.cpp)
target_include_directories(mqtt_ros 
  PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(mqtt_ros 
  rclcpp 
  rclcpp_components
  std_msgs
  nlohmann_json)
target_link_libraries(mqtt_ros PahoMqttCpp::paho-mqttpp3)
rclcpp_components_register_node(mqtt_ros
  PLUGIN "mqtt_ros::MqttRos" 
  EXECUTABLE mqtt_ros_node)

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

install(TARGETS mqtt_ros EXPORT export_mqtt_ros DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_mqtt_ros HAS_LIBRARY_TARGET)

ament_package()

