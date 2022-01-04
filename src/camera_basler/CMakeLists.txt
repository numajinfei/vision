cmake_minimum_required(VERSION 3.5)

cmake_policy(SET CMP0074 OLD) 

# compiler configuration

set(CMAKE_CXX_STANDARD 14)


project(camera_basler VERSION 1.0.0.001 LANGUAGES CXX)

#--------------------------------------------
# Project setting
#--------------------------------------------
# include(cmake/FindPYLON.cmake)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)

message(INFO "--------------------------------")
message(STATUS "PROJECT_NAME: ${PROJECT_NAME}")
#aux_source_directory(. CURRENT_DIR_SRCS)
#message (STATUS "CURRENT_DIR_SRCS: ${CURRENT_DIR_SRCS}")
#add_executable(${PROJECT_NAME} ${CURRENT_DIR_SRCS})
#add_executable(${PROJECT_NAME} src/camera_basler.cpp)

add_library(${PROJECT_NAME} SHARED src/camera_basler.cpp)

find_package(OpenCV REQUIRED)
#if(OpenCV_FOUND)
	#target_link_libraries(${PROJECT_NAME} PUBLIC ${OpenCV_LIBS})	
#else()
	#message(FATAL_ERROR "[${PROJECT_NAME}]: OpenCV Not Found")
#endif(OpenCV_FOUND)


set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" CMAKE_MODULE_PATH)
find_package(PYLON REQUIRED)
if(PYLON_FOUND)
    message(STATUS "PYLON_INCLUDE_DIRS: ${PYLON_INCLUDE_DIRS}")
	message(STATUS "PYLON_LIBRARY: ${PYLON_LIBRARY}")
    # target_include_directories(${PROJECT_NAME} PUBLIC ${PYLON_INCLUDE_DIRS})
else()
    message(FATAL_ERROR "[${PROJECT_NAME}]: PYLON Not Found")
endif(PYLON_FOUND)

target_include_directories(${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME} rclcpp rclcpp_components std_srvs sensor_msgs)

target_link_libraries(${PROJECT_NAME} PYLON::PYLON )
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} gpiod)

rclcpp_components_register_node(${PROJECT_NAME} PLUGIN "camera_basler::CameraBasler" EXECUTABLE camera_basler_node)


install(TARGETS ${PROJECT_NAME} EXPORT export_camera_basler DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_camera_basler HAS_LIBRARY_TARGET)

ament_package()