cmake_minimum_required(VERSION 3.8)
project(architecture_measurement)

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

find_package(OpenCV REQUIRED)
find_package(PCL 1.3 REQUIRED)
# find_package(PCL 1.3 REQUIRED COMPONENTS common)
find_package(yaml-cpp REQUIRED)  
find_package(Eigen3 3.3.7 REQUIRED NO_MODULE)

#link pcl
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

message("${PROJECT_NAME}_SOURCE_DIR: ${${PROJECT_NAME}_SOURCE_DIR}")

set(IMPL_SRC 
  ${${PROJECT_NAME}_SOURCE_DIR}/src/impl/file_io.cpp 
  ${${PROJECT_NAME}_SOURCE_DIR}/src/impl/types.cpp ${${PROJECT_NAME}_SOURCE_DIR}/src/impl/space_analytic_geometry.cpp
  ${${PROJECT_NAME}_SOURCE_DIR}/src/impl/cloud_pretreatment.cpp
  ${${PROJECT_NAME}_SOURCE_DIR}/src/impl/cloud_header.cpp
  ${${PROJECT_NAME}_SOURCE_DIR}/src/impl/fundamental.cpp)

  set(IMPL_SRC_EX
    ${${PROJECT_NAME}_SOURCE_DIR}/src/impl/file_io.cpp
    # ${${PROJECT_NAME}_SOURCE_DIR}/src/impl/types.cpp ${${PROJECT_NAME}_SOURCE_DIR}/src/impl/space_analytic_geometry.cpp
    ${${PROJECT_NAME}_SOURCE_DIR}/src/impl/fundamental.cpp
    ${${PROJECT_NAME}_SOURCE_DIR}/src/impl/inclinometer.cpp
    ${${PROJECT_NAME}_SOURCE_DIR}/src/impl/point_cloud_gui.cpp
    ${${PROJECT_NAME}_SOURCE_DIR}/src/impl/cloud_pretreatment_ex.cpp
    ${${PROJECT_NAME}_SOURCE_DIR}/src/impl/cloud_header_ex.cpp)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shared_interfaces REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

#floor height
add_library(floor_height SHARED  src/floor_height.cpp ${IMPL_SRC})

target_include_directories(floor_height
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(floor_height
  rclcpp
  rclcpp_components
  std_srvs
  sensor_msgs
  shared_interfaces
  nlohmann_json
)
target_link_libraries(floor_height ${PCL_LIBRARIES} ${OpenCV_LIBS} pthread yaml-cpp Eigen3::Eigen)
# target_link_libraries(floor_height ${PCL_LIBRARIES} file_io space_analytic_geometry cloud_header cloud_pretreatment)

rclcpp_components_register_node(floor_height
  PLUGIN "am::FloorHeight"
  EXECUTABLE floor_height_node
)
# message("floor_height finished")

#internal_and_external_angle
add_library(internal_and_external_angle SHARED  src/internal_and_external_angle.cpp ${IMPL_SRC})

target_include_directories(internal_and_external_angle
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(internal_and_external_angle
  rclcpp
  rclcpp_components
  std_srvs
  sensor_msgs
  shared_interfaces
  nlohmann_json
)
target_link_libraries(internal_and_external_angle ${PCL_LIBRARIES} ${OpenCV_LIBS} pthread yaml-cpp Eigen3::Eigen)
# target_link_libraries(internal_and_external_angle ${PCL_LIBRARIES} file_io space_analytic_geometry cloud_header cloud_pretreatment)

rclcpp_components_register_node(internal_and_external_angle
  PLUGIN "am::InternalAndExternalAngle"
  EXECUTABLE internal_and_external_angle_node
)
# message("internal_and_external_angle finished")

#perpendicularity
add_library(perpendicularity SHARED  src/perpendicularity.cpp ${IMPL_SRC})

target_include_directories(perpendicularity
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(perpendicularity
  rclcpp
  rclcpp_components
  std_srvs
  sensor_msgs
  shared_interfaces
  nlohmann_json
)
target_link_libraries(perpendicularity ${PCL_LIBRARIES} ${OpenCV_LIBS} pthread yaml-cpp Eigen3::Eigen)
# target_link_libraries(perpendicularity_and_levelness ${PCL_LIBRARIES} file_io space_analytic_geometry cloud_header cloud_pretreatment)

rclcpp_components_register_node(perpendicularity
  PLUGIN "am::Perpendicularity"
  EXECUTABLE perpendicularity_node
)
# message("perpendicularity_and_levelness finished")

#levelness
add_library(levelness SHARED  src/levelness.cpp ${IMPL_SRC})

target_include_directories(levelness
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(levelness
  rclcpp
  rclcpp_components
  std_srvs
  sensor_msgs
  shared_interfaces
  nlohmann_json
)
target_link_libraries(levelness ${PCL_LIBRARIES} ${OpenCV_LIBS} pthread yaml-cpp Eigen3::Eigen)
# target_link_libraries(perpendicularity_and_levelness ${PCL_LIBRARIES} file_io space_analytic_geometry cloud_header cloud_pretreatment)

rclcpp_components_register_node(levelness
  PLUGIN "am::Levelness"
  EXECUTABLE levelness_node
)
# message("perpendicularity_and_levelness finished")

#pillar_section_size
add_library(pillar_section_size SHARED  src/pillar_section_size.cpp ${IMPL_SRC})

target_include_directories(pillar_section_size
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(pillar_section_size
  rclcpp
  rclcpp_components
  std_srvs
  sensor_msgs
  shared_interfaces
  nlohmann_json
)
target_link_libraries(pillar_section_size ${PCL_LIBRARIES} ${OpenCV_LIBS} pthread yaml-cpp Eigen3::Eigen)
# target_link_libraries(pillar_section_size ${PCL_LIBRARIES} file_io space_analytic_geometry cloud_header cloud_pretreatment)

rclcpp_components_register_node(pillar_section_size
  PLUGIN "am::PillarSectionSize"
  EXECUTABLE pillar_section_size_node
)
# message("pillar_section_size finished")

#surface_flatness
add_library(surface_flatness SHARED  src/surface_flatness.cpp ${IMPL_SRC})

target_include_directories(surface_flatness
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(surface_flatness
  rclcpp
  rclcpp_components
  std_srvs
  sensor_msgs
  shared_interfaces
  nlohmann_json
)
target_link_libraries(surface_flatness ${PCL_LIBRARIES} ${OpenCV_LIBS} pthread yaml-cpp Eigen3::Eigen)
# target_link_libraries(surface_flatness ${PCL_LIBRARIES} file_io space_analytic_geometry cloud_header cloud_pretreatment)

rclcpp_components_register_node(surface_flatness
  PLUGIN "am::SurfaceFlatness"
  EXECUTABLE surface_flatness_node
)
# message("surface_flatness finished")

#masonry_surface_flatness
add_library(masonry_surface_flatness SHARED  src/masonry_surface_flatness_node.cpp src/masonry_surface_flatness.cpp src/masonry.cpp  ${IMPL_SRC_EX})

target_include_directories(masonry_surface_flatness
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(masonry_surface_flatness
  rclcpp
  rclcpp_components
  std_srvs
  sensor_msgs
  shared_interfaces
  nlohmann_json
)
target_link_libraries(masonry_surface_flatness ${PCL_LIBRARIES} ${OpenCV_LIBS} pthread yaml-cpp Eigen3::Eigen)
# target_link_libraries(surface_flatness ${PCL_LIBRARIES} file_io space_analytic_geometry cloud_header cloud_pretreatment)

rclcpp_components_register_node(masonry_surface_flatness
  PLUGIN "am::MasonrySurfaceFlatnessNode"
  EXECUTABLE masonry_surface_flatness_node
)
# message("surface_flatness finished")

#masonry_perpendicularity
add_library(masonry_perpendicularity SHARED  src/masonry_perpendicularity_node.cpp src/masonry_perpendicularity.cpp src/masonry.cpp  ${IMPL_SRC_EX})

target_include_directories(masonry_perpendicularity
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(masonry_perpendicularity
  rclcpp
  rclcpp_components
  std_srvs
  sensor_msgs
  shared_interfaces
  nlohmann_json
)
target_link_libraries(masonry_perpendicularity ${PCL_LIBRARIES} ${OpenCV_LIBS} pthread yaml-cpp Eigen3::Eigen)
# target_link_libraries(surface_flatness ${PCL_LIBRARIES} file_io space_analytic_geometry cloud_header cloud_pretreatment)

rclcpp_components_register_node(masonry_perpendicularity
  PLUGIN "am::MasonryPerpendicularityNode"
  EXECUTABLE masonry_perpendicularity_node
)
# message("surface_flatness finished")

#opening size
add_library(opening_size SHARED src/opening_size.cpp ${IMPL_SRC_EX})

target_include_directories(opening_size
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(opening_size
  rclcpp
  rclcpp_components
  std_srvs
  sensor_msgs
  shared_interfaces
  nlohmann_json
)
target_link_libraries(opening_size ${PCL_LIBRARIES} ${OpenCV_LIBS} pthread yaml-cpp Eigen3::Eigen FSMeasure)
# target_link_libraries(surface_flatness ${PCL_LIBRARIES} file_io space_analytic_geometry cloud_header cloud_pretreatment)

rclcpp_components_register_node(opening_size
  PLUGIN "am::OpeningSize"
  EXECUTABLE opening_size_node
)


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

# install(TARGETS file_io DESTINATION lib)
# install(TARGETS space_analytic_geometry DESTINATION lib)
# install(TARGETS cloud_header DESTINATION lib)
# install(TARGETS cloud_pretreatment DESTINATION lib)

install(TARGETS floor_height EXPORT export_floor_height DESTINATION lib)
install(TARGETS internal_and_external_angle EXPORT export_internal_and_external_angle DESTINATION lib)
install(TARGETS perpendicularity EXPORT export_perpendicularity DESTINATION lib)
install(TARGETS levelness EXPORT export_levelness DESTINATION lib)
install(TARGETS pillar_section_size EXPORT export_pillar_section_size DESTINATION lib)
install(TARGETS surface_flatness EXPORT export_surface_flatness DESTINATION lib)
install(TARGETS opening_size EXPORT export_opening_size DESTINATION lib)
install(TARGETS masonry_perpendicularity EXPORT export_masonry_perpendicularity DESTINATION lib)
install(TARGETS masonry_surface_flatness EXPORT export_masonry_surface_flatness DESTINATION lib)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_floor_height HAS_LIBRARY_TARGET)
ament_export_targets(export_internal_and_external_angle HAS_LIBRARY_TARGET)
ament_export_targets(export_perpendicularity HAS_LIBRARY_TARGET)
ament_export_targets(export_levelness HAS_LIBRARY_TARGET)
ament_export_targets(export_pillar_section_size HAS_LIBRARY_TARGET)
ament_export_targets(export_surface_flatness HAS_LIBRARY_TARGET)
ament_export_targets(export_opening_size HAS_LIBRARY_TARGET)
ament_export_targets(export_masonry_perpendicularity HAS_LIBRARY_TARGET)
ament_export_targets(export_masonry_surface_flatness HAS_LIBRARY_TARGET)

ament_package()
