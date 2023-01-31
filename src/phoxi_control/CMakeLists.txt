cmake_minimum_required(VERSION 3.8)

if(POLICY CMP0054)
    cmake_policy(SET CMP0054 NEW)
endif()

project(phoxi_control)

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

if (NOT PHO_API_CMAKE_CONFIG_PATH)
    set(PHO_API_CMAKE_CONFIG_PATH "$ENV{PHOXI_CONTROL_PATH}")
endif()

# set(CMAKE_RELEASE_POSTFIX "_Release")
# set(CMAKE_DEBUG_POSTFIX "_Debug")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shared_interfaces REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)

list(APPEND CMAKE_PREFIX_PATH "${PHO_API_CMAKE_CONFIG_PATH}")
find_package(PhoXi REQUIRED CONFIG PATHS "${PHO_API_CMAKE_CONFIG_PATH}")
find_package(OpenCV REQUIRED)
find_package(PCL 1.3 REQUIRED COMPONENTS common segmentation)
find_package(yaml-cpp REQUIRED)  
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(phoxi_control SHARED src/phoxi_control.cpp)
target_include_directories(phoxi_control
  PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
    $<INSTALL_INTERFACE:include>
    ${PHOXI_INCLUDE_DIRS})
ament_target_dependencies(phoxi_control
    rclcpp 
    rclcpp_components 
    std_srvs 
    std_msgs
    sensor_msgs
    shared_interfaces 
    nlohmann_json)

message("phoxi_library: ${PHOXI_LIBRARY}")
target_link_libraries(phoxi_control
    ${PHOXI_LIBRARY}
    $<$<PLATFORM_ID:Linux>:rt>
)

if(opencv_world IN_LIST OpenCV_LIBS)
 target_link_libraries(phoxi_control opencv_world)
else()
 target_link_libraries(phoxi_control ${OpenCV_LIBS})
endif()

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
target_link_libraries(phoxi_control ${PCL_LIBRARIES})

target_link_libraries(phoxi_control yaml-cpp)

rclcpp_components_register_node(phoxi_control
  PLUGIN "phoxi_control::PhoXiControl" 
  EXECUTABLE phoxi_control_node)

get_target_property(ReleaseBinFolder phoxi_control RUNTIME_OUTPUT_DIRECTORY_RELEASE)
get_target_property(DebugBinFolder phoxi_control RUNTIME_OUTPUT_DIRECTORY_DEBUG)

if(MSVC)
    add_custom_command(TARGET phoxi_control POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<$<CONFIG:Release>:${PHOXI_DLL_RELEASE}>
            $<$<CONFIG:Debug>:${PHOXI_DLL_DEBUG}>
            $<TARGET_FILE_DIR:phoxi_control>
    )
endif(MSVC)

# set_target_properties(phoxi_control
#     PROPERTIES
#     #for executables, inheritance of variables CMAKE_DEBUG_POSTFIX... does not work
#     DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX}
#     RELEASE_POSTFIX ${CMAKE_RELEASE_POSTFIX}
# )

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

install(TARGETS phoxi_control EXPORT export_phoxi_control DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_phoxi_control HAS_LIBRARY_TARGET)

ament_package()


