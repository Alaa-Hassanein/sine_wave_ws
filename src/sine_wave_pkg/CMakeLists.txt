cmake_minimum_required(VERSION 3.8)
project(sine_wave_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(ament_cmake_gtest REQUIRED)



set(INSTALL_DIR ${CMAKE_INSTALL_PREFIX})

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  std_msgs

)

generate_parameter_library(
  sine_wave_parameters
  params/sine_wave_prams_sturcture.yaml

)

include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${generate_parameter_library_INCLUDE_DIRS}
)


rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetImage.srv"
  DEPENDENCIES sensor_msgs
)

add_executable(sine_wave_publisher_cpp src/sine_wave_publisher.cpp)
ament_target_dependencies(sine_wave_publisher_cpp PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(sine_wave_publisher_cpp PUBLIC sine_wave_parameters)




install(TARGETS sine_wave_publisher_cpp 
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/gray_image_server.py
  scripts/gray_image_client.py
  scripts/sine_wave_publisher.py
  scripts/sine_wave_subscriber.py
  test/test_sine_wave_publisher.py
  
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

file(MAKE_DIRECTORY ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME})
add_custom_command(
  OUTPUT ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/sine_wave_parameters.py
  COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_parameter.py >
  ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/sine_wave_parameters.py
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "Running generate_parameter.py to generate params.py"
)

ament_export_include_directories(include)

add_custom_target(run_python_script ALL DEPENDS
  ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/sine_wave_parameters.py)

file(MAKE_DIRECTORY ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME})

ament_package()
