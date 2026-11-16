cmake_minimum_required(VERSION 3.16)
project(px4_gazebo_tilt_servo_plugin LANGUAGES CXX)

find_package(gazebo 11 REQUIRED)
if(NOT gazebo_VERSION VERSION_LESS 12)
	message(FATAL_ERROR "The tilt servo plugin requires Gazebo Classic 11")
endif()

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs std_msgs)

add_library(gazebo_tilt_servo_plugin SHARED gazebo_tilt_servo_plugin.cpp)
target_compile_features(gazebo_tilt_servo_plugin PRIVATE cxx_std_17)
target_include_directories(gazebo_tilt_servo_plugin SYSTEM PRIVATE ${GAZEBO_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
target_link_directories(gazebo_tilt_servo_plugin PRIVATE ${GAZEBO_LIBRARY_DIRS})
target_link_libraries(gazebo_tilt_servo_plugin PRIVATE ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
set_target_properties(gazebo_tilt_servo_plugin PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}")

if(GAZEBO_CXX_FLAGS)
	separate_arguments(gazebo_cxx_flags NATIVE_COMMAND "${GAZEBO_CXX_FLAGS}")
	# CMake selects C++17 for this target, including on Gazebo packages that export -std=c++11.
	list(FILTER gazebo_cxx_flags EXCLUDE REGEX "^-std=")
	target_compile_options(gazebo_tilt_servo_plugin PRIVATE ${gazebo_cxx_flags})
endif()
