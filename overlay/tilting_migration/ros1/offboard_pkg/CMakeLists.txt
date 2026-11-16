cmake_minimum_required(VERSION 3.0.2)
project(offboard_pkg)

find_package(catkin REQUIRED COMPONENTS geometry_msgs mavros_msgs roscpp std_msgs)
catkin_package()

add_executable(eight src/eight.cpp)
target_compile_features(eight PRIVATE cxx_std_14)
target_include_directories(eight PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(eight ${catkin_LIBRARIES})

install(TARGETS eight RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
