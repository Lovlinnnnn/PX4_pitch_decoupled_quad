px4_add_library(shared_tilt_control
	SharedTiltControl.cpp
	Allocation.cpp
)

target_include_directories(shared_tilt_control PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

px4_add_unit_gtest(SRC SharedTiltControlTest.cpp LINKLIBS shared_tilt_control)
