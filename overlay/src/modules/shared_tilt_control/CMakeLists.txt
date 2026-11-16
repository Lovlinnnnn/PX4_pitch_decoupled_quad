px4_add_module(
	MODULE modules__shared_tilt_control
	MAIN shared_tilt_control
	STACK_MAIN 2500
	SRCS SharedTiltBench.cpp FlightOuterLoop.cpp
	DEPENDS px4_work_queue shared_tilt_control Takeoff
)
