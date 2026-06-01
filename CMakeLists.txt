cmake_minimum_required(VERSION 3.15)

project(control_algo_lib C)

add_library(control_algo STATIC
	src/clarke.c
	src/component.c
	src/current_loop.c
	src/dq_control.c
	src/feedforward.c
	src/fundamental_curr_detect.c
	src/fundamental_curr_ctrl.c
	src/harmonic_detect.c
	src/harmonic_control.c
	src/init.c
	src/inverse_clarke.c
	src/inverse_park.c
	src/isr.c
	src/lpf.c
	src/math.c
	src/mean.c
	src/open_loop.c
	src/park.c
	src/pi_design.c
	src/pid.c
	src/pll.c
	src/sequence.c
	src/spwm.c
	src/Vbus_volt_ctrl.c
)

target_include_directories(control_algo PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/include
)

set_target_properties(control_algo PROPERTIES
	C_STANDARD 99
	C_STANDARD_REQUIRED YES
	OUTPUT_NAME control_algo
)

if(MSVC)
	target_compile_options(control_algo PRIVATE /W3)
else()
	target_compile_options(control_algo PRIVATE -Wall -Wextra)
	target_link_libraries(control_algo PUBLIC m)
endif()
