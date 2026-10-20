cmake_minimum_required(VERSION 3.10)
project(podracer C)

# podracer shares antigrav's host types (raycast, route, pose); only the
# header is needed to build the library
add_library(podracer STATIC podracer.c podracer.h)
target_include_directories(podracer PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../antigrav)
set_target_properties(podracer PROPERTIES C_STANDARD 99 C_STANDARD_REQUIRED ON)
if(NOT MSVC)
	target_link_libraries(podracer PUBLIC m)
endif()

# Build the simulation tests by default only when this is the top level project
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
	set(PODRACER_TESTS_DEFAULT ON)
else()
	set(PODRACER_TESTS_DEFAULT OFF)
endif()
option(PODRACER_BUILD_TESTS "Build the podracer simulation tests" ${PODRACER_TESTS_DEFAULT})

if(PODRACER_BUILD_TESTS)
	enable_testing()
	# The tests reuse antigrav's test geometry and BVH raycaster
	add_executable(podracer_sim_test test/sim_test.c ../antigrav/test/test_world.c ../antigrav/antigrav_mesh.c)
	target_link_libraries(podracer_sim_test PRIVATE podracer)
	set_target_properties(podracer_sim_test PROPERTIES C_STANDARD 99)
	add_test(NAME podracer_sim_test COMMAND podracer_sim_test)
endif()
