cmake_minimum_required(VERSION 3.28.0)
list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/drivers/lsm6dsl_custom)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tremor_detection)
target_sources(app PRIVATE
	src/main.c
	src/episode_fsm.c
	src/ble_service.c
	src/dsp/pipeline.c
)
target_include_directories(app PRIVATE src)
