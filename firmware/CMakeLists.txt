cmake_minimum_required(VERSION 3.16)

set(IDF_TARGET "esp32c5")
set(EXTRA_COMPONENT_DIRS src)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Use custom partition table
set(PARTITION_CSV_PATH "${CMAKE_SOURCE_DIR}/partitions.csv")

# Exclude json component since we don't use it
set(EXCLUDE_COMPONENTS "json" CACHE STRING "")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(firmware)