cmake_minimum_required(VERSION 3.5)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ecompass)

# Add components
set(EXTRA_COMPONENT_DIRS components/lsm6dsox components/lis3mdl components/sensors)

# Include driver component
idf_build_set_property(COMPILE_OPTIONS "-I$ENV{IDF_PATH}/components/driver/include" APPEND)
