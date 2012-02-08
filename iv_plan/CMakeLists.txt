cmake_minimum_required(VERSION 2.8)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR}/externals make RESULT_VARIABLE _make_failed)
if (_make_failed)
  message(FATAL_ERROR "Build of iv_plan/externals failed")
endif(_make_failed)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_boost_directories()
rosbuild_add_library(cpqp src/cpqp.cpp)
include_directories(/usr/include/python2.6)
include_directories(${PROJECT_SOURCE_DIR}/externals/PQP_v1.3/include)
link_directories()
target_link_libraries(cpqp boost_python ${PROJECT_SOURCE_DIR}/externals/PQP_v1.3/lib/libPQP.a)

#add_subdirectory(src)
