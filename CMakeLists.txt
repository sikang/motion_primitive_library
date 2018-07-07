cmake_minimum_required(VERSION 2.8)
project(motion_primitive_library)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-deprecated-declarations")

set(BUILD_SHARED_LIBS ON)

find_package(PCL 1.7 REQUIRED)
if(NOT "${PCL_LIBRARIES}" STREQUAL "")
  list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
endif()
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
PKG_CHECK_MODULES(YAMLCPP REQUIRED yaml-cpp)
include_directories(${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

if(YAMLCPP_FOUND)
  message(STATUS "Found yaml-cpp.")
else()
  message(FATAL_ERROR "Could not find yaml-cpp.")
endif()

add_library(poly_solver src/poly_solver/poly_solver.cpp
  src/poly_solver/poly_traj.cpp)

add_library(mp_base_util
  src/planner/mp_base_util.cpp
  src/planner/graph_search.cpp
  src/planner/state_space.cpp)

add_library(mp_map_util src/planner/mp_map_util.cpp)
target_link_libraries(mp_map_util mp_base_util)

add_library(mp_cloud_util src/planner/mp_cloud_util.cpp
  src/collision_checking/decomp_util.cpp)
target_link_libraries(mp_cloud_util mp_base_util ${PCL_LIBRARIES})

include(CTest)

add_executable(test_planner_2d test/test_planner_2d.cpp)
target_link_libraries(test_planner_2d mp_map_util ${YAMLCPP_LIBRARIES} ${Boost_LIBRARIES})
add_test(test_planner_2d test_planner_2d ${CMAKE_SOURCE_DIR}/data/corridor.yaml)

add_executable(test_planner_2d_prior_traj test/test_planner_2d_prior_traj.cpp)
target_link_libraries(test_planner_2d_prior_traj mp_map_util ${YAMLCPP_LIBRARIES} ${Boost_LIBRARIES})
add_test(test_planner_2d_prior_traj test_planner_2d_prior_traj ${CMAKE_SOURCE_DIR}/data/corridor.yaml)

install(FILES "${PROJECT_NAME}Config.cmake" "${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION "share/${PROJECT_NAME}/cmake")

install(TARGETS poly_solver mp_base_util mp_map_util mp_cloud_util
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME})
