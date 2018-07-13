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

add_library(poly_solver src/mpl_traj_solver/poly_solver.cpp
  src/mpl_traj_solver/poly_traj.cpp)

add_library(map_planner src/mpl_planner/map_planner.cpp)

add_library(ellipsoid_planner src/mpl_planner/ellipsoid_planner.cpp
  src/mpl_collision/decomp_util.cpp)
target_link_libraries(ellipsoid_planner ${PCL_LIBRARIES})

include(CTest)

add_executable(test_traj_solver test/test_traj_solver.cpp)
target_link_libraries(test_traj_solver poly_solver ${Boost_LIBRARIES})
add_test(test_traj_solver test_traj_solver)

add_executable(test_planner_2d test/test_planner_2d.cpp)
target_link_libraries(test_planner_2d map_planner ${YAMLCPP_LIBRARIES} ${Boost_LIBRARIES})
add_test(test_planner_2d test_planner_2d ${CMAKE_SOURCE_DIR}/data/corridor.yaml)

add_executable(test_planner_2d_prior_traj test/test_planner_2d_prior_traj.cpp)
target_link_libraries(test_planner_2d_prior_traj map_planner ${YAMLCPP_LIBRARIES} ${Boost_LIBRARIES})
add_test(test_planner_2d_prior_traj test_planner_2d_prior_traj ${CMAKE_SOURCE_DIR}/data/corridor.yaml)

add_executable(test_planner_2d_with_yaw test/test_planner_2d_with_yaw.cpp)
target_link_libraries(test_planner_2d_with_yaw map_planner ${YAMLCPP_LIBRARIES} ${Boost_LIBRARIES})
add_test(test_planner_2d_with_yaw test_planner_2d_with_yaw ${CMAKE_SOURCE_DIR}/data/corridor.yaml)

add_executable(test_distance_map_planner_2d test/test_distance_map_planner_2d.cpp)
target_link_libraries(test_distance_map_planner_2d map_planner ${YAMLCPP_LIBRARIES} ${Boost_LIBRARIES})
add_test(test_distance_map_planner_2d test_distance_map_planner_2d ${CMAKE_SOURCE_DIR}/data/corridor.yaml)

install(FILES "${PROJECT_NAME}Config.cmake" "${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION "share/${PROJECT_NAME}/cmake")

install(TARGETS poly_solver map_planner ellipsoid_planner
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(DIRECTORY include/mpl_basis include/mpl_traj_solver include/mpl_planner include/mpl_collision
  DESTINATION include
  )
