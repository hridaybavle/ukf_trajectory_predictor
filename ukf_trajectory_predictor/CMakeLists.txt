cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME ukf_trajectory_predictor)
project(${PROJECT_NAME})

set(CMAKE_CXX_FLAGS "-std=c++11")

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
#SET(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo

set(CMAKE_BUILD_TYPE "RELEASE")

set(UKF_TRAJ_PRED_SOURCE_DIR
    src/sources)

set(UKF_TRAJ_PRED_INCLUDE_DIR
    src/include
    )

FILE(GLOB_RECURSE UKF_TRAJ_PRED_SOURCE_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/source/*.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/src/source/ukf/*.cpp)

FILE(GLOB_RECURSE UKF_TRAJ_PRED_HEADER_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/include/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/include/*.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/include/ukf/*.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/include/ukf/*.h)


find_package(catkin REQUIRED
    COMPONENTS roscpp std_msgs sensor_msgs tf_conversions tf cv_bridge robot_process)


find_package(Eigen3)
if(NOT EIGEN3_FOUND)
    # Fallback to cmake_modules
    find_package(cmake_modules REQUIRED)
    find_package(Eigen REQUIRED)
    set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
    set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
    # Possibly map additional variables to the EIGEN3_ prefix.
    message(WARN "Using Eigen2!")
else()
    set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

catkin_package(
    INCLUDE_DIRS ${UKF_TRAJ_PRED_INCLUDE_DIR}
    LIBRARIES ukf_trajectory_predictor_lib
    CATKIN_DEPENDS roscpp std_msgs sensor_msgs tf_conversions tf cv_bridge robot_process
    )

include_directories(${UKF_TRAJ_PRED_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIRS})

add_library(ukf_trajectory_predictor_lib ${UKF_TRAJ_PRED_SOURCE_FILES} ${UKF_TRAJ_PRED_HEADER_FILES})
add_dependencies(ukf_trajectory_predictor_lib ${PROJECT_NAME}_gencpp)
add_dependencies(ukf_trajectory_predictor_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(ukf_trajectory_predictor_lib ${catkin_LIBRARIES})
target_link_libraries(ukf_trajectory_predictor_lib ${EIGEN3_LIBRARIES})

add_executable(ukf_trajectory_predictor_node src/test/ukf_trajectory_predictor_node.cpp)
add_dependencies(ukf_trajectory_predictor_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(ukf_trajectory_predictor_node ukf_trajectory_predictor_lib)
target_link_libraries(ukf_trajectory_predictor_node ${catkin_LIBRARIES})
