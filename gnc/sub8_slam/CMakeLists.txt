cmake_minimum_required(VERSION 2.8.3)
project(sub8_slam)
# sudo apt-get install mathgl libmgl-dev
# sudo apt-get install ros-indigo-sophus
# Currently depending on --ros-indigo-sparse-bundle-adjustment
# Should consider using libg2o instead (but documentation sparsity is not the sparsity I was looking for!)
# g2o -- Not yet!
# Require C++11
# [1] http://answers.ros.org/question/152276/is-there-a-way-to-enable-c11-support-for-catkin-packages/
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11 -fext-numeric-literals -g -rdynamic")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()
add_definitions(-std=gnu++11 -lmgl)
add_definitions ("-Wall")

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")


find_package(catkin REQUIRED COMPONENTS
  roscpp sba
)
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)
find_package(MathGL REQUIRED)
find_package(Eigen REQUIRED)
find_package(sba REQUIRED)
find_package(sophus REQUIRED)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sub8_slam
  CATKIN_DEPENDS roscpp
  DEPENDS Eigen OpenCV Boost system_lib MathGL sba_vis sba
)

include_directories(
  include
  ${EIGEN_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${MathGL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)


# TODO Don't list the files like a schmuck
# TODO Make sub8_slam a library
set(slam_sources
  src/tracking
  src/motion
  src/visualization3d
  src/visualization2d
  src/frame
  src/sba
)

add_executable(slam src/slam
  ${slam_sources}
  include/backward.cpp
)

add_executable(slam ${slam_sources} include/backward.cpp)

target_link_libraries(slam
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  # ${Boost_LIBRARIES}
  ${CERES_LIBRARIES}
  bfd
  dl
)

# add_executable(toy analysis/toy ${slam_sources} include/backward.cpp)
# target_link_libraries(toy

set(ptam_sources
  src/ptam
  src/ptam_map
  src/posetracker
  src/tracking
  src/motion
  src/visualization3d
  src/visualization2d
  src/frame
  src/matching
  # src/sba
)

add_executable(ptam ${ptam_sources} include/backward.cpp)

target_link_libraries(ptam
  ${OpenCV_LIBS} # maybe _LIBRARIES
  ${MathGL_LIBRARIES}
  ${catkin_LIBRARIES}
  # ${Boost_LIBRARIES}
  ${SBA_LIB}
  bfd
  dl
)


# Not a fan of fpermissive - but SBA won't link in C++11 without it
set(CMAKE_CXX_FLAGS "-std=c++11 -fext-numeric-literals -fpermissive -g -rdynamic")

# add_executable(sba_demo analysis/sba_test analysis/visualization include/backward.cpp)
# target_link_libraries(sba_demo
#   ${SBA_LIB}
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBS}
#   ${CERES_LIBRARIES}
#   bfd
#   dl
# )
