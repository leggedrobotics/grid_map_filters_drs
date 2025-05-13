cmake_minimum_required(VERSION 3.0.2)
project(grid_map_filters_drs)
set (CMAKE_CXX_STANDARD 14)

set(DEFAULT_BUILD "Release")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
   message(STATUS "Setting build type to '${DEFAULT_BUILD}' as none was specified.")
   set(CMAKE_BUILD_TYPE ${DEFAULT_BUILD} CACHE STRING "Choose the type of build." FORCE)
   # Set the possible values of build type for cmake-gui
   set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug"
                                                        "Release"
                                                        "MinSizeRel"
                                                        "RelWithDebInfo")
endif()


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  filters
  geometry_msgs
  nav_msgs
  visualization_msgs
  sensor_msgs
  std_srvs
  pcl_ros
  pcl_conversions
  grid_map_core
  grid_map_filters
  grid_map_cv
  grid_map_msgs
  grid_map_ros
  message_generation
  eigen_conversions
  tf
  tf_conversions
)

# Libraries dependencies
# OpenCV
find_package(OpenCV REQUIRED)
# Eigen3
find_package(Eigen3 REQUIRED)
# Limbo for Gaussian Process regression (optional)
find_package(limbo CONFIG QUIET)
# FOr learned motion costs
find_package(gpu_path_optimizer CONFIG QUIET)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS  roscpp
                  std_msgs
                  filters
                  geometry_msgs
                  nav_msgs
                  visualization_msgs
                  sensor_msgs
                  pcl_ros
                  pcl_conversions
                  grid_map_core
                  grid_map_filters
                  grid_map_cv
                  grid_map_msgs
                  grid_map_ros
                  message_generation
                  eigen_conversions
                  tf
                  tf_conversions
  DEPENDS OpenCV
)

###########
## Build ##
###########
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${limbo_INCLUDE_DIRS}
)

# Files
set(FILTERS_SRC src/grid_map_filters_drs/BaseHeightThresholdFilter.cpp
                src/grid_map_filters_drs/ChangeNormalsFrameFilter.cpp
                src/grid_map_filters_drs/CropFilter.cpp
                src/grid_map_filters_drs/DeleteAllButFilter.cpp
                src/grid_map_filters_drs/DenoiseFilter.cpp
                src/grid_map_filters_drs/DenoiseAndInpaintFilter.cpp
                src/grid_map_filters_drs/FastNormalsVectorFilter.cpp
                src/grid_map_filters_drs/FootprintMaskFilter.cpp
                src/grid_map_filters_drs/GeodesicDistanceField2dFilter.cpp
                src/grid_map_filters_drs/GeodesicFieldFilter.cpp
                src/grid_map_filters_drs/InpaintFilter.cpp
                src/grid_map_filters_drs/NanFillerFilter.cpp
                src/grid_map_filters_drs/NanMaskFilter.cpp
                src/grid_map_filters_drs/PotentialGoalFieldFilter.cpp
                src/grid_map_filters_drs/RadialInpaintFilter.cpp
                src/grid_map_filters_drs/SignedDistanceField2dFilter.cpp
)

set(UTILS_SRC src/grid_map_filters_drs/utils/timer.cpp
              src/grid_map_filters_drs/utils/profiler.cpp
)

if (limbo_FOUND)
  set(FILTERS_SRC ${FILTERS_SRC}
                  src/grid_map_filters_drs/GaussianProcessInpaintFilter.cpp)
else()
  message(WARNING "limbo not found. Will not build GP Inpaint Filter. To enable GP Inpaint Filter, clone https://github.com/ori-drs/limbo into your workspace")
endif()

if (gpu_path_optimizer_FOUND)
  set(FILTERS_SRC ${FILTERS_SRC}
                  src/grid_map_filters_drs/LearnedMotionCostsFilter.cpp)
else()
  message(WARNING "gpu_path_optimizer not found. Will not build LearnedMotionCostsFilter.")
endif()

## Declare elevation_map_filters_drs library
add_library(${PROJECT_NAME} ${FILTERS_SRC}
                            ${UTILS_SRC})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# Declare elevation_map_filter_drs node
add_executable(grid_map_filter_drs_node   src/elevation_map_filter/elevation_map_filter_node.cpp
                                          src/elevation_map_filter/elevation_map_filter.cpp
                                          ${UTILS_SRC})
target_link_libraries(grid_map_filter_drs_node  ${catkin_LIBRARIES}
                                                ${OpenCV_LIBS})

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

# Mark other files for installation
install(
  FILES filter_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
