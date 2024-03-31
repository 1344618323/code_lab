cmake_minimum_required(VERSION 3.10)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 17)

# Make (almost) all warnings into errors.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-unused-but-set-variable -Werror")

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
message(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV_LIBRARIES: ${OpenCV_LIBRARIES}")
message(STATUS "OpenCV_VERSION: ${OpenCV_VERSION}")

# Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
message(STATUS "EIGEN3_INCLUDE_DIR: ${EIGEN3_INCLUDE_DIR}")
message(STATUS "EIGEN3_VERSION: ${EIGEN3_VERSION}")

# pcl
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

# gflag
find_package(gflags REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        sensor_msgs
        pcl_ros
        pcl_conversions
        )
include_directories(${catkin_INCLUDE_DIRS})

# Sophus
# `NO_DEFAULT_PATH` tells CMake not to search for the package from the system default path
# `PATHS` points to the path where we install third-party libraries
find_package(Sophus REQUIRED NO_DEFAULT_PATH PATHS ${PROJECT_SOURCE_DIR}/Thirdparty/Thirdparty_install/)

# Pangolin
find_package(Pangolin REQUIRED NO_DEFAULT_PATH PATHS ${PROJECT_SOURCE_DIR}/Thirdparty/Thirdparty_install/)
message(STATUS "Pangolin_LIBRARIES: ${Pangolin_LIBRARIES}")

# Ceres
find_package(Ceres REQUIRED NO_DEFAULT_PATH PATHS ${PROJECT_SOURCE_DIR}/Thirdparty/Thirdparty_install/)
include_directories(${CERES_INCLUDE_DIRS})

# g2o
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/cmake_modules)
SET(G2O_ROOT ${PROJECT_SOURCE_DIR}/Thirdparty/Thirdparty_install)
FIND_PACKAGE(G2O REQUIRED)
# find_package(G2O REQUIRED NO_DEFAULT_PATH PATHS ${PROJECT_SOURCE_DIR}/Thirdparty/Thirdparty_install/)  don't work!
# refer to https://github.com/rubengooj/pl-slam/issues/1, we can know that while installation, g2o do not 
# copy the `FindG2O.cmake` into the target installation path. We add `FindG2O.cmake` by `CMAKE_MODULE_PATH`.
# And `FindG2O.cmake` will find include/lib based on the guidance of `G2O_ROOT`

include_directories(${G2O_INCLUDE_DIR})
message(STATUS "G2O_INCLUDE_DIR: ${G2O_INCLUDE_DIR}")
message(STATUS "G2O_LIBRARY: ${G2O_CORE_LIBRARY}, ${G2O_STUFF_LIBRARY}")

set(third_party_libs
    ${OpenCV_LIBRARIES}
    Sophus::Sophus
    ${Pangolin_LIBRARIES}
    ${CERES_LIBRARIES}
    ${G2O_CORE_LIBRARY}
    ${G2O_STUFF_LIBRARY}
    ${PCL_LIBRARIES}
    gflags
    # ${catkin_LIBRARIES} # link ros lib will cost much time
)

add_executable(main 
    main.cpp
)
target_link_libraries(main
    ${third_party_libs}
)

add_subdirectory(slam_lab)
add_subdirectory(vo_lab)