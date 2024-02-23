cmake_minimum_required(VERSION 3.10)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

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

# Sophus
find_package(Sophus REQUIRED)

# Pangolin
find_package(Pangolin REQUIRED)

# Ceres
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

# g2o
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

# pcl
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

# gflag
find_package(gflags REQUIRED)

# find_package(Qt5 REQUIRED COMPONENTS Core)

set(third_party_libs
    ${OpenCV_LIBRARIES}
    Sophus::Sophus
    ${Pangolin_LIBRARIES}
    ${CERES_LIBRARIES}
    ${G2O_CORE_LIBRARY}
    ${G2O_STUFF_LIBRARY}
    ${PCL_LIBRARIES}
    gflags
    # Qt5::Core
)

add_executable(main 
    main.cpp
)
target_link_libraries(main
    ${third_party_libs}
)

add_subdirectory(slam_lab)