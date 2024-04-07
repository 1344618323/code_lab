cmake_minimum_required(VERSION 3.10)
find_package(OpenCV REQUIRED)

pybind11_add_module(common common.cpp)
target_include_directories(common PUBLIC
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(common PUBLIC
    ${OpenCV_LIBRARIES}
)

install(TARGETS common DESTINATION .)
