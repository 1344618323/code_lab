add_library(slam_lab_common
    common/io_utils.cpp
    common/static_imu_init.cpp
    common/utm_convert/utm.cpp
    common/utm_convert/tranmerc.cpp
    common/utm_convert.cpp
    common/eskf.cpp
    common/imu_preintegration.cpp
    common/g2o_types.cpp
    common/gins_preintegration.cpp
)
target_link_libraries(slam_lab_common
    ${third_party_libs}
)

add_executable(rotation
    rotation.cpp
)
target_link_libraries(rotation
    Sophus::Sophus
)

add_executable(imu_integration
    imu_integration.cpp
)
target_link_libraries(imu_integration
    slam_lab_common
)

add_executable(eskf_gins
    eskf_gins.cpp
)
target_link_libraries(eskf_gins
    slam_lab_common
)

add_executable(run_gins_preintegration
    run_gins_preintegration.cpp
)
target_link_libraries(run_gins_preintegration
    slam_lab_common
)
