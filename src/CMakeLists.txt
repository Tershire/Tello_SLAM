add_library(tello_slam SHARED
    camera/camera.cpp
    camera/brown_conrady.cpp
    camera/pinhole.cpp
    data_types/feature.cpp
    data_types/frame.cpp
    data_types/landmark.cpp
    door/door_detector.cpp
    imu/imu_reader.cpp
    marker/aruco_detector.cpp
    port/config.cpp
    port/setting.cpp
    tool/supervisor.cpp
    tool/tool_system.cpp
    backend.cpp
    frontend.cpp
    map.cpp
    system.cpp
    viewer.cpp)

target_link_libraries(tello_slam 
    ${THIRD_PARTY_LIBS})
