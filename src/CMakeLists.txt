add_library(tello_slam SHARED
    camera/camera.cpp
    camera/brown_conrady.cpp
    camera/pinhole.cpp
    door/door_detector.cpp
    imu/imu_reader.cpp
    marker/aruco_detector.cpp
    port/config.cpp
    port/setting.cpp
    tool/supervisor.cpp
    tool/tool_system.cpp
    system.cpp)

target_link_libraries(tello_slam 
    ${THIRD_PARTY_LIBS})
