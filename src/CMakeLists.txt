add_library(tello_slam SHARED
    camera/camera.cpp
    camera/brown_conrady.cpp
    camera/pinhole.cpp
    marker/aruco_detector.cpp
    port/config.cpp
    port/setting.cpp
    system.cpp)

target_link_libraries(tello_slam 
    ${THIRD_PARTY_LIBS})
