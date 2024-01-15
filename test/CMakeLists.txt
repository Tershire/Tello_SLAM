add_executable(detect_aruco detect_aruco.cpp)

target_link_libraries(detect_aruco
    tello_slam ${THIRD_PARTY_LIBS})
