add_executable(read_roll read_roll.cpp)
add_executable(detect_aruco_minimal detect_aruco_minimal.cpp)

target_link_libraries(read_roll
    tello_slam ${THIRD_PARTY_LIBS})
target_link_libraries(detect_aruco_minimal
    tello_slam ${THIRD_PARTY_LIBS})
    