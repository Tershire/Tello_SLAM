add_executable(read_roll read_roll.cpp)
add_executable(collect_aruco_data collect_aruco_data.cpp)

target_link_libraries(read_roll
    tello_slam ${THIRD_PARTY_LIBS})
target_link_libraries(collect_aruco_data
    tello_slam ${THIRD_PARTY_LIBS})
