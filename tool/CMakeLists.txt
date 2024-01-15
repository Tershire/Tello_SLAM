add_executable(aruco_roll_reader aruco_roll_reader.cpp)

target_link_libraries(aruco_roll_reader
    tello_slam ${THIRD_PARTY_LIBS})
    