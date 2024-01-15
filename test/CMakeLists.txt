add_executable(detect_aruco detect_aruco.cpp)
add_executable(tello_test tello_test.cpp)
add_executable(tello_flight_test tello_flight_test.cpp)
add_executable(tello_vision_test tello_vision_test.cpp)

target_link_libraries(detect_aruco
    tello_slam ${THIRD_PARTY_LIBS})
target_link_libraries(tello_test
    tello_slam ${THIRD_PARTY_LIBS})
target_link_libraries(tello_flight_test
    tello_slam ${THIRD_PARTY_LIBS})
target_link_libraries(tello_vision_test
    tello_slam ${THIRD_PARTY_LIBS})
