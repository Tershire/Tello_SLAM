add_executable(run_tello_slam run_tello_slam.cpp)

target_link_libraries(run_tello_slam 
    tello_slam ${THIRD_PARTY_LIBS})
