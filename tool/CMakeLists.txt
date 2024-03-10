add_executable(read_roll read_roll.cpp)

target_link_libraries(read_roll
    tello_slam ${THIRD_PARTY_LIBS})
