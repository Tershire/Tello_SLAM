add_executable(roll_reader roll_reader.cpp)

target_link_libraries(roll_reader
    tello_slam ${THIRD_PARTY_LIBS})
    