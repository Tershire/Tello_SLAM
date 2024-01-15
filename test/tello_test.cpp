// tello_test.cpp

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 JAN 15
// Wonhee LEE

// reference:


#include <iostream>

#include "tello.hpp"

// using namespace tello_slam;


int main(int argc, char **argv)
{
    Tello tello;
    if (!tello.connect()) 
    {
        return -1;
    }

    tello.takeoff();

    // do something, fly around

    tello.land();

    return 0;
}
