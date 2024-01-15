// tello_test.cpp

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 JAN 15
// Wonhee LEE

// reference: https://github.com/HerrNamenlos123/tello/tree/master


#include <iostream>
#include <opencv2/opencv.hpp>

#include "tello.hpp"

// using namespace tello_slam;

void vision_task(cv::VideoCapture cap, cv::Mat frame);


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
