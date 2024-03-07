// tello_vision_test_minimal.cpp

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 MAR 07
// Wonhee LEE

// reference:


#include <iostream>
#include <opencv2/opencv.hpp>

// #include "tello.hpp"

// using namespace tello_slam;


int main(int argc, char **argv)
{
    cv::VideoCapture cap{"udp://0.0.0.0:11111", cv::CAP_FFMPEG};
    cv::Mat frame;

    // Tello tello;
    // if (!tello.connect()) 
    // {
    //     return -1;
    // }

    // tello.enable_video_stream();
    //
    // tello.takeoff();

    for (;;)
    {
        cap >> frame;
        if (!frame.empty())
        {
            cv::imshow("Tello View", frame);
        }
        if (cv::waitKey(1) == 27)
        {
            break;
        }
    }

    //
    // tello.land();

    return 0;
}
