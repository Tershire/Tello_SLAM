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
    cv::VideoCapture cap{"udp://0.0.0.0:11111", cv::CAP_FFMPEG};
    cv::Mat frame;

    Tello tello;
    if (!tello.connect()) 
    {
        return -1;
    }

    tello.enable_video_stream();
    std::thread vision_thread(vision_task, cap, frame);

    std::cout << "TEST" << std::endl;
    for (;;)
    {
        // std::cout << "/";
        if (cv::waitKey(1) == 27) {
            break;
        }
    }
    std::cout << std::endl;

    // tello.takeoff();

    // do something, fly around

    // tello.land();

    // vision_thread.join();

    return 0;
}

// ============================================================================
void vision_task(cv::VideoCapture cap, cv::Mat frame)
{
    for (;;)
    {
        cap >> frame;
        if (!frame.empty())
        {
            cv::imshow("Tello View", frame);
        }
        if (cv::waitKey(1) == 27) {
            break;
        }
    }
}

