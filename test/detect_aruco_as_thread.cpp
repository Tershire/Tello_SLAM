// detect_aruco_as_thread.cpp

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 JAN 20
// Wonhee LEE

// reference:


#include <iostream>

#include "port/config.h"
#include "system.h"
#include "marker/aruco_detector.h"
#include "tello.hpp"

using namespace tello_slam;


int main(int argc, char **argv)
{
    std::string configuration_file_path = "./config/vision_system_config.yaml";
    
    System::Ptr vision_system = std::make_shared<System>(configuration_file_path);    
    assert(vision_system->initialize() == true);

    // connect to Tello
    Tello tello;
    if (!tello.connect()) 
    {
        return -1;
    }

    tello.enable_video_stream();
    
    ArUco_Detector::Ptr aruco_detector = vision_system->get_aruco_detector();
    int target_id;
	std::cout << "Enter target_id: ";
	std::cin >> target_id;
    aruco_detector->set_target_id(target_id);

    aruco_detector->run_as_thread();

    // main thread task
    while (true)
    {
        std::cout << "\a";
        std::cout << tello.state().roll << std::endl;
    }
    
    aruco_detector->close();

    return 0;
}
