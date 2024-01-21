// read_roll.cpp

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 JAN 15
// Wonhee LEE

// reference:


#include <iostream>

#include "port/config.h"
#include "tool_system.h"
#include "marker/aruco_detector.h"
#include "tello.hpp"

using namespace tello_slam;


int main(int argc, char **argv)
{
    // configure system =======================================================
    std::string configuration_file_path = "./config/vision_system_config.yaml";
    
    Tool_System::Ptr tool_system = std::make_shared<Tool_System>(configuration_file_path);    
    assert(tool_system->initialize() == true);

    // connect to Tello =======================================================
    Tello tello;
    if (!tello.connect()) 
    {
        return -1;
    }

    tello.enable_video_stream();
    
    // configure system components ============================================
    ArUco_Detector::Ptr aruco_detector = tool_system->get_aruco_detector();
    int target_id;
	std::cout << "Enter target_id: ";
	std::cin >> target_id;
    aruco_detector->set_target_id(target_id);

    // initiate threads =======================================================
    aruco_detector->run_as_thread();

    // main thread task =======================================================
    while (true)
    {
        std::cout << "\a";
        std::cout << tello.state().roll << std::endl;
    }
    
    // join threads ===========================================================
    aruco_detector->close();

    return 0;
}