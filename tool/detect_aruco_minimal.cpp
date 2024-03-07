// detect_aruco_minimal.cpp

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 MAR 07
// Wonhee LEE

// reference:


#include <iostream>

#include "port/config.h"
#include "system.h"
#include "marker/aruco_detector.h"

using namespace tello_slam;


int main(int argc, char **argv)
{
    std::string configuration_file_path = "./config/vision_system_config.yaml";
    
    System::Ptr vision_system = std::make_shared<System>(configuration_file_path);    
    assert(vision_system->initialize() == true);
    
    ArUco_Detector::Ptr aruco_detector = vision_system->get_aruco_detector();
    /*
    int target_id;
	std::cout << "Enter target_id: ";
	std::cin >> target_id;
    aruco_detector->set_target_id(target_id);
    */

    aruco_detector->run_minimal();

    return 0;
}
