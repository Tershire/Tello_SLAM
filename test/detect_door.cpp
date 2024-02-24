// detect_door.cpp

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 FEB 24
// Wonhee LEE

// reference:

#include <iostream>

#include "port/config.h"
#include "system.h"
#include "door/door_detector.h"

using namespace tello_slam;


int main(int argc, char **argv)
{
    std::string configuration_file_path = "./config/vision_system_config.yaml";
    
    System::Ptr vision_system = std::make_shared<System>(configuration_file_path);    
    assert(vision_system->initialize() == true);
    
    Door_Detector::Ptr door_detector = vision_system->get_door_detector();

    door_detector->run();

    return 0;
}
