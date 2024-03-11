// run_tello_slam.cpp

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 MAR 11
// Wonhee LEE

// reference: slambook


// #include <easy/profiler.h>

#include "system.h"

using namespace tello_slam;


int main(int argc, char **argv)
{
    std::string configuration_file_path = "./config/system_config.yaml";

    System::Ptr system(new System(configuration_file_path));
    assert(system->initialize() == true);
    
    std::cout << "\nTello SLAM: start!" << std::endl;
    system->run();

    return 0;
}
