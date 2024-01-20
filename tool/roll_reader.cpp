// aruco_roll_reader.cpp

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 JAN 15
// Wonhee LEE

// reference:


#include <iostream>

#include "port/config.h"
#include "tool/tool_system.h"
#include "marker/aruco_detector.h"
#include "tello.hpp"

using namespace tello_slam;


bool read_imu();


int main(int argc, char **argv)
{
    // Tello connection =======================================================
    // connect to Tello
    Tello tello;
    if (!tello.connect()) 
    {
        return -1;
    }

    tello.enable_video_stream();

    // configure the system ===================================================
    std::string configuration_file_path = "./config/vision_system_config.yaml";
    
    Tool_System::Ptr tool_system = std::make_shared<Tool_System>(configuration_file_path);    
    assert(tool_system->initialize() == true);

    // get system components --------------------------------------------------
    ArUco_Detector::Ptr aruco_detector = tool_system->get_aruco_detector();
    int target_id;
	std::cout << "Enter target_id: ";
	std::cin >> target_id;
    aruco_detector->set_target_id(target_id);

    IMU_Reader::Ptr imu_reader = tool_system->get_imu_reader();
    imu_reader->set_tello(&tello);

    // initiate threads =======================================================
    imu_reader->run_as_thread();
    aruco_detector->run();
    // aruco_detector->run_as_thread(); // double free or corruption (out); Aborted (core dumped)

    // main thread task =======================================================
    // for (;;)
    // {
    //     // std::cout << "/";
    //     if (cv::waitKey(1) == 27) {
    //         break;
    //     }
    // }

    // join threads ===========================================================
    imu_reader->close();
    aruco_detector->close();
    
    return 0;
}

// helper /////////////////////////////////////////////////////////////////////
bool read_imu(Tello& tello)
{
    int32_t current_roll;
    for (;;)
    {
        // accelerometer measurements in {x, y, z} axes
        // Vec3 a_IMU{tello.state().agx, tello.state().agy, tello.state().agz};

        // velocity measurements in {x, y, z} axes
        // Vec3 v_IMU{tello.state().vgx, tello.state().vgy, tello.state().vgz};

        // {pitch, roll, yaw}
        // Vec3 euler_angles_IMU{tello_.state().pitch, tello_.state().roll, tello_.state().yaw};s
        current_roll = tello.state().roll;
        std::cout << current_roll << std::endl;
    }
}
