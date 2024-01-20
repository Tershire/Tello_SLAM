// imu_reader.cpp

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 JAN 17
// Wonhee LEE

// reference:


#include "imu/imu_reader.h"


namespace tello_slam
{

// public XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// constructor & destructor ///////////////////////////////////////////////////
IMU_Reader::IMU_Reader()
{
    thread_ = std::thread(&IMU_Reader::run, this);
}

// member methods /////////////////////////////////////////////////////////////
bool IMU_Reader::run()
{
    std::cout << "[IMU Reader] started running." << std::endl;
    for (;;)
    {
        std::cout << "tello_ != nullptr: " << (tello_ != nullptr) << std::endl;
        // accelerometer measurements in {x, y, z} axes
        // Vec3 a_IMU{tello.state().agx, tello.state().agy, tello.state().agz};

        // velocity measurements in {x, y, z} axes
        // Vec3 v_IMU{tello.state().vgx, tello.state().vgy, tello.state().vgz};

        // {pitch, roll, yaw}
        // Vec3 euler_angles_IMU{tello_.state().pitch, tello_.state().roll, tello_.state().yaw};
        current_roll_ = tello_->state().roll;

        std::cout << "[IMU Reader] " << current_roll_ << std::endl;
    }
}

// ----------------------------------------------------------------------------
bool IMU_Reader::run_as_thread()
{
    std::cout << "[IMU Reader] started running as thread." << std::endl;
    thread_ = std::thread(&IMU_Reader::run, this);
}

// ----------------------------------------------------------------------------
void IMU_Reader::close()
{
    thread_.join();
}

} // namespace tello_slam
