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
IMU_Reader::IMU_Reader() {}

// member methods /////////////////////////////////////////////////////////////
bool IMU_Reader::run()
{
    for (;;)
    {
        // accelerometer measurements in {x, y, z} axes
        // Vec3 a_IMU{tello.state().agx, tello.state().agy, tello.state().agz};

        // velocity measurements in {x, y, z} axes
        // Vec3 v_IMU{tello.state().vgx, tello.state().vgy, tello.state().vgz};

        // {pitch, roll, yaw}
        // Vec3 euler_angles_IMU{tello_.state().pitch, tello_.state().roll, tello_.state().yaw};s
        current_roll_ = tello_.state().roll;
    }
}

} // namespace tello_slam
