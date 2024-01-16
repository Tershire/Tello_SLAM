// tello_sensor_test.cpp

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 JAN 15
// Wonhee LEE

// reference: 


#include "tello.hpp"
#include "common.h"

// using namespace tello_slam;


int main(int argc, char **argv)
{
    Tello tello;
    if (!tello.connect()) 
    {
        return -1;
    }

    int32_t time;
    for (;;)
    {
        // amount of time the motor has been used
        time = tello.state().time;

        // accelerometer measurements in {x, y, z} axes
        Vec3 a_IMU{tello.state().agx, tello.state().agy, tello.state().agz};

        // velocity measurements in {x, y, z} axes
        Vec3 v_IMU{tello.state().vgx, tello.state().vgy, tello.state().vgz};

        // {pitch, roll, yaw}
        Vec3 euler_angles_IMU{tello.state().pitch, tello.state().roll, tello.state().yaw};

        //
        std::cout << time << std::endl;
        std::cout << a_IMU.transpose() << std::endl;
        std::cout << v_IMU.transpose() << std::endl;
        std::cout << euler_angles_IMU.transpose() << std::endl;
        std::cout << std::endl;

        if (cv::waitKey(1) == 27) {
            break;
        }
    }

    return 0;
}
