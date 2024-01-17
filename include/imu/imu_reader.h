// imu_reader.h

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 JAN 17
// Wonhee LEE

// reference:


#ifndef TELLOSLAM_IMU_IMUREADER_H
#define TELLOSLAM_IMU_IMUREADER_H

#include "common.h"
#include "tello.hpp"


namespace tello_slam
{

class IMU_Reader
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<IMU_Reader> Ptr;

    // constructor & destructor ///////////////////////////////////////////////
    IMU_Reader();

    // getter & setter ////////////////////////////////////////////////////////
    // getter =================================================================
    int32_t get_current_roll() const {return current_roll_;}

    // member methods /////////////////////////////////////////////////////////
    bool run();

private:
    // member data ////////////////////////////////////////////////////////////
    Tello tello_;

    int32_t current_roll_;
};

} // namespace tello_slam

#endif // TELLOSLAM_IMU_IMUREADER_H
