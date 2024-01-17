// supervisor.h

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 JAN 17
// Wonhee LEE

// reference:


#ifndef TELLOSLAM_SUPERVISOR_H
#define TELLOSLAM_SUPERVISOR_H

#include <pangolin/pangolin.h>

#include "common.h"
#include "marker/aruco_detector.h"
#include "imu/imu_reader.h"


namespace tello_slam
{

/**
 * data writier & viewer for the sensor fusion
 */
class Supervisor
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Supervisor> Ptr;

    // member data ////////////////////////////////////////////////////////////
    // step-by-step functionality =============================================
    bool do_pause_ = true;
    bool step_to_next_frame_ = false;

    // constructor & destructor ///////////////////////////////////////////////
    Supervisor();

    // getter & setter ////////////////////////////////////////////////////////
    // setter =================================================================
    void set_aruco_detector(ArUco_Detector::Ptr aruco_detector) {aruco_detector_ = aruco_detector;}
    void set_imu_reader(IMU_Reader::Ptr imu_reader) {imu_reader_ = imu_reader;}

    // member methods /////////////////////////////////////////////////////////
    /**
     * 
     */
    void close();

private:
    // member data ////////////////////////////////////////////////////////////
    std::thread thread_loop_;
    bool viewer_running_ = true;

    std::mutex data_mutex_;

    // motion log =============================================================
    bool motion_log_on_;
    ArUco_Detector::Ptr aruco_detector_;
    IMU_Reader::Ptr imu_reader_;
    
    // member methods /////////////////////////////////////////////////////////
    /**
     * loop to run in thread 
     */
    void thread_loop();
};

} // namespace tello_slam

#endif // TELLOSLAM_SUPERVISOR_H
