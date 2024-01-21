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
    void set_current_roll_imu(const int32_t& current_roll_imu) {current_roll_imu_ = current_roll_imu;}

    // member methods /////////////////////////////////////////////////////////
    void run_as_thread();

    /**
     * 
     */
    void close();

private:
    // member data ////////////////////////////////////////////////////////////
    std::thread thread_;
    std::mutex mutex_;
    bool supervisor_running_ = true;

    int32_t current_roll_imu_;
    double current_roll_aruco_;

    // motion log =============================================================
    bool motion_log_on_;
    ArUco_Detector::Ptr aruco_detector_;
    IMU_Reader::Ptr imu_reader_;
    
    // member methods /////////////////////////////////////////////////////////
    /**
     * loop to run in thread 
     */
    void thread_task();
};

} // namespace tello_slam

#endif // TELLOSLAM_SUPERVISOR_H
