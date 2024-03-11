// dataset.h

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 MAR 11
// Wonhee LEE

// reference: slambook


#ifndef TELLOSLAM_DATASET_H
#define TELLOSLAM_DATASET_H

#include <deque>

#include "common.h"
#include "camera/camera.h"
#include "frame.h"


namespace tello_slam
{

/**
 * Data set read
 * The configuration file path is passed in during construction, and the dataset_dir of the configuration file is the dataset path
 * After Init, the camera and the next frame image can be obtained
 */
class Dataset
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Dataset> Ptr;

    // constructor & destructor ///////////////////////////////////////////////
    Dataset(const std::string& dataset_directory_path);

    // member methods /////////////////////////////////////////////////////////
    /**
     * create and return the next frame containing the stereo images 
     */
    Frame::Ptr load_next_frame();

    /**
     * load timestamp data
     */
    void load_timestamps(const std::string& times_file_path);

    /**
     * 
     */
    double load_next_timestamp();

private:
    // member data ////////////////////////////////////////////////////////////
    std::string dataset_directory_path_;
    std::vector<std::string> file_paths_;
    int num_images_;
    int current_image_index_ = 0;

    float pre_resize_factor_;

    // configuration ==========================================================
    bool clahe_on_;

    // pre-processing =========================================================
    cv::Ptr<cv::CLAHE> clahe_;

    // motion log: timestamp ==================================================
    bool motion_log_on_;
    std::deque<double> timestamps_;
};

} // namespace tello_slam

#endif // TELLOSLAM_DATASET_H
