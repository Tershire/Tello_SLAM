// data_stream.h

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 MAR 11
// Wonhee LEE

// reference: slambook


#ifndef TELLOSLAM_DATASTREAM_H
#define TELLOSLAM_DATASTREAM_H

#include <deque>

#include "common.h"
#include "frame.h"


namespace tello_slam
{

/**
 * 
 */
class Data_Stream
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Data_Stream> Ptr;

    // state member ///////////////////////////////////////////////////////////
    enum Input_Mode
    {
        TELLO,
        USB,
        VIDEO,
    };
    
    // member data ////////////////////////////////////////////////////////////
    Input_Mode input_mode_;

    // constructor & destructor ///////////////////////////////////////////////
    Data_Stream();

    // member methods /////////////////////////////////////////////////////////
    /**
     * create and return the next frame containing the stereo images 
     */
    Frame::Ptr load_next_frame();

private:
    // member data ////////////////////////////////////////////////////////////
    cv::VideoCapture cap_;

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

#endif // TELLOSLAM_DATASTREAM_H
