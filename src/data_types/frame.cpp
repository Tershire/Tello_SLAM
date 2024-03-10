// frame.cpp

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 MAR 10
// Wonhee LEE

// reference: slambook


#include "frame.h"


namespace tello_slam
{

// public XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// constructor & destructor ///////////////////////////////////////////////////
Frame::Frame(long id, double timestamp, const SE3& T_cw, const cv::Mat& image):
    id_(id), timestamp_(timestamp), T_cw_(T_cw), image_(image) {}

// member methods /////////////////////////////////////////////////////////////
void Frame::set_as_keyframe()
{
    static long factory_id = 0;
    
    is_keyframe_ = true;
    keyframe_id_ = factory_id += 1;
}

// static methods /////////////////////////////////////////////////////////////
Frame::Ptr Frame::create_frame()
{
    static long factory_id = 0;

    Frame::Ptr frame(new Frame);
    frame->id_ = factory_id += 1;

    return frame;
}

} // namespace tello_slam
