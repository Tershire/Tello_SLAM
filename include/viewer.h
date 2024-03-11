// viewer.h

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 MAR 10
// Wonhee LEE

// reference: slambook


#ifndef TELLOSLAM_VIEWER_H
#define TELLOSLAM_VIEWER_H

#include <pangolin/pangolin.h>

#include "common.h"
#include "frame.h"
#include "map.h"
#include "frontend.h"


namespace tello_slam
{

/**
 * visualization
 */
class Viewer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Viewer> Ptr;

    // member data ////////////////////////////////////////////////////////////
    // step-by-step functionality =============================================
    bool do_pause_ = true;
    bool step_to_next_frame_ = false;

    // constructor & destructor ///////////////////////////////////////////////
    Viewer();

    // getter & setter ////////////////////////////////////////////////////////
    // setter =================================================================
    void set_map(Map::Ptr map) {map_ = map;}
    void set_frontend(Frontend::Ptr frontend) {frontend_ = frontend;}

    void set_current_frame(Frame::Ptr current_frame);

    // member methods /////////////////////////////////////////////////////////
    /**
     * 
     */
    void close();
   
    /**
     * update map 
     */
    void update_map();
    void request_update_map();

private:
    // member data ////////////////////////////////////////////////////////////
    Frame::Ptr current_frame_ = nullptr;
    Map::Ptr map_ = nullptr;

    std::thread thread_loop_;
    bool viewer_running_ = true;

    std::unordered_map<unsigned long, Frame::Ptr> active_keyframes_;
    std::unordered_map<unsigned long, Landmark::Ptr> active_landmarks_;
    std::unordered_map<unsigned long, ArUco_Landmark::Ptr> active_aruco_landmarks_;
    bool map_updated_ = false;

    std::mutex data_mutex_;

    bool flag_request_update_map = false;

    // trail ==================================================================
    bool trail_on_;
    std::vector<Vec3> trail_points_;

    // motion log =============================================================
    bool motion_log_on_;
    Frontend::Ptr frontend_;
    
    // member methods /////////////////////////////////////////////////////////
    /**
     * loop to run in thread 
     */
    void thread_loop();

    /**
     * draw active map points and keyframes
     */
    void draw_active_landmarks();

    /**
     * 
     */
    void draw_active_keyframes();

    /**
     * draw frame
     */
    void draw_frame(Frame::Ptr frame, const float* COLOR);

    /**
     * follow current frame 
     */
    void follow_current_frame(pangolin::OpenGlRenderState& s_cam);

    /**
     * 
     */
    void draw_trail(Frame::Ptr frame, const float* COLOR);

    /**
     * draw features in image
     */
    cv::Mat draw_aruco_features_in_image();
};

} // namespace tello_slam

#endif // TELLOSLAM_VIEWER_H
