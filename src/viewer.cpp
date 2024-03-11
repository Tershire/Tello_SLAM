// viewer.h

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 MAR 10
// Wonhee LEE

// reference: slambook


#include <pangolin/pangolin.h>
#include <opencv2/opencv.hpp>

#include "viewer.h"
#include "frame.h"
#include "feature.h"
#include "aruco_feature.h"
#include "config.h"


namespace tello_slam
{

// public XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// constructor & destructor ///////////////////////////////////////////////////
Viewer::Viewer()
{
    thread_loop_ = std::thread(&Viewer::thread_loop, this);

    trail_on_ = Config::read<int>("trail_on");
    motion_log_on_ = Config::read<int>("motion_log_on");
}

// getters & setters //////////////////////////////////////////////////////////
void Viewer::set_current_frame(Frame::Ptr current_frame)
{
    //std::unique_lock<std::mutex> lock(data_mutex_); //this mutex is not needed
    current_frame_ = current_frame;
}

// member methods /////////////////////////////////////////////////////////////
void Viewer::close()
{
    viewer_running_ = false;
    thread_loop_.join();
}

void Viewer::update_map()
{
    //std::unique_lock<std::mutex> lock(data_mutex_);
    assert(map_ != nullptr);
    active_landmarks_ = map_->get_active_landmarks();
    active_keyframes_  = map_->get_active_keyframes();
    map_updated_ = true;
}

void Viewer::request_update_map()
{
    flag_request_update_map = true;
}

// private XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// member methods /////////////////////////////////////////////////////////////
void Viewer::thread_loop()
{
    pangolin::CreateWindowAndBind("Tello SLAM: Map", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Map Viewer setting =====================================================
    // define projection and initial ModelView matrix
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, 400, 400, 512, 384, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -5, -10, 0, 0, 0, 0.0, -1.0, 0.0));

    // create interactive view in window
    pangolin::Handler3D handler(s_cam);
    pangolin::View& vis_display = pangolin::CreateDisplay()
        // .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
        .SetBounds(0.0, 1.0, 0.0, 1.0)
        .SetAspect(1024.0 / 480.0)
        .SetHandler(&handler);

    // data logger setting ===================================================
    // data logger object
    pangolin::DataLog log;

    // add named labels 
    std::vector<std::string> labels;
    labels.push_back(std::string("v_x (ref.: left cam.)"));
    labels.push_back(std::string("v_y (ref.: left cam.)"));
    labels.push_back(std::string("v_z (ref.: left cam.)"));
    log.SetLabels(labels);

    // create interactive view in window
    pangolin::Plotter plotter(&log, 0.0, 300, -1, 1, 30, 0.5);
    plotter.Track("$i"); // (TO DO) does it follow correctly in accordance with the timestamp (?)

    // Add some sample annotations to the plot
    plotter.AddMarker(pangolin::Marker::Vertical, -1000, pangolin::Marker::LessThan, pangolin::Colour::Blue().WithAlpha(0.2f));
    plotter.AddMarker(pangolin::Marker::Horizontal, 100, pangolin::Marker::GreaterThan, pangolin::Colour::Red().WithAlpha(0.2f));
    plotter.AddMarker(pangolin::Marker::Horizontal, 10, pangolin::Marker::Equal, pangolin::Colour::Green().WithAlpha(0.2f));

    // create display =========================================================
    pangolin::Display("Multi Monitor")
        .SetBounds(0.0, 1.0, 0.0, 1.0)
        .SetLayout(pangolin::LayoutVertical)
        .AddDisplay(vis_display)
        .AddDisplay(plotter);

    // ------------------------------------------------------------------------
    // side menu setting ======================================================
    // (TO DO) GL segmentation fault at the end of the program run
    pangolin::CreatePanel("menu")
        .SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(175));

    // toggle
    pangolin::Var<bool> toggle_pause("menu.Pause", true ,true);
    pangolin::Var<bool> button_next_frame("menu.Next Frame", false, false);
    // ------------------------------------------------------------------------

    // color palette
    const float WHITE[3] = {1.0F, 1.0F, 1.0F};
    const float RED[3] = {1.0F, 0.0F, 0.0F};

    while (!pangolin::ShouldQuit() && viewer_running_)
    {
        //update the map if it has been requested by the frontend
        if(flag_request_update_map == true)
        {
            update_map();
            flag_request_update_map = false;
        }

        // clear screen and activate view to render into
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        
        // set background color
        glClearColor(0.15F, 0.15F, 0.15F, 0.0F);

        // side menu operation ------------------------------------------------
        if (toggle_pause)
        {
            do_pause_ = true;
        }
        else
        {
            do_pause_ = false;
        }

        if (pangolin::Pushed(button_next_frame))
        {
            step_to_next_frame_ = true;
        }
        else
        {
            step_to_next_frame_ = false;
        }
        // --------------------------------------------------------------------

        vis_display.Activate(s_cam);

        // log ================================================================
        if (motion_log_on_ && !do_pause_)
        {
            Vec3 velocity = frontend_->get_current_camera_translational_velocity();
            log.Log(velocity[0], velocity[1], velocity[2]);
        }

        // draw map ===========================================================
        std::unique_lock<std::mutex> lock(data_mutex_);
        if (map_)
        {
            draw_active_landmarks();
            draw_active_keyframes();
        }

        if (current_frame_)
        {
            if (trail_on_) draw_trail(current_frame_, WHITE);
            draw_frame(current_frame_, RED);
            follow_current_frame(s_cam);

            cv::Mat image = draw_aruco_features_in_image();
            cv::imshow("Tello SLAM: Image", image);
            cv::waitKey(1);
        }

        pangolin::FinishFrame();

        // usleep(5000);
    }
    std::cout << "viewer: thread loop ended" << std::endl;
}

// ----------------------------------------------------------------------------
void Viewer::draw_frame(Frame::Ptr frame, const float* COLOR)
{
    Sophus::SE3d T_wc = frame->get_T_cw().inverse();
    
    const float fx = 400; // should I change this (?)
    const float fy = 400; // should I change this (?)
    const float cx = 512; // should I change this (?)
    const float cy = 384; // should I change this (?)

    const float SCALE = 0.25F; // scale frame size
    const int   LINE_WIDTH = 2;
    const float WIDTH  = 1080;
    const float HEIGHT = 768;

    glPushMatrix();

    Sophus::Matrix4f m = T_wc.matrix().template cast<float>();
    glMultMatrixf((GLfloat*) m.data());

    if (COLOR == nullptr)
    {
        glColor3f(0, 0, 1);
    }
    else
        glColor3f(COLOR[0], COLOR[1], COLOR[2]);

    glLineWidth(LINE_WIDTH);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(SCALE * (0 - cx) / fx, SCALE * (0 - cy) / fy, SCALE);
    glVertex3f(0, 0, 0);
    glVertex3f(SCALE * (0 - cx) / fx, SCALE * (HEIGHT - 1 - cy) / fy, SCALE);
    glVertex3f(0, 0, 0);
    glVertex3f(SCALE * (WIDTH - 1 - cx) / fx, SCALE * (HEIGHT - 1 - cy) / fy, SCALE);
    glVertex3f(0, 0, 0);
    glVertex3f(SCALE * (WIDTH - 1 - cx) / fx, SCALE * (0 - cy) / fy, SCALE);

    glVertex3f(SCALE * (WIDTH - 1 - cx) / fx, SCALE * (0 - cy) / fy, SCALE);
    glVertex3f(SCALE * (WIDTH - 1 - cx) / fx, SCALE * (HEIGHT - 1 - cy) / fy, SCALE);

    glVertex3f(SCALE * (WIDTH - 1 - cx) / fx, SCALE * (HEIGHT - 1 - cy) / fy, SCALE);
    glVertex3f(SCALE * (0 - cx) / fx, SCALE * (HEIGHT - 1 - cy) / fy, SCALE);

    glVertex3f(SCALE * (0 - cx) / fx, SCALE * (HEIGHT - 1 - cy) / fy, SCALE);
    glVertex3f(SCALE * (0 - cx) / fx, SCALE * (0 - cy) / fy, SCALE);

    glVertex3f(SCALE * (0 - cx) / fx, SCALE * (0 - cy) / fy, SCALE);
    glVertex3f(SCALE * (WIDTH - 1 - cx) / fx, SCALE * (0 - cy) / fy, SCALE);

    glEnd();
    glPopMatrix();
}

// ----------------------------------------------------------------------------
void Viewer::draw_active_keyframes()
{
    const float COLOR[3] = {0.0F, 0.0F, 1.0F};

    for (auto& active_keyframe : active_keyframes_)
    {
        draw_frame(active_keyframe.second, COLOR);
    }
}

// ----------------------------------------------------------------------------
void Viewer::draw_active_landmarks()
{
    const float COLOR[3] = {0.15F, 1.0F, 0.0F};

    glPointSize(2);
    glBegin(GL_POINTS);
    for (auto& active_landmark : active_landmarks_)
    {
        auto position = active_landmark.second->get_position();
        glColor3f(COLOR[0], COLOR[1], COLOR[2]);
        glVertex3d(position[0], position[1], position[2]);
    }
    glEnd();
}

// ----------------------------------------------------------------------------
void Viewer::draw_trail(Frame::Ptr frame, const float* COLOR)
{
    Sophus::SE3d T_wc = frame->get_T_cw().inverse();
    Vec3 position = T_wc * Vec3(0, 0, 0);
    trail_points_.push_back(position);

    glPointSize(1);

    glBegin(GL_POINTS);
    for (auto& position : trail_points_)
    {
        glColor3f(COLOR[0], COLOR[1], COLOR[2]);
        glVertex3d(position[0], position[1], position[2]);
    }
    glEnd();
}

// ----------------------------------------------------------------------------
void Viewer::follow_current_frame(pangolin::OpenGlRenderState& s_cam)
{
    Sophus::SE3d T_wc = current_frame_->get_T_cw().inverse();
    pangolin::OpenGlMatrix m(T_wc.matrix());
    s_cam.Follow(m, true);
}

// ----------------------------------------------------------------------------
cv::Mat Viewer::draw_aruco_features_in_image() // (TO DO) fix segmentation fault
{
    //
    // std::cout << "[0]" << std::endl;

    cv::Mat image_out;
    cv::cvtColor(current_frame_->image_, image_out, cv::COLOR_GRAY2BGR);

    //
    // std::cout << "[1]" << std::endl;

    for (size_t i = 0; i < current_frame_->aruco_features_.size(); ++i)
    {
        //
        // std::cout << "[2]" << std::endl;

        if (current_frame_->aruco_features_[i]) // -> segfault fix try #1
        {
            //
            // std::cout << "[3]" << std::endl;

            // (guess) [segfault] it seems like feature_L_[i] could be erased by the other thread
            if (current_frame_->aruco_features_[i]->aruco_landmark_.lock())
            {
                //
                // std::cout << "[4]" << std::endl;

                auto aruco_feature = current_frame_->aruco_features_[i];
                if (aruco_feature->is_outlier_)
                {
                    // std::cout << "[5]" << std::endl;
                    cv::circle(image_out, aruco_feature->keypoint_.pt, 3, cv::Scalar(0, 0, 255), 2);
                }
                else
                {
                    cv::circle(image_out, aruco_feature->keypoint_.pt, 3, cv::Scalar(0, 250, 0), 2);
                }
                //
                // std::cout << "[6]" << std::endl;
            }
        }
    }
    return image_out;
}

} // namespace tello_slam