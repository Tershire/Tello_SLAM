// supervisor.cpp

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 JAN 17
// Wonhee LEE

// reference:


#include <pangolin/pangolin.h>
#include <opencv2/opencv.hpp>

#include "supervisor.h"
#include "port/config.h"


namespace tello_slam
{

// public XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// constructor & destructor ///////////////////////////////////////////////////
Supervisor::Supervisor()
{
    motion_log_on_ = Config::read<int>("motion_log_on");
}

// getters & setters //////////////////////////////////////////////////////////

// member methods /////////////////////////////////////////////////////////////
void Supervisor::run_as_thread()
{
    thread_ = std::thread(&Supervisor::thread_task, this);
}

void Supervisor::close()
{
    supervisor_running_ = false;
    thread_.join();
}

// private XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// member methods /////////////////////////////////////////////////////////////
void Supervisor::thread_task()
{
    pangolin::CreateWindowAndBind("VO: Map", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Supervisor setting =====================================================
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

    // Data Logger setting ====================================================
    // data logger object
    pangolin::DataLog log;

    // add named labels
    std::vector<std::string> labels;
    labels.push_back(std::string("roll [deg]"));
    log.SetLabels(labels);

    // create interactive view in window
    pangolin::Plotter plotter(&log, 0.0, 300, -1, 1, 30, 0.5);
    plotter.Track("$i"); // (TO DO) does it follow correctly in accordance with the timestamp (?)

    // Add some sample annotations to the plot (TO DO) what is this (?)
    plotter.AddMarker(pangolin::Marker::Vertical  , -1000, pangolin::Marker::LessThan   , pangolin::Colour::Blue() .WithAlpha(0.2f));
    plotter.AddMarker(pangolin::Marker::Horizontal,   100, pangolin::Marker::GreaterThan, pangolin::Colour::Red()  .WithAlpha(0.2f));
    plotter.AddMarker(pangolin::Marker::Horizontal,    10, pangolin::Marker::Equal      , pangolin::Colour::Green().WithAlpha(0.2f));

    // create display =========================================================
    pangolin::Display("Monitor")
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
    pangolin::Var<bool>      toggle_pause("menu.Pause"     , true , true );
    pangolin::Var<bool> button_next_frame("menu.Next Frame", false, false);
    // ------------------------------------------------------------------------

    // color palette
    const float WHITE[3] = {1.0F, 1.0F, 1.0F};
    const float RED[3] = {1.0F, 0.0F, 0.0F};

    while (!pangolin::ShouldQuit() && supervisor_running_)
    {
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
             log.Log(current_roll_imu_);
        }

        pangolin::FinishFrame();
    }

    std::cout << "viewer: thread loop ended" << std::endl;
}

} // namespace tello_slam
