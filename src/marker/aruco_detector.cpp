// aruco_detector.cpp

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 JAN 15
// Wonhee LEE

// reference:
// https://stackoverflow.com/questions/997946/how-to-get-current-time-and-date-in-c


#include <unsupported/Eigen/EulerAngles>
#include <chrono> // for experiment

#include "marker/aruco_detector.h"
#include "port/config.h"
#include "toolbox/conversion_toolbox.h"


namespace tello_slam
{

// public XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// constructor & destructor ///////////////////////////////////////////////////
ArUco_Detector::ArUco_Detector() {}

ArUco_Detector::ArUco_Detector(const int& target_id,
    const std::string& predifined_dictionary_name,
    const double& marker_length, const Camera::Ptr camera)
    : target_id_(target_id), marker_length_(marker_length), camera_(camera)
{
    // camera =================================================================
    cameraMatrix_ = camera->cameraMatrix_;
    distCoeffs_   = camera->distCoeffs_;

    // ArUco ==================================================================
    cv::aruco::PredefinedDictionaryType predifined_dictionary = cv::aruco::DICT_5X5_50;

    if (predifined_dictionary_name == "DICT_5X5_50")
    {
        predifined_dictionary = cv::aruco::DICT_5X5_50;
    }
    else if (predifined_dictionary_name == "DICT_4X4_50")
    {
        predifined_dictionary = cv::aruco::DICT_4X4_50;
    }
    else
    {
        std::cout << "ERROR: given ArUco predefined dictionary is not registered" << std::endl;
    }

    dictionary_ = cv::aruco::getPredefinedDictionary(predifined_dictionary);
    detector_parameters_ = cv::aruco::DetectorParameters();

    detector_ = std::make_shared<cv::aruco::ArucoDetector>(dictionary_, detector_parameters_);
    
    // PnP --------------------------------------------------------------------
    cv::Point3d p3D0_target(-marker_length / 2,  marker_length / 2, 0);
    cv::Point3d p3D1_target( marker_length / 2,  marker_length / 2, 0);
    cv::Point3d p3D2_target( marker_length / 2, -marker_length / 2, 0);
    cv::Point3d p3D3_target(-marker_length / 2, -marker_length / 2, 0);
    p3Ds_marker_ = {p3D0_target, p3D1_target, p3D2_target, p3D3_target};

    // port ===================================================================
    std::string input_mode = Config::read<std::string>("input_mode");
    if (input_mode == "usb")
        input_mode_ = Input_Mode::USB;
    else if (input_mode == "video")
        input_mode_ = Input_Mode::VIDEO;
    else if (input_mode == "tello")
        input_mode_ = Input_Mode::TELLO;
    else if (input_mode == "raspberry")
        input_mode_ = Input_Mode::RASPBERRY;
    else if (input_mode == "realsense")
        input_mode_ = Input_Mode::REALSENSE;
    else
        std::cout << "ERROR: input mode wrong\n";

    pre_resize_factor_ = Config::read<float>("pre_resize_factor");
    resize_scale_factor_ = Config::read<float>("resize_scale_factor");
}

// member methods /////////////////////////////////////////////////////////////
// ----------------------------------------------------------------------------
bool ArUco_Detector::detect(cv::Mat& image, std::vector<int>& ids, 
    std::vector<std::vector<cv::Point2f>>& p2Dss_pixel)
{
    std::vector<std::vector<cv::Point2f>> rejected_p2Dss_pixel;
    detector_->detectMarkers(image, p2Dss_pixel, ids, rejected_p2Dss_pixel);

    return true;
}

// ----------------------------------------------------------------------------
bool ArUco_Detector::estimate_poses(std::vector<int>& ids, 
    std::vector<std::vector<cv::Point2f>>& p2Dss_pixel,
    std::vector<SE3>& Ts_cm)
{
    std::vector<cv::Point2f> p2Ds_pixel;

    cv::Vec3d rvec, tvec;
    cv::Matx33d rmat;

    Vec3 r_cm, t_cm;
    Mat33 R_cm;
    SE3 T_cm;

    int num_iterations = 100;
    float reprojection_error = 8.0;
    double confidence = 0.99;
    cv::Mat inlier_corner_point_indices;

    std::vector<int> inlier_ids;
        std::vector<std::vector<cv::Point2f>> inlier_p2Dss_pixel;

    for (size_t i = 0; i < ids.size(); ++i)
    {
        p2Ds_pixel = p2Dss_pixel[i];

        // estimate pose with RANSAC
        cv::solvePnPRansac(p3Ds_marker_, p2Ds_pixel, 
            cameraMatrix_, distCoeffs_, rvec, tvec, false, num_iterations, 
            reprojection_error, confidence, inlier_corner_point_indices, cv::SOLVEPNP_IPPE_SQUARE);
        
        if (inlier_corner_point_indices.total() == 4)
        {
            // refine pose iteratively using Levenberg-Marquardt method
            cv::solvePnPRefineLM(p3Ds_marker_, p2Ds_pixel, 
                cameraMatrix_, distCoeffs_, rvec, tvec);

            // convert rotation vector to rotation matrix
            cv::Rodrigues(rvec, rmat);

            // convert to Eigen
            R_cm << rmat(0, 0), rmat(0, 1), rmat(0, 2),
                    rmat(1, 0), rmat(1, 1), rmat(1, 2),
                    rmat(2, 0), rmat(2, 1), rmat(2, 2);

            t_cm = Vec3(tvec[0], tvec[1], tvec[2]);

            // convert to Sophus
            Quaternion q_cm(R_cm);
            T_cm = SE3(q_cm, t_cm); // SE3(SO3(R_cm), t_cm)

            //
            Ts_cm.push_back(T_cm);
            inlier_ids.push_back(ids[i]);
            inlier_p2Dss_pixel.push_back(p2Dss_pixel[i]);
        }
    }

    // update
    ids = inlier_ids;
    p2Dss_pixel = inlier_p2Dss_pixel;

    return true;
}

// ----------------------------------------------------------------------------
bool ArUco_Detector::run()
{
    // image //////////////////////////////////////////////////////////////////
    cv::Mat image, image_out;

    // port ///////////////////////////////////////////////////////////////////
    double fps;

    cv::VideoCapture cap;
    switch (input_mode_)
    {
        case USB:
            cap = cv::VideoCapture(Config::read<int>("USB_camera_ID"));
            break;

        case VIDEO:
            cap = cv::VideoCapture(Config::read<std::string>("video_file_path"));
            break;

        case TELLO:
            cap = cv::VideoCapture(Config::read<std::string>("tello_video_stream"), cv::CAP_FFMPEG);
            break;

        case RASPBERRY:
            cap = cv::VideoCapture(Config::read<std::string>("raspberry_pipeline"));    
    }
    std::cout << "[ArUco Detector] got cap." << std::endl;

    if (input_mode_ == USB ||
        input_mode_ == VIDEO ||
        input_mode_ == TELLO ||
        input_mode_ == RASPBERRY)
    {
        // check capture
        if (!cap.isOpened()) 
        {
            std::cerr << "ERROR: capturer is not open\n";
            return -1;
        }

        // get FPS
        fps = cap.get(cv::CAP_PROP_FPS);

        std::cout << "FPS: " << fps << std::endl;

        // read first frame to get frame size
        cap >> image; 
    }
    else if (input_mode_ == REALSENSE)
    {
        //
    }

    const int IMAGE_WIDTH  = image.size().width;
    const int IMAGE_HEIGHT = image.size().height;
    cv::Point IMAGE_CENTER(IMAGE_WIDTH / 2, IMAGE_HEIGHT / 2);

    // setting ////////////////////////////////////////////////////////////////
    // main variable ==========================================================
    SE3 T_cm;

    // ArUco ==================================================================
    int target_index = false;
    
    // pose estimation ========================================================
    cv::Vec3d rvec, tvec; // rotation, translation vectors
    cv::Matx33d rmat;

    // in Eigen & Sophus
    Vec3 r_cm, t_cm;
    Mat33 R_cm;

    // filter
    const float POSE_ERROR_THRESHOLD = 0.5;
    float reset_time_window = 0.1; // [s]
    const unsigned int NUM_FRAMES_DURING_RESET_TIME_WINDOW = ceil(reset_time_window * fps);

    ///////////////////////////////////////////////////////////////////////////
    unsigned int frame_count = 0;
    for (;;)
    {   
        // load frame /////////////////////////////////////////////////////////
        // if (input_mode_ == USB ||
        //     input_mode_ == VIDEO ||
        //     input_mode_ == TELLO ||
        //     input_mode_ == RASPBERRY)
        // {
        //     cap >> image;
        // }
        // else if (input_mode_ == REALSENSE)
        // {
        //     //
        // }
        cap >> image;

        // check frame
        if (image.empty()) 
        {
            std::cerr << "ERROR: blank frame\n";
            break;
        }

        // rescale ////////////////////////////////////////////////////////////
        if (input_mode_ == USB ||
            input_mode_ == RASPBERRY)
        {
            cv::resize(image, image, cv::Size(), 
                pre_resize_factor_, pre_resize_factor_, cv::INTER_LINEAR);
        }
        
        // pre-processing /////////////////////////////////////////////////////
        // convert to grayscale
        cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
            
        // // apply blur filter (to reduce noise)
        // cv::blur(image, image, cv::Size(3, 3)); // Sobel

        // --------------------------------------------------------------------
        // convert to BGR for output
        cv::cvtColor(image, image_out, cv::COLOR_GRAY2BGR);
        // --------------------------------------------------------------------

        // main ///////////////////////////////////////////////////////////////
        // detect =============================================================      
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> p2Dss_pixel, rejected_p2Dss_pixel;
        detector_->detectMarkers(image, p2Dss_pixel, ids, rejected_p2Dss_pixel);

        target_index = find_target_index(ids);
        target_found_ = target_index >= 0;

        // estimate pose ======================================================
        if (target_found_)
        {   
            std::vector<cv::Point2f> p2Ds_pixel = p2Dss_pixel.at(target_index);

            // solve initial pose guess with RANSAC
            cv::solvePnPRansac(p3Ds_marker_, p2Ds_pixel, 
                cameraMatrix_, distCoeffs_, rvec, tvec, 
                false, cv::SOLVEPNP_IPPE_SQUARE);

            // convert rotation vector to rotation matrix
            cv::Rodrigues(rvec, rmat);

            // convert to Eigen then Sophus
            R_cm << rmat(0, 0), rmat(0, 1), rmat(0, 2),
                    rmat(1, 0), rmat(1, 1), rmat(1, 2),
                    rmat(2, 0), rmat(2, 1), rmat(2, 2);

            // convert to euler angles ----------------------------------------
            /* <!> (TO DO) ANGLE STRANGE EXCEPT ROLL. NEED FIX 
            https://stackoverflow.com/questions/31589901/euler-to-quaternion-quaternion-to-euler-using-eigen */

            // euler_angles_cm_ = R_cm.eulerAngles(2, 1, 0);
            euler_angles_cm_ = Eigen::EulerAngles<double, Eigen::EulerSystemZYX>(R_cm).angles();
            if (verbose_)
            {
                Vec3 euler_angles_deg = euler_angles_cm_.transpose()*(180/M_PI);
                euler_angles_deg[0] = -euler_angles_deg[0];
                printf("(roll, pitch, yaw): (%5.0f, %5.0f, %5.0f)\n", 
                    euler_angles_deg[0], euler_angles_deg[2], euler_angles_deg[1]);
            }
            // ----------------------------------------------------------------

            t_cm = Vec3(tvec[0], tvec[1], tvec[2]);

            Quaternion q_cm(R_cm);
            T_cm = SE3(q_cm, t_cm); // SE3(SO3(R_cm), t_cm)

            // set pose based on filter
            if (previous_T_cm_)
            {
                double pose_error = compute_pose_error(T_cm);
                if (pose_error < POSE_ERROR_THRESHOLD)
                {
                    // refine pose iteratively using Levenberg-Marquardt method
                    cv::solvePnPRefineLM(p3Ds_marker_, p2Ds_pixel, 
                        cameraMatrix_, distCoeffs_, rvec, tvec);

                    // update
                    T_cm = rvec_and_tvec_to_T(rvec, tvec);
                    T_cm_ = T_cm;
                    previous_T_cm_ = std::make_shared<SE3>(T_cm);

                    // interface ==============================================
                    // t_cm = T_cm.translation();tello_cm_));

                    // update_state_output();
                    /// =======================================================

                    is_pose_ok_ = true;
                }
                else
                {
                    is_pose_ok_ = false;
                    if (verbose_)
                        std::cout << "XXXXXXX <!!!> Pose NOT Good! <!!!> XXXXXXX" << std::endl;
                }
                if (verbose_)
                    std::cout << "pose error: " << pose_error << std::endl;  
            }
        }

        // reset
        if (previous_T_cm_ && frame_count > NUM_FRAMES_DURING_RESET_TIME_WINDOW)
        {
            frame_count = 0;
        }

        if (frame_count == 0)
        {
            previous_T_cm_ = std::make_shared<SE3>(T_cm);
        }

        frame_count += 1;

        // output /////////////////////////////////////////////////////////////
        // draw ---------------------------------------------------------------
        if (!ids.empty())
        {
            cv::aruco::drawDetectedMarkers(image_out, p2Dss_pixel, ids);
        }

        if (target_index >= 0 && is_pose_ok_)
        {
            cv::drawFrameAxes(image_out, cameraMatrix_, distCoeffs_, rvec, tvec, 0.1, 2);
        }

        if (verbose_)
            std::cout << "T_cm:\n" << T_cm_.matrix() << std::endl; 

        // show ===============================================================
        // resize
        cv::resize(image_out, image_out, cv::Size(), resize_scale_factor_, resize_scale_factor_, cv::INTER_LINEAR);

        // show
        cv::imshow("ArUco Tracker", image_out);
        int key = cv::waitKey(10);
        if (key == 27)
        {
            break; // quit when 'esc' pressed
        }
    }
    std::cout << "END" << std::endl;

    return true;
}

// ----------------------------------------------------------------------------
bool ArUco_Detector::run_as_thread()
{
    std::cout << "[ArUco Detector] started running as thread." << std::endl;
    thread_ = std::thread(&ArUco_Detector::run, this);

    return true;
}

// ----------------------------------------------------------------------------
void ArUco_Detector::close()
{
    thread_.join();
}

// ============================================================================
int ArUco_Detector::find_target_index(const std::vector<int>& ids) const
{
    auto iterator = std::find(ids.begin(), ids.end(), target_id_);
    if(iterator != ids.end())
    {
        return iterator - ids.begin();
    }
    else
    {
        return -1;
    }
}

} // namespace tello_slam
