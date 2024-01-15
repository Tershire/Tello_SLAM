// setting.cpp

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 JAN 15
// Wonhee LEE

// reference: ORB-SLAM3, slambook


#include "port/setting.h"


namespace tello_slam
{

// public XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// constructor & destructor ///////////////////////////////////////////////////
Setting::Setting(const std::string& setting_file_path)
{
    // open setting file
    file_ = cv::FileStorage(setting_file_path, cv::FileStorage::READ);
    if (!file_.isOpened())
    {
        std::cerr << "[ERROR]: could not open setting file at: " 
                  << setting_file_path << std::endl;
        std::cerr << "Aborting..." << std::endl;

        exit(-1);
    }
    else
    {
        std::cout << "Loading settings from " << setting_file_path << std::endl;
    }

    read_and_set_usb_camera();

    read_and_set_raspberry_camera();

    read_and_set_color_imager();
}

// private XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// read_parameter =============================================================
// ----------------------------------------------------------------------------
template<>
double Setting::read_parameter<double>(cv::FileStorage& file, 
    const std::string& parameter, 
    bool& found, const bool& required)
{
    cv::FileNode node = file[parameter];
    if(node.empty())
    {
        if (required)
        {
            std::cerr << parameter 
                      << " required parameter does not exist, aborting..." 
                      << std::endl;
            exit(-1);
        }
        else
        {
            std::cerr << parameter 
                      << " optional parameter does not exist..." 
                      << std::endl;
            found = false;
            return 0.0f;
        }
    }
    else if (!node.isReal())
    {
        std::cerr << parameter 
                  << " parameter must be a real number, aborting..."
                  << std::endl;
        exit(-1);
    }
    else
    {
        found = true;

        std::cout << node.real() << std::endl;

        return node.real();
    }
}

// ----------------------------------------------------------------------------
template<>
std::string Setting::read_parameter<std::string>(cv::FileStorage& file, 
    const std::string& parameter, 
    bool& found, const bool& required)
{
    cv::FileNode node = file[parameter];
    if (node.empty())
    {
        if (required)
        {
            std::cerr << parameter 
                      << " required parameter does not exist, aborting..." 
                      << std::endl;
            exit(-1);
        }
        else
        {
            std::cerr << parameter 
                      << " optional parameter does not exist..." 
                      << std::endl;
            found = false;
            return std::string();
        }
    }
    else if (!node.isString())
    {
        std::cerr << parameter 
                  << " parameter must be a string, aborting..."
                  << std::endl;
        exit(-1);
    }
    else
    {
        found = true;
        return node.string();
    }
}

// ----------------------------------------------------------------------------
template<>
cv::Mat Setting::read_parameter<cv::Mat>(cv::FileStorage& file, 
    const std::string& parameter, 
    bool& found, const bool& required)
{
    cv::FileNode node = file[parameter];
    if(node.empty())
    {
        if (required)
        {
            std::cerr << parameter 
                      << " required parameter does not exist, aborting..." 
                      << std::endl;
            exit(-1);
        }
        else
        {
            std::cerr << parameter 
                      << " optional parameter does not exist..."
                      << std::endl;
            found = false;
            return cv::Mat();
        }
    }
    else
    {
        found = true;
        return node.mat();
    }
}

// read camera ================================================================
// USB ------------------------------------------------------------------------
void Setting::read_and_set_usb_camera()
{
    bool found;
    std::vector<cv::Mat> intrinsic_parameters;

    // read camera model
    std::string camera_model = read_parameter<std::string>(file_, "USB.camera_model", found);

    if (camera_model == "Pinhole")
    {
        std::cout << "Setting Pinhole USB camera..." << std::endl;

        // read camera intrinsic parameters
        cv::Mat cameraMatrix = read_parameter<cv::Mat>(file_, "USB.K", found);

        std::cout << cameraMatrix << std::endl;

        // intrinsics
        intrinsic_parameters.push_back(cameraMatrix);

        // extrinsics       
        usb_camera_= std::make_shared<Pinhole>(intrinsic_parameters, SE3(SO3(), Vec3(0, 0, 0)));
    }
    else if (camera_model == "Brown-Conrady")
    {
        std::cout << "Setting Brown-Conrady USB camera..." << std::endl;

        // read camera intrinsic parameters
        cv::Mat cameraMatrix = read_parameter<cv::Mat>(file_, "USB.K", found);
        cv::Mat distCoeffs = read_parameter<cv::Mat>(file_, "USB.D", found);

        std::cout << cameraMatrix << std::endl;
        
        // intrinsics
        intrinsic_parameters.push_back(cameraMatrix);
        intrinsic_parameters.push_back(distCoeffs);

        // extrinsics       
        usb_camera_= std::make_shared<Brown_Conrady>(intrinsic_parameters, SE3(SO3(), Vec3(0, 0, 0)));
    }
    else
    {
        std::cerr << "Error: " << camera_model << " not known" << std::endl;
        exit(-1);
    }
    std::cout << "\t-Loaded USB camera" << std::endl;
}

// raspberry ------------------------------------------------------------------
void Setting::read_and_set_raspberry_camera()
{
    bool found;
    std::vector<cv::Mat> intrinsic_parameters;

    // read camera model
    std::string camera_model = read_parameter<std::string>(file_, "raspberry.camera_model", found);

    if (camera_model == "Pinhole")
    {
        std::cout << "Setting Pinhole raspberry camera..." << std::endl;

        // read camera intrinsic parameters
        cv::Mat cameraMatrix = read_parameter<cv::Mat>(file_, "raspberry.K", found);

        std::cout << cameraMatrix << std::endl;

        // intrinsics
        intrinsic_parameters.push_back(cameraMatrix);

        // extrinsics       
        raspberry_camera_= std::make_shared<Pinhole>(intrinsic_parameters, SE3(SO3(), Vec3(0, 0, 0)));
    }
    else if (camera_model == "Brown-Conrady")
    {
        std::cout << "Setting Brown-Conrady raspberry camera..." << std::endl;

        // read camera intrinsic parameters
        cv::Mat cameraMatrix = read_parameter<cv::Mat>(file_, "raspberry.K", found);
        cv::Mat distCoeffs = read_parameter<cv::Mat>(file_, "raspberry.D", found);

        std::cout << cameraMatrix << std::endl;
        
        // intrinsics
        intrinsic_parameters.push_back(cameraMatrix);
        intrinsic_parameters.push_back(distCoeffs);

        // extrinsics       
        raspberry_camera_= std::make_shared<Brown_Conrady>(intrinsic_parameters, SE3(SO3(), Vec3(0, 0, 0)));
    }
    else
    {
        std::cerr << "Error: " << camera_model << " not known" << std::endl;
        exit(-1);
    }
    std::cout << "\t-Loaded raspberry camera" << std::endl;
}

// realsense ------------------------------------------------------------------
void Setting::read_and_set_color_imager()
{
    bool found;
    std::vector<cv::Mat> intrinsic_parameters;

    // read camera model
    std::string camera_model = read_parameter<std::string>(file_, "color_imager.camera_model", found);

    if (camera_model == "Pinhole")
    {
        std::cout << "Setting Pinhole color imager..." << std::endl;

        // read camera intrinsic parameters
        cv::Mat cameraMatrix = read_parameter<cv::Mat>(file_, "color_imager.K", found);

        std::cout << cameraMatrix << std::endl;

        // intrinsics
        intrinsic_parameters.push_back(cameraMatrix);

        // extrinsics       
        color_imager_= std::make_shared<Pinhole>(intrinsic_parameters, SE3(SO3(), Vec3(0, 0, 0)));
    }
    else if (camera_model == "Brown-Conrady")
    {
        std::cout << "Setting Brown-Conrady color imager..." << std::endl;

        // read camera intrinsic parameters
        cv::Mat cameraMatrix = read_parameter<cv::Mat>(file_, "color_imager.K", found);
        cv::Mat distCoeffs = read_parameter<cv::Mat>(file_, "color_imager.D", found);

        std::cout << cameraMatrix << std::endl;
        
        // intrinsics
        intrinsic_parameters.push_back(cameraMatrix);
        intrinsic_parameters.push_back(distCoeffs);

        // extrinsics       
        color_imager_= std::make_shared<Brown_Conrady>(intrinsic_parameters, SE3(SO3(), Vec3(0, 0, 0)));
    }
    else
    {
        std::cerr << "Error: " << camera_model << " not known" << std::endl;
        exit(-1);
    }
    std::cout << "\t-Loaded color imager" << std::endl;
}

void Setting::read_and_set_depth_imagers(const int& num_cameras)
{
    // (!) NOT YET IMPLEMENTED
    for (int camera_id = 0; camera_id < num_cameras; ++camera_id)
    {
    }
}

} // namespace tello_slam
