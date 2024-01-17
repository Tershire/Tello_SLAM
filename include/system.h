// system.h

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 JAN 15
// Wonhee LEE

// reference:


#ifndef TELLOSLAM_SYSTEM_H
#define TELLOSLAM_SYSTEM_H

#include "common.h"
#include "port/setting.h"
#include "camera/camera.h"
#include "marker/aruco_detector.h"


namespace tello_slam
{

/**
 * vision system
 */
class System
{
public:
    typedef std::shared_ptr<System> Ptr;

    // state member ///////////////////////////////////////////////////////////
    enum Mode
    {
        TEST,
        MISSION
    };

    // member data ////////////////////////////////////////////////////////////
    // mode ===================================================================
    Mode mode_;

    // verbosity ==============================================================
    bool verbose_;

    // constructor & destructor ///////////////////////////////////////////////
    System(const std::string& configuration_file_path);

    // getter & setter ////////////////////////////////////////////////////////
    // getter =================================================================
    ArUco_Detector::Ptr get_aruco_detector() const {return aruco_detector_;}

    // member methods /////////////////////////////////////////////////////////
    /**
     * initialize system
     * @return true if success
     */
    bool initialize();
    
    /**
     * run system in infinite loop
     */
    void run();

    // /**
    //  * step forward
    //  */
    // bool step();

private:
    // member data ////////////////////////////////////////////////////////////
    std::string configuration_file_path_;
    
    // port ===================================================================
    Setting::Ptr setting_ = nullptr;

    // camera -----------------------------------------------------------------
    Camera::Ptr usb_camera_;
    Camera::Ptr raspberry_camera_;
    Camera::Ptr color_imager_;
    std::vector<Camera::Ptr> depth_imagers_;
    
    Camera::Ptr mono_camera_;
    std::string mono_camera_type_;
    float mono_camera_scale_factor_;    

    // system components ======================================================
    ArUco_Detector::Ptr aruco_detector_ = nullptr;

    // ArUco Detector =========================================================
    std::string predifined_dictionary_name_;
    float marker_length_;
};

} // namespace tello_slam

#endif // TELLOSLAM_SYSTEM_H
