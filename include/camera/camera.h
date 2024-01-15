// camera.h

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 JAN 15
// Wonhee LEE

// reference:


#ifndef TELLOSLAM_CAMERA_CAMERA_H
#define TELLOSLAM_CAMERA_CAMERA_H

#include "common.h"


namespace tello_slam
{

/**
 * 
 */
class Camera
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Camera> Ptr;

    // state member ///////////////////////////////////////////////////////////
    enum Camera_Model
    {
        PINHOLE,
        BROWN_CONRADY,
        KANNALA_BRANDT
    };

    // member data ////////////////////////////////////////////////////////////   
    std::vector<cv::Mat> intrinsic_parameters_;

    cv::Mat cameraMatrix_; // camera matrix
    cv::Mat distCoeffs_;

    double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0;
    
    SE3 T_cNc_; // pose T_cNc (multiple camera system -> cameraN) // camera: coordinate system of the multiple camera system. (coincides with camera1: so, T_cNc is identity for camera1)
    SE3 T_cNc_inv_;

    Camera_Model camera_model_;

    // constructor & destructor ///////////////////////////////////////////////
    Camera() {}
    
    Camera(const std::vector<cv::Mat>& intrinsic_parameters, const SE3& T_cNc);

    // getter & setter ////////////////////////////////////////////////////////
    // getter =================================================================
    SE3 get_T_cNc() const {return T_cNc_;}

    // virtual member methods /////////////////////////////////////////////////
    /**
     * project
     * @return p2D_pixel (of cN)
     */
    virtual Vec2 camera_to_pixel(const Vec3& p3D_camera) = 0;

    /**
     * unproject on normalized image plane
     * @return p3D_normalized (of cN)
     */
    virtual Vec3 pixel_to_normalized_image_plane(const Vec2& p2D_pixel) = 0;

    // member methods /////////////////////////////////////////////////////////
    // coordinate system transform ============================================
    /**
     * @return p3D_camera (of cN)
     */
    // Vec3 pixel_to_camera(const Vec2& p2D_pixel, double depth) // depth: z in camera coordinate system
    // {
    //     return pixel_to_normalized_image_plane(p2D_pixel) * depth;
    // }

    /**
     * @return p3D_camera (of cN)
     */
    Vec3 world_to_camera(const Vec3& p3D_world, const SE3& T_cw)
    {
        return T_cNc_ * T_cw * p3D_world;
    }

    /**
     * @return p2D_pixel (of cN)
     */
    Vec2 world_to_pixel(const Vec3& p3D_world, const SE3& T_cw)
    {
        return camera_to_pixel(world_to_camera(p3D_world, T_cw));
    }

    // ========================================================================
    /**
     * rescale
     */
    void rescale(const float& scale_factor);
};

} // namespace tello_slam

#endif // TELLOSLAM_CAMERA_CAMERA_H
