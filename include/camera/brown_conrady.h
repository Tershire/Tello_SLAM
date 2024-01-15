// brown_conrady.h

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 JAN 15
// Wonhee LEE

// reference:


#ifndef TELLOSLAM_CAMERA_BROWNCONRADY_H
#define TELLOSLAM_CAMERA_BROWNCONRADY_H

// #include <assert.h>

#include "camera/camera.h"


namespace tello_slam
{

/**
 * 
 */
class Brown_Conrady: public Camera
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    // constructor & destructor ///////////////////////////////////////////////
    Brown_Conrady()
    {
        camera_model_ = BROWN_CONRADY;
    }

    Brown_Conrady(const std::vector<cv::Mat>& intrinsic_parameters, const SE3& T_cNc)
        : Camera(intrinsic_parameters, T_cNc)
    {   
        distCoeffs_ = intrinsic_parameters.at(1);

        k1_ = distCoeffs_.at<double>(0);
        k2_ = distCoeffs_.at<double>(1);
        p1_ = distCoeffs_.at<double>(2);
        p2_ = distCoeffs_.at<double>(3);
        k3_ = distCoeffs_.at<double>(4);

        camera_model_ = BROWN_CONRADY;
    }

    // virtual member methods /////////////////////////////////////////////////
    Vec2 camera_to_pixel(const Vec3& p3D_camera)
    {
        std::vector<cv::Point2f> imagePoints;

        cv::Point3f objectPoint(p3D_camera[0], p3D_camera[1], p3D_camera[2]);
        std::vector<cv::Point3f> objectPoints;
        objectPoints.push_back(objectPoint);
        cv::Vec3d rvec(0, 0, 0);
        cv::Vec3d tvec(0, 0, 0);
        cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix_, distCoeffs_,
            imagePoints);

        cv::Point2f imagePoint = imagePoints.at(0);

        return Vec2(imagePoint.x, imagePoint.y);
    }

    Vec3 pixel_to_normalized_image_plane(const Vec2& p2D_pixel)
    {
        Vec3 p3Dn_pinhole = pixel_to_normalized_image_plane_pinhole(p2D_pixel);
        double x_u_n = p3Dn_pinhole[0];
        double y_u_n = p3Dn_pinhole[1];

        double r_u_n_2 = Vec2(x_u_n, y_u_n).squaredNorm();
        double r_u_n_4 = r_u_n_2 * r_u_n_2,
               r_u_n_6 = r_u_n_4 * r_u_n_2;
        
        double radial_distortion_factor = (1 + k1_*r_u_n_2 + k2_*r_u_n_4 + k3_*r_u_n_6);
        double x_d_n = x_u_n * radial_distortion_factor + 2*p1_*x_u_n*y_u_n + p2_*(r_u_n_2 + 2*x_u_n*x_u_n);
        double y_d_n = y_u_n * radial_distortion_factor + 2*p2_*x_u_n*y_u_n + p1_*(r_u_n_2 + 2*y_u_n*y_u_n);                          

        return Vec3(x_d_n, y_d_n, 1);
    }

    Vec3 pixel_to_normalized_image_plane_pinhole(const Vec2& p2D_pixel)
    {
        return Vec3((p2D_pixel[0] - cx_) / fx_,
                    (p2D_pixel[1] - cy_) / fy_,
                    1);
    }

private:
    // member data ////////////////////////////////////////////////////////////
    double k1_, k2_, p1_, p2_, k3_;
};

} // namespace tello_slam

#endif // TELLOSLAM_CAMERA_BROWNCONRADY_H
