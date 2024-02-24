// door_detector.h

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 FEB 23
// Wonhee LEE

// reference:


#ifndef TELLOSLAM_DOORDETECTOR_H
#define TELLOSLAM_DOORDETECTOR_H

#include "common.h"
#include "camera/camera.h"
#include "toolbox/utility_toolbox.h"


namespace tello_slam
{

/**
 * detect
 */
class Door_Detector
{
public:
    typedef std::shared_ptr<Door_Detector> Ptr;

    // state member ///////////////////////////////////////////////////////////
    enum Input_Mode
    {
        USB,
        VIDEO,
        RASPBERRY,
        REALSENSE
    };

    // member data ////////////////////////////////////////////////////////////
    bool verbose_;
    float input_resize_factor_;

    // neural net =============================================================
    std::map<unsigned int, std::string> classes_;

    // constructor & destructor ///////////////////////////////////////////////
    Door_Detector() {}

    Door_Detector(Camera::Ptr camera);

    // getter & setter ////////////////////////////////////////////////////////
    // getter =================================================================
    cv::dnn::Net get_net() const {return net_;}

    bool get_is_detected() const {return is_detected_;}

    // setter =================================================================
    void set_is_for_tracking_only_one(bool is_for_tracking_only_one) 
    {
        is_for_tracking_only_one_ = is_for_tracking_only_one;
    }

    // port -------------------------------------------------------------------
    void set_input_mode(const Input_Mode& input_mode) {input_mode_ = input_mode;}
    void set_verbose(const bool& verbose) {verbose_ = verbose;}
    void set_input_resize_factor(const float& input_resize_factor) {input_resize_factor_ = input_resize_factor;}

    // member methods /////////////////////////////////////////////////////////
    /**
     * object detection loop   
     */
    bool run();

    /**
     * detect objects in an image 
     */
    bool detect(cv::Mat& image,
        std::vector<int>& class_IDs,
        std::vector<float>& confidences,
        std::vector<cv::Rect>& boxes);

    /**
     * detect or track only one cone 
     */
    void detect_or_track_only_one(cv::Mat& image, const cv::Point& ref_p2D_pixel,
        std::vector<int>& class_IDs,
        std::vector<float>& confidences,
        std::vector<cv::Rect>& boxes,
        int& class_ID,
        float& confidence,
        cv::Rect& box,
        cv::Rect& tracker_box, double& iou);

    /**
     * detect only on object in an image according to the criteria
     */
    bool choose_only_one(cv::Mat& image, const cv::Point& ref_p2D_pixel,
        std::vector<int>& class_IDs,
        std::vector<float>& confidences,
        std::vector<cv::Rect>& boxes,
        int& class_ID,
        float& confidence,
        cv::Rect& box);

    /**
     * draw predictions
     */
    void draw_predictions(cv::Mat& image, 
        std::vector<int>& class_IDs,
        std::vector<float>& confidences,
        std::vector<cv::Rect>& boxes);

    /**
     * draw a prediction with its bounding box, class, and confidence level
     */
    void draw_prediction(cv::Mat& image, 
        const unsigned int& class_ID, 
        const float& confidence,
        const cv::Rect& box);

    /**
     * get output layer names of the net
     */
    std::vector<std::string> get_output_layer_names();

    /**
     * read classes
     * @return dictionary of <class ID, class name>
     */
    std::map<unsigned int, std::string> get_classes(const std::string& names_path);

private:
    // member data ////////////////////////////////////////////////////////////
    // configuration ==========================================================
    bool is_for_tracking_only_one_ = false;

    // camera =================================================================
    Camera::Ptr camera_;

    // neural net =============================================================
    cv::dnn::Net net_;
    float score_threshold_, confidence_threshold_, nms_threshold_;
    std::vector<std::string> output_layer_names_;

    bool is_detected_ = false;

    bool letter_box_for_square_ = true;
    cv::Size2f input_shape_ = {640, 640};

    // tracking ===============================================================
    cv::Ptr<cv::Tracker> tracker_;
    bool is_tracking_lost_ = true;

    // port ===================================================================
    Input_Mode input_mode_;
    float resize_scale_factor_;

    // member methods /////////////////////////////////////////////////////////
    /**
     * post processing for YOLO (reading from output blobs and applying NMS) 
     */
    void post_process(cv::Mat& image, const std::vector<cv::Mat>& output_blobs,
        std::vector<int>& class_IDs,
        std::vector<float>& confidences,
        std::vector<cv::Rect>& boxes,
        cv::Mat& input);

    // criteria to detect only one cone =======================================
    bool find_the_nearest_one(cv::Mat& image,
        cv::Point ref_p2D_pixel,
        std::vector<int>& class_IDs,
        std::vector<float>& confidences,
        std::vector<cv::Rect>& boxes,
        int& class_ID,
        float& confidence,
        cv::Rect& box);

    // ========================================================================
    std::tuple<float, float> get_model_factors(cv::Mat& input, 
        std::vector<cv::Mat>& output_blobs,
        bool& is_yolo_v8);

    cv::Mat format_to_square(const cv::Mat& source);
};

} // namespace tello_slam

#endif // TELLOSLAM_DOORDETECTOR_H
