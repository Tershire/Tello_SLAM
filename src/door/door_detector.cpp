// door_detector.cpp

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 FEB 23
// Wonhee LEE

// reference:


#include <fstream> // std::ifstream
#include <opencv2/tracking.hpp>

#include "door/door_detector.h"
#include "port/config.h"
#include "toolbox/conversion_toolbox.h"


namespace tello_slam
{

// public XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// constructor & destructor ///////////////////////////////////////////////////
Door_Detector::Door_Detector(Camera::Ptr camera)
    : camera_(camera)
{
    // neural net =============================================================
    std::string cfg_path, weights_path;
    cfg_path     = Config::read<std::string>("cfg_path");
    weights_path = Config::read<std::string>("weights_path");

    // load network
    net_ = cv::dnn::readNetFromDarknet(cfg_path, weights_path);
    net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

    // thresholds
    confidence_threshold_ = Config::read<float>("confidence_threshold");
    nms_threshold_ = Config::read<float>("nms_threshold");

    // check info.
    output_layer_names_ = get_output_layer_names();
    std::cout << "output layer names: " << std::endl;
    for (auto& name : output_layer_names_)
    {
        std::cout << name << std::endl;
    }

    // load classes
    std::string names_path = Config::read<std::string>("names_path");
    classes_ = get_classes(names_path);

    std::cout << "\t-Loaded neural net" << std::endl;

    // tracking ===============================================================
    tracker_ = cv::TrackerCSRT::create();

    // port ===================================================================
    std::string input_mode = Config::read<std::string>("input_mode");
    if (input_mode == "usb")
        input_mode_ = Input_Mode::USB;
    else if (input_mode == "video")
        input_mode_ = Input_Mode::VIDEO;
    else if (input_mode == "raspberry")
        input_mode_ = Input_Mode::RASPBERRY;
    else if (input_mode == "realsense")
        input_mode_ = Input_Mode::REALSENSE;
    else
        std::cout << "ERROR: input mode wrong\n";

    resize_scale_factor_ = Config::read<float>("resize_scale_factor");
}

// member methods /////////////////////////////////////////////////////////////
// ----------------------------------------------------------------------------
bool Door_Detector::run()
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

        case RASPBERRY:
            cap = cv::VideoCapture(Config::read<std::string>("raspberry_pipeline")); 
    }

    if (input_mode_ == USB ||
        input_mode_ == VIDEO ||
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

    ///////////////////////////////////////////////////////////////////////////
    for (;;)
    {
        // load frame /////////////////////////////////////////////////////////
        if (input_mode_ == USB ||
            input_mode_ == VIDEO ||
            input_mode_ == RASPBERRY)
        {
            cap >> image;
        }
        else if (input_mode_ == REALSENSE)
        {
            //
        }

        // check frame
        if (image.empty()) 
        {
            std::cerr << "ERROR: blank frame\n";
            break;
        }
        
        // pre-processing /////////////////////////////////////////////////////            
        // apply blur filter (to reduce noise)
        cv::blur(image, image, cv::Size(3, 3)); // Sobel

        // main ///////////////////////////////////////////////////////////////
        std::vector<int> class_IDs;
        std::vector<float> confidences;
        std::vector<cv::Rect> boxes;

        int class_ID;
        float confidence;
        cv::Rect box;

        cv::Rect tracker_box;
        double iou;

        cv::Point IMAGE_CENTER(image.size().width/2, image.size().height/2);    
        cv::Point ref_p2D_pixel = IMAGE_CENTER;

        if (is_for_tracking_only_one_)
        {
            detect_or_track_only_one(image, ref_p2D_pixel,
                class_IDs, confidences, boxes,
                class_ID, confidence, box,
                tracker_box, iou);

            // std::cout << "box.x: " << box.x << std::endl;
        }
        else
        {
            is_detected_ = detect(image, class_IDs, confidences, boxes);
        }      

        // output /////////////////////////////////////////////////////////////
        // draw
        image_out = image.clone();
        if (is_for_tracking_only_one_)
        {
            if (is_detected_) 
            {
                draw_predictions(image_out, class_IDs, confidences, boxes);
                // draw_prediction(image_out, class_ID, confidence, box);
                draw_box(image_out, tracker_box, cv::Scalar(255, 0, 0));
            }
        }
        else
        {
            draw_predictions(image_out, class_IDs, confidences, boxes);
        }

        // show ===============================================================
        // resize
        cv::resize(image_out, image_out, cv::Size(), resize_scale_factor_, resize_scale_factor_, cv::INTER_LINEAR);

        // show
        cv::imshow("Door Detector", image_out);
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
bool Door_Detector::detect(cv::Mat& image,
    std::vector<int>& class_IDs,
    std::vector<float>& confidences,
    std::vector<cv::Rect>& boxes)
{
    // set input
    net_.setInput(cv::dnn::blobFromImage(image, 1/255.0, cv::Size(416, 416), cv::Scalar(0,0,0), true, false));
    
    // main
    std::vector<cv::Mat> output_blobs;
    net_.forward(output_blobs, output_layer_names_);

    // post-processing
    post_process(image, output_blobs, class_IDs, confidences, boxes);

    // check if anyone is found
    if (boxes.empty())
        return false;
    else
        return true; 
}

// ----------------------------------------------------------------------------
void Door_Detector::detect_or_track_only_one(cv::Mat& image, const cv::Point& ref_p2D_pixel,
    std::vector<int>& class_IDs,
    std::vector<float>& confidences,
    std::vector<cv::Rect>& boxes,
    int& class_ID,
    float& confidence,
    cv::Rect& box, 
    cv::Rect& tracker_box, double& iou)
{
    detect(image, class_IDs, confidences, boxes);

    if (is_tracking_lost_)
    {
        std::cout << "Detecting..." << std::endl;

        is_detected_ = choose_only_one(image, ref_p2D_pixel, 
            class_IDs, confidences, boxes,
            class_ID, confidence, box);

        if (is_detected_) 
        {
            tracker_->init(image, box);

            is_tracking_lost_ = false;
        }
    }
    else
    {
        std::cout << "->->-> Tracking..." << std::endl;

        tracker_->update(image, tracker_box);

        is_tracking_lost_ = true;
        for (auto& box_to_check : boxes)
        {
            iou = compute_IoU(box_to_check, tracker_box);
            if (iou > 0.5)
            {
                is_tracking_lost_ = false;
                break;
            }
        }
    }
}

// ----------------------------------------------------------------------------
bool Door_Detector::choose_only_one(cv::Mat& image, const cv::Point& ref_p2D_pixel,
    std::vector<int>& class_IDs,
    std::vector<float>& confidences,
    std::vector<cv::Rect>& boxes,
    int& class_ID,
    float& confidence,
    cv::Rect& box)
{
    bool is_chosen = false;

    // search the best one
    if (boxes.size() == 1)
    {
        class_ID = class_IDs[0];
        confidence = confidences[0];
        box = boxes[0];

        if (confidence > 0.9) // filter with confidence
        {
            is_chosen = true;
        }
    }
    else if (boxes.size() > 1)
    {
        is_chosen = find_the_nearest_one(image,
            ref_p2D_pixel,
            class_IDs, confidences, boxes,
            class_ID, confidence, box);
    }

    return is_chosen;
}

// ----------------------------------------------------------------------------
void Door_Detector::draw_predictions(cv::Mat& image, 
    std::vector<int>& class_IDs,
    std::vector<float>& confidences,
    std::vector<cv::Rect>& boxes)
{
    cv::Rect box;
    for (size_t i = 0; i < class_IDs.size(); ++i)
    {
        box = boxes[i];
        draw_prediction(image,
            class_IDs[i], 
            confidences[i],
            boxes[i]);
    }
}

// ----------------------------------------------------------------------------
void Door_Detector::draw_prediction(cv::Mat& image, 
    const unsigned int& class_ID, 
    const float& confidence,
    const cv::Rect& box)
{
    // draw bounding box
    draw_box(image, box, cv::Scalar(0, 255, 0));
    
    // get class name and confidence
    std::string label = cv::format("%.2f", confidence);
    if (!classes_.empty())
    {
        assert(class_ID < classes_.size());
        label = classes_.at(class_ID) + ": " + label;
    }
    
    // draw label
    cv::putText(image, label, cv::Point(box.x, box.y - 7), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 255, 0), 2);
}

// ----------------------------------------------------------------------------
std::vector<std::string> Door_Detector::get_output_layer_names()
{
    static std::vector<std::string> output_layer_names;
    if (output_layer_names.empty())
    {
        // get the names of all layers in the network
        std::vector<std::string> layer_names = net_.getLayerNames();

        // get indices of the output layers
        std::vector<int> output_layer_IDs = net_.getUnconnectedOutLayers();
         
        // get output layer names filtered from the all layer names
        output_layer_names.resize(output_layer_IDs.size());
        for (unsigned int i = 0; i < output_layer_IDs.size(); ++i)
        {
            output_layer_names[i] = layer_names[output_layer_IDs[i] - 1];
        }
    }
    return output_layer_names;
}

// ----------------------------------------------------------------------------
std::map<unsigned int, std::string> Door_Detector::get_classes(const std::string& names_path)
{
    std::map<unsigned int, std::string> classes;

    unsigned int class_ID = 0;

    std::ifstream input_stream(names_path.c_str());
    std::string line;
    while (getline(input_stream, line)) 
    {
        classes.insert({class_ID, line});
        class_ID += 1;
    }

    return classes;
}

// private XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// member methods /////////////////////////////////////////////////////////////
// ----------------------------------------------------------------------------
void Door_Detector::post_process(cv::Mat& image, const std::vector<cv::Mat>& output_blobs,
    std::vector<int>& class_IDs,
    std::vector<float>& confidences,
    std::vector<cv::Rect>& boxes)
{
    for (size_t i = 0; i < output_blobs.size(); ++i)
    {
        cv::Mat output_blob = output_blobs[i];
        float* data = (float*) output_blob.data;
        for (int j = 0; j < output_blob.rows; ++j)
        {
            /// get scores, which are located from column 5
            cv::Mat scores = output_blob.row(j).colRange(5, output_blob.cols);
            
            // get value & location of max score (class confidence)
            double confidence;
            cv::Point class_ID_point;
            cv::minMaxLoc(scores, 0, &confidence, 0, &class_ID_point);
            if (confidence > confidence_threshold_)
            {
                int center_x = int(data[0] * image.cols);
                int center_y = int(data[1] * image.rows);
                int width  = int(data[2] * image.cols);
                int height = int(data[3] * image.rows);
                int left = center_x - width  / 2;
                int top  = center_y - height / 2;
                
                class_IDs.push_back(class_ID_point.x);
                confidences.push_back(float(confidence)); 
                boxes.push_back(cv::Rect(left, top, width, height)); 
            }
            data += output_blob.cols;
        }
    }
    
    // Non-Maximum Suppression
    std::vector<int> filtered_class_IDs;
    std::vector<float> filtered_confidences;
    std::vector<cv::Rect> filtered_boxes;
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, confidence_threshold_, nms_threshold_, indices);

    int index;
    for (size_t i = 0; i < indices.size(); ++i)
    {
        index = indices[i];

        filtered_class_IDs.push_back(class_IDs[index]);
        filtered_confidences.push_back(confidences[index]);
        filtered_boxes.push_back(boxes[index]);
    }

    // overwrite
    class_IDs = filtered_class_IDs;
    confidences = filtered_confidences;
    boxes = filtered_boxes;
}

// criteria to detect only one cone ===========================================
bool Door_Detector::find_the_nearest_one(cv::Mat& image,
    cv::Point ref_p2D_pixel,
    std::vector<int>& class_IDs,
    std::vector<float>& confidences,
    std::vector<cv::Rect>& boxes,
    int& class_ID,
    float& confidence,
    cv::Rect& box)
{
    bool is_detected = false;

    // setting
    std::vector<double> distances;
    double distance;
    int nearest_one_index = 0;
    cv::Point box_centroid;

    for (unsigned int i = 0; i < boxes.size(); ++i)
    {
        class_ID = class_IDs[i];
        confidence = confidences[i];
        box = boxes[i];

        if (confidence > 0.9) // filter with confidence
        {
            is_detected = true;
            box_centroid = compute_box_centroid(box);
            distance = (Vec2(ref_p2D_pixel.x, ref_p2D_pixel.y)
                      - Vec2(box_centroid.x, box_centroid.y)).norm();  
        }
        else
        {
            distance = std::numeric_limits<float>::infinity();
        }
        distances.push_back(distance);
    }

    nearest_one_index = std::distance(std::begin(distances), 
        std::min_element(std::begin(distances), std::end(distances)));
        
    // update
    class_ID = class_IDs[nearest_one_index];
    confidence = confidences[nearest_one_index];
    box = boxes[nearest_one_index];

    // check
    distance = distances[nearest_one_index];

    if (is_detected) 
    {
        std::cout << "nearest_one_index, min_distance: " << nearest_one_index << ", " << distance << std::endl;
    }
    
    return is_detected;
}

} // namespace tello_slam
