// dataset.cpp

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 MAR 11
// Wonhee LEE

// reference: slambook


#include <fstream>
#include <opencv2/opencv.hpp>

#include "dataset.h"
#include "frame.h"
#include "config.h"


namespace tello_slam
{

// public XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// constructor & destructor ///////////////////////////////////////////////////
Dataset::Dataset(const std::string& dataset_directory_path):
    dataset_directory_path_(dataset_directory_path)
{
    cv::glob(dataset_directory_path + "/*.png", file_paths_, false);

    num_images_ = file_paths_.size();

    // resize =================================================================
    pre_resize_factor_ = Config::read<float>("pre_resize_factor"); 

    // pre-processing =========================================================
    clahe_on_ = Config::read<int>("clahe_on");
    if (clahe_on_)
    {
        clahe_ = cv::createCLAHE(3.0, cv::Size(8, 8));
    }

    // load UNIX timestamps ===================================================
    // to estimate motion data (ex. velocity)
    motion_log_on_ = Config::read<int>("motion_log_on");
    if (motion_log_on_)
    {
        std::cout << "reading times file from: " << (dataset_directory_path + "/times.txt") << "..." << std::endl;
        load_timestamps(dataset_directory_path + "/times.txt");
        std::cout << "\t-loaded timestamps having size: " << timestamps_.size() << std::endl;
    }
}

// ----------------------------------------------------------------------------
Frame::Ptr Dataset::load_next_frame()
{
    cv::Mat image;
    if (current_image_index_ < num_images_)
    {
        // read images
        image = cv::imread(file_paths_[current_image_index_], cv::IMREAD_GRAYSCALE);

        // CLAHE --------------------------------------------------------------
        // probably slower than reading as grayscale
        // image_L = cv::imread(file_paths_L_[current_image_index_], cv::IMREAD_COLOR);

        // cv::cvtColor(image_L, image_L, cv::COLOR_BGR2Lab);
        // --------------------------------------------------------------------

        //
        std::cout << "image name: " << file_paths_[current_image_index_] << std::endl;
    }

    // ------------------------------------------------------------------------
    if (image.data == nullptr)
    {
        std::cout << "ERROR: cannot find images at index " << current_image_index_ << std::endl;
        return nullptr;
    }

    // pre-resizing -----------------------------------------------------------
    cv::resize(image, image, cv::Size(), pre_resize_factor_, pre_resize_factor_,
        cv::INTER_NEAREST);    
    // ------------------------------------------------------------------------

    // pre-processing: CLAHE ==================================================
    if (clahe_on_)
    {   
        // // Extract L channel from LAB
        // cv::extractChannel(image_L, image_L, 0);
        // cv::extractChannel(image_R, image_R, 0);

        // apply CLAHE
        clahe_->apply(image, image);
    }
    // ========================================================================
    
    auto frame = Frame::create_frame();

    frame->image_ = image;

    current_image_index_ += 1;

    if (motion_log_on_) 
    {
        frame->timestamp_ = load_next_timestamp();
    }

    return frame;
}

// ----------------------------------------------------------------------------
void Dataset::load_timestamps(const std::string& times_file_path)
{
    std::ifstream file_stream(times_file_path);

    if (file_stream.is_open())
    {
        size_t index = 0;
        long long initial_time;
        long long time;
        while (file_stream >> time)
        {
            if (index == 0)
            {
                initial_time = time;
            }

            timestamps_.push_back((time - initial_time) * 1E-9);
            
            index += 1;
        }
    }
    else
    {
        std::cout << "ERROR: can't open times file" << std::endl;
    }
}

// ----------------------------------------------------------------------------
double Dataset::load_next_timestamp()
{
    double timestamp = timestamps_.front();
    timestamps_.pop_front();
    return timestamp;
}

} // namespace tello_slam
