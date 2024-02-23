// utility_toolbox.h

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 FEB 23
// Wonhee LEE

// reference:


#ifndef TELLOSLAM_UTILITYTOOLBOX_H
#define TELLOSLAM_UTILITYTOOLBOX_H

#include "common.h"


namespace tello_slam
{

/**
 * crop RoI for a box with a margin if needed
 */
inline cv::Mat crop_RoI(cv::Mat image, const cv::Rect& box, 
    const float& percentage_margin)
{
    int row_ini, row_fin, col_ini, col_fin;
    row_ini = box.y - box.height * (percentage_margin / 100);
    if (row_ini < 0) row_ini = 0;
    row_fin = box.y + box.height * (1 + percentage_margin / 100);
    if (row_fin > image.rows) row_fin = image.rows;
    col_ini = box.x - box.width * (percentage_margin / 100);
    if (col_ini < 0) col_ini = 0;
    col_fin = box.x + box.width * (1 + percentage_margin / 100);
    if (col_fin > image.cols) col_fin = image.cols;
    
    return image(cv::Range(row_ini, row_fin), 
                 cv::Range(col_ini, col_fin));
}

/**
 * compute centroid of a box 
 */
inline cv::Point compute_box_centroid(const cv::Rect& box)
{    
    return cv::Point(box.x + box.width / 2,
                     box.y + box.height / 2);
}

/**
 * compute IoU of two boxes 
 */
inline double compute_IoU(const cv::Rect& box1, const cv::Rect& box2)
{
    double intersection_area, union_area;

    // intersection
    int x1, x2, y1, y2;
    x1 = std::max(box1.x, box2.x);
    y1 = std::max(box1.y, box2.y);
    x2 = std::min(box1.x + box1.width, box2.x + box2.width);
    y2 = std::min(box1.y + box1.height, box2.y + box2.height);

    intersection_area = std::max(x2 - x1, 0) * std::max(y2 - y1, 0);

    // union
    double box1_area, box2_area;
    box1_area = box1.width * box1.height;
    box2_area = box2.width * box2.height;

    union_area = box1_area + box2_area - intersection_area;

    return intersection_area / union_area;
}

/**
 * draw a bounding box
 */
inline void draw_box(cv::Mat& image, const cv::Rect& box, const cv::Scalar& color)
{
    int left, top, right, bottom;
    left = box.x;
    top = box.y;
    right = box.x + box.width;
    bottom = box.y + box.height;
    cv::rectangle(image, cv::Point(left, top), cv::Point(right, bottom), color, 2);
}

} // namespace tello_slam

#endif // TELLOSLAM_UTILITYTOOLBOX_H
