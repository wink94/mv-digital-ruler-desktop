#ifndef HOMOGENEOUS_BG_DETECTOR_H
#define HOMOGENEOUS_BG_DETECTOR_H

#include <opencv2/opencv.hpp>

class HomogeneousBgDetector
{
public:
    HomogeneousBgDetector();
    std::vector<std::vector<cv::Point>> detect_objects(cv::Mat frame);

private:
    int min_area_threshold = 5000;
    double approx_poly_dp_ratio = 0.03;
    int adaptive_threshold_block_size = 19;
    int adaptive_threshold_constant = 5;
};

#endif
