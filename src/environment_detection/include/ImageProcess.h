#pragma once

#include <opencv2/opencv.hpp>
#include "Constants.h"


class ImageProcessor {
public:
    struct ContourAnalysisResult {
        cv::Point contour_center;
        double extent;
        double area;
        int left_black_pixel_count;
        int right_black_pixel_count;
        int mid_pixel;
        bool valid;
        int width;
        int height;
        int middle_x;
        int middle_y;
    };

    ImageProcessor();
    ContourAnalysisResult process(const cv::Mat &image);

private:
    cv::Point getContourCenter(const std::vector<cv::Point> &contour);
    double getContourExtent(const std::vector<cv::Point> &contour);
};

