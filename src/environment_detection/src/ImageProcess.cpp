#include "ImageProcess.h"
#include <opencv2/opencv.hpp>

ImageProcessor::ImageProcessor() {}

ImageProcessor::ContourAnalysisResult ImageProcessor::process(const cv::Mat &image) {
    ContourAnalysisResult result;

    if (image.empty()) {
        return result;
    }

    cv::Mat gray_image, threshold_image;
    cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
    cv::threshold(gray_image, threshold_image, Constants::ThresholdValue, 255.0, cv::THRESH_BINARY_INV);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(threshold_image, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) {
        return result;
    }

    auto main_contour = *std::max_element(contours.begin(), contours.end(),
                                         [](const std::vector<cv::Point> &a, const std::vector<cv::Point> &b) {
                                             return cv::contourArea(a) < cv::contourArea(b);
                                         });


    cv::Mat left_part, right_part;
    left_part = threshold_image(cv::Range(Constants::LeftYStartValue, Constants::LeftYEndValue),
                                 cv::Range(Constants::LeftXStartValue, Constants::LeftXEndValue));
    right_part = threshold_image(cv::Range(Constants::RightYStartValue, Constants::RightYEndValue),
                                  cv::Range(Constants::RightXStartValue, Constants::RightXEndValue));

    int left_black_pixel_count = cv::countNonZero(left_part);
    int right_black_pixel_count = cv::countNonZero(right_part);

    result.contour_center = getContourCenter(main_contour);
    result.extent = getContourExtent(main_contour);
    result.area = cv::contourArea(main_contour);
    result.left_black_pixel_count = left_black_pixel_count;
    result.right_black_pixel_count = right_black_pixel_count;
    result.valid = true;
    result.width = image.cols;
    result.height = image.rows;
    result.middle_x = result.width / 2;
    result.middle_y = result.height / 2;
    result.mid_pixel = (int)threshold_image.at<uchar>(result.middle_y, result.middle_x);


    return result;
}

cv::Point ImageProcessor::getContourCenter(const std::vector<cv::Point> &contour) {
    cv::Moments M = cv::moments(contour);
    if (M.m00 == 0) {
        return cv::Point(0, 0);
    }
    return cv::Point(static_cast<int>(M.m10 / M.m00), static_cast<int>(M.m01 / M.m00));
}

double ImageProcessor::getContourExtent(const std::vector<cv::Point> &contour) {
    double area = cv::contourArea(contour);
    cv::Rect bounding_rect = cv::boundingRect(contour);
    double rect_area = static_cast<double>(bounding_rect.width * bounding_rect.height);
    double extent = rect_area > 0 ? (area / rect_area) : 0;
    return extent;
}