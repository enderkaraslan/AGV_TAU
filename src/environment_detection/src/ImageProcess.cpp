#include "ImageProcess.h"

ImageProcessor::ImageProcessor() 
{
    // Constructor
}

cv::Mat ImageProcessor::preprocessImage(const cv::Mat &image) 
{
    cv::Mat gray_image, threshold_image;
    cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
    cv::threshold(gray_image, threshold_image, Constants::BINARY_THRESHOLD_VALUE, 255.0, cv::THRESH_BINARY_INV);
    return threshold_image;
}

std::vector<cv::Point> ImageProcessor::getMainContour(const std::vector<std::vector<cv::Point>> &contours) 
{
    return *std::max_element(contours.begin(), contours.end(),
                             [](const std::vector<cv::Point> &a, const std::vector<cv::Point> &b) {
                                 return cv::contourArea(a) < cv::contourArea(b);
                             });
}

cv::Mat ImageProcessor::getPart(const cv::Mat &image, int y_start, int y_end, int x_start, int x_end) 
{
    return image(cv::Range(y_start, y_end), cv::Range(x_start, x_end));
}

cv::Point ImageProcessor::getContourCenter(const std::vector<cv::Point> &contour) 
{
    cv::Moments M = cv::moments(contour);
    if (M.m00 == 0) {
        return cv::Point(0, 0);
    }
    return cv::Point(static_cast<int>(M.m10 / M.m00), static_cast<int>(M.m01 / M.m00));
}

double ImageProcessor::getContourExtent(const std::vector<cv::Point> &contour) 
{
    double area = cv::contourArea(contour);
    cv::Rect bounding_rect = cv::boundingRect(contour);
    double rect_area = static_cast<double>(bounding_rect.width * bounding_rect.height);
    return rect_area > 0 ? (area / rect_area) : 0.0;
}

ImageProcessor::ContourAnalysisResult ImageProcessor::process(const cv::Mat &image) 
{
    ContourAnalysisResult result;

    // Check for empty image
    if (image.empty()) {
        return result;
    }

    // Preprocess the image (grayscale and threshold)
    cv::Mat threshold_image = preprocessImage(image);

    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(threshold_image, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    // If no contours found, return
    if (contours.empty()) {
        return result;
    }

    // Get the main contour
    auto main_contour = getMainContour(contours);

    // Divide image into parts and count black pixels
    cv::Mat left_part = getPart(threshold_image, Constants::LEFT_Y_START, Constants::LEFT_Y_END,
                                Constants::LEFT_X_START, Constants::LEFT_X_END);
    cv::Mat right_part = getPart(threshold_image, Constants::RIGHT_Y_START, Constants::RIGHT_Y_END,
                                 Constants::RIGHT_X_START, Constants::RIGHT_X_END);

    result.left_black_pixel_count = cv::countNonZero(left_part);
    result.right_black_pixel_count = cv::countNonZero(right_part);

    // Compute additional contour properties
    result.contour_center = getContourCenter(main_contour);
    result.extent = getContourExtent(main_contour);
    result.area = cv::contourArea(main_contour);
    result.valid = true;

    // Compute image properties
    result.width = image.cols;
    result.height = image.rows;
    result.middle_x = result.width / 2;
    result.middle_y = result.height / 2;

    // Extract pixel value at the middle of the threshold image
    result.mid_pixel = static_cast<int>(threshold_image.at<uchar>(result.middle_y, result.middle_x));

    return result;
}
