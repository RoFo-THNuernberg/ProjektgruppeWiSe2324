#ifndef SQUARE_DETECTION_HPP
#define SQUARE_DETECTION_HPP

#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>

#include "utility.hpp"
#include "constants.hpp"

bool sortByXAxis(const cv::Point &a, const cv::Point &b);

bool sortByYAxis(const cv::Point &a, const cv::Point &b);

double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0);

void drawSquares(cv::Mat &image, std::vector<cv::Point> &squares);

void findSquares(const cv::Mat &imageCanny, cv::Mat &imageToDrawOn, std::vector<cv::Point> &corners);

#endif