#ifndef CIRCLE_DETECTION_HPP
#define CIRCLE_DETECTION_HPP

#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>

#include <iostream>
#include <utility>

#include "utility.hpp"
#include "constants.hpp"

void drawCircles(cv::Mat &input, const std::vector<cv::Vec3f> &circles, cv::Scalar colour = cv::Scalar(255, 0, 255));

/**
 * @brief Function to find the Circles in a given Image. Already calls drawing function 
 * 
 * @note in return vector [elem][0] x-Coordinate of center 
 * @note in return vector [elem][1] y-Coordinate of center 
 * @note in return vector [elem][2] radius of detected circle
 * 
 * @param[in] input Reference to cv::Mat matrix of Image where circle-location is desired. Needs to be grey   
 * @param[out] inputDrawOn Reference to cv::Mat matrix, where found circles will be "drawn" on using the "drawCircles" function 
 * @param[in] detectionSizes const Reference to std::pair<int, int> where the upper and lower detection thresholds for the circle-sizes are passed 
 * @param[in] colour optional parameter of cv::Scalar to specify special colour for circle-drawing. Otherwise the standard colour is picked
 * 
 * @return returns "output" float Vector of found circles. One element means one circle
*/
std::vector<cv::Vec3f> findCircles(cv::Mat &input, cv::Mat &inputDrawOn, const std::pair<int, int> &detectionSizes, cv::Scalar colour = cv::Scalar(255, 0, 255));
#endif