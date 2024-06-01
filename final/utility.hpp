#ifndef UTILITY_HPP
#define UTILITY_HPP

#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include "control_window_params.hpp"

#include <fstream>
#include <vector>
//#include <cstring>

struct parameterDescription;

cv::Mat getBlurred(cv::Mat& original, parameterDescription &blurMode, parameterDescription &blurSize);
cv::Mat getCanny(cv::Mat& original, parameterDescription &lowerThreshold, parameterDescription &higherThreshold);

void readInData(const std::string& fileName, std::vector<std::vector<std::string>> &dataDestination);
std::vector<std::string>& seperateString(std::string toSeperate, const char *seperator);

void drawPoint(cv::Mat &image, const cv::Point &point, int sizeAdd = 0);
double getAngleRobot(const cv::Point &center1, const cv::Point &center2, const bool degree = false);

std::pair<double, double> getNormalizedPosition(const cv::Point &position, const std::pair<int, int> &size_cm, const std::pair<int, int> &size_px, const bool startBottomLeft = true);

#endif