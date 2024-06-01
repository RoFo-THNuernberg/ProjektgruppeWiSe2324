#include "utility.hpp"

cv::Mat getCanny(cv::Mat& original, parameterDescription &lowerThreshold, parameterDescription &higherThreshold) {
    //std::cout << std::endl << "           " << lowerThreshold.getValue() << "   " << higherThreshold.getValue() << std::endl;
    cv::Mat imageCanny;
    cv::Canny(original, imageCanny, lowerThreshold.getValue(), higherThreshold.getValue()); 
    return imageCanny;
}

cv::Mat getBlurred(cv::Mat& original, parameterDescription &blurMode, parameterDescription &blurSize) {
    if(!(blurSize.getValue()%2)) return original;
    cv::Mat blurred;
    if(blurMode.getValue()) {
        cv::GaussianBlur(original, blurred, cv::Size(blurSize.getValue(), blurSize.getValue()), 0, 0);
    } else {
        cv::medianBlur(original, blurred, blurSize.getValue());
    }
    return blurred;
}

void readInData(const std::string& fileName, std::vector<std::vector<std::string>> &dataDestination) {
    std::ifstream myFile;
    myFile.open(fileName);

    std::string inputData;

    std::vector<std::string> paragraph;
    if(myFile.is_open()) {
        while(myFile) {
            if(myFile.eof()) inputData = "";
            std::getline(myFile, inputData);
            if(inputData != "") {
                paragraph.push_back(inputData);
            } else {
                dataDestination.push_back(paragraph);
                paragraph.clear();
            }
        }
    }
}

std::vector<std::string>& seperateString(std::string toSeperate, const char *seperator) {
    static std::vector<std::string> result;
    result.clear();
    char* dataAsPointer = &toSeperate[0];
    char* token = strtok(dataAsPointer, seperator);
    result.push_back(token);
    
    while(token != NULL) {
        token = strtok(NULL, seperator);
        if(token != NULL) result.push_back(token);
    }
    
    return result;
}

void drawPoint(cv::Mat &image, const cv::Point &point, int sizeAdd) {
    circle(image, point, 1, cv::Scalar(0, 255, 0), 3+sizeAdd, cv::LINE_AA);
}

double getAngleRobot(const cv::Point &centerLarge, const cv::Point &centerSmall, const bool degree) {
    double deltaX = centerSmall.x - double(centerLarge.x);
    double deltaY = centerSmall.y - double(centerLarge.y);
    
    //For normal coordinates +, but with - for "image coordinates"
    double angle = - std::atan2(deltaY, deltaX);
    
    if(degree) angle *= (180/M_PI);
    
    return angle;
}

std::pair<double, double> getNormalizedPosition(const cv::Point &position, const std::pair<int, int> &size_cm, const std::pair<int, int> &size_px, const bool startBottomLeft) {
    std::pair<double, double> normalizedPosition;

    double x = double(position.x);
    double y = double(position.y);

    x = x * (double(size_cm.first) / double(size_px.first)); 
    y = abs(startBottomLeft * size_cm.second - (y * (double(size_cm.second) / double(size_px.second))));

    //Output in m
    normalizedPosition.first = x / 100;
    normalizedPosition.second = y / 100;

    return normalizedPosition;
}