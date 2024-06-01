#include "circle_detection.hpp"

void drawCircles(cv::Mat &input, const std::vector<cv::Vec3f> &circles, cv::Scalar colour) {
    for(unsigned long i = 0; i < circles.size(); i++) //Drawing the circles
    {
        cv::Vec3i c = circles[i];
        cv::Point center = cv::Point(c[0], c[1]);//drawing the centre

        //"utility.hpp" function
        drawPoint(input, center);

        int radius = c[2];
        circle(input, center, radius, colour, 3, cv::LINE_AA);

        std::cout << "Drew Circle at " << center << " with r=" << radius << "  length: " << circles.size() << std::endl;
    }
}

std::vector<cv::Vec3f> findCircles(cv::Mat &input, cv::Mat &inputDrawOn, const std::pair<int, int> &detectionSizes, cv::Scalar colour) {
    std::vector<cv::Vec3f> circles;

    cv::HoughCircles(input, circles, cv::HOUGH_GRADIENT, 1, input.rows / 16, 100, 30, detectionSizes.first, detectionSizes.second);

    if(debug_consts::drawDetectedCircles) drawCircles(inputDrawOn, circles, colour);

    return circles;
}