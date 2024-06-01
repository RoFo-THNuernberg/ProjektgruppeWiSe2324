#include "square_detection.hpp"

//Set in constants.hpp Header
const float sensitivity = camera_consts::sensitivity;
const float sensitivityCorner = camera_consts::sensitivityCorner;

const int minSizeContour = camera_consts::minSizeSquare;
const int maxSizeContour = camera_consts::maxSizeSquare;

void findSquares(const cv::Mat &imageCanny, cv::Mat &imageToDrawOn, std::vector<cv::Point> &corners) {
    std::vector<std::vector<cv::Point>> contours;

    //Fill out holes in canny contours
    cv::dilate(imageCanny, imageCanny, cv::Mat(), cv::Point(-1, -1));

    findContours(imageCanny, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
    
    std::vector<cv::Point> approx;

    for(size_t i = 0; i < contours.size(); i++) {
        cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true) * sensitivity, true);

        //4 Corners because of Square, Size to filter out accidently detected squares, absolut value as depending on the orientation negative Areas are possible and Convex to check if contour is actually a rectangle 
        if(approx.size() == 4 && fabs(cv::contourArea(cv::Mat(approx))) > minSizeContour && (fabs(cv::contourArea(cv::Mat(approx))) < maxSizeContour || 1) && cv::isContourConvex(cv::Mat(approx))) {
            double maxCosine = 0;

            for(int j = 2; j < 5; j++) {
                double cosine = fabs(angle(approx[j % 4], approx[j - 2], approx[j - 1]));
                maxCosine = MAX(maxCosine, cosine);
            }

            //if cos smaller than 0.3 then corner at almost 90° -> therefore squared 
            if(maxCosine < sensitivityCorner) {
                drawSquares(imageToDrawOn, approx);
                
                //In this application sorting by X axis was chosen
                sort(approx.begin(), approx.end(), sortByXAxis);

                cv::Point center(0, 0);

                center.x = (approx[3].x - approx[0].x) / 2 + approx[0].x;
                center.y = (approx[1].y - approx[0].y) / 2 + approx[0].y;

                if(center.x < corners[0].x && center.y < corners[0].y) {
                    corners[0] = center;
                } else if(center.x < corners.at(1).x && center.y > corners.at(1).y) {
                    corners.at(1) = center;
                } else if(center.x > corners.at(2).x && center.y > corners.at(2).y) {
                    corners.at(2) = center;
                }

                //"utility.hpp" function -> extra large Point
                drawPoint(imageToDrawOn, center, 5);
            }
        }
    }
}

void drawSquares(cv::Mat &image, std::vector<cv::Point> &squares) {
    cv::Point *p = &squares[0];

    int n = squares.size();

    //prevent image Bounds from beeing detected
    if(squares[0].x > 3 && squares[0].y > 3) {
        cv::polylines(image, &p, &n, 1, true, cv::Scalar(0, 255, 0), 3, cv::LINE_AA);
    }
}

double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0) {
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return ((dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10));
}

/**
 * Sort functions
**/ 
bool sortByYAxis(const cv::Point &a, const cv::Point &b) {
    return a.y < b.y;
}

bool sortByXAxis(const cv::Point &a, const cv::Point &b) {
    return a.x < b.x;
}
