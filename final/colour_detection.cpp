#include "colour_detection.hpp"

using namespace cv;
using namespace std;

cv::Mat outputColour;

//colour
// Definiere Farbintervalle für Rot, Grün, Blau und Schwarz im HSV-Farbraum
inline cv::Scalar lower_red =      cv::Scalar(160, 100, 140);
inline cv::Scalar upper_red =      cv::Scalar(180, 255, 255);

inline cv::Scalar lower_green =    cv::Scalar(40, 0, 90);
inline cv::Scalar upper_green =    cv::Scalar(100, 60, 135);

inline cv::Scalar lower_blue =     cv::Scalar(100, 150, 150);
inline cv::Scalar upper_blue =     cv::Scalar(140, 255, 255);

inline cv::Scalar lower_black =    cv::Scalar(100, 0, 0);
inline cv::Scalar upper_black =    cv::Scalar(180, 255, 140);

int colour(int x, int y)
{
    int exit;

    Mat img_hsv;
    cvtColor(outputColour, img_hsv, COLOR_BGR2HSV);

    Vec3b pixel_hsv = img_hsv.at<Vec3b>(y, x);

    if
       (pixel_hsv[0] >= lower_red[0] && pixel_hsv[0] <= upper_red[0] &&
        pixel_hsv[1] >= lower_red[1] && pixel_hsv[1] <= upper_red[1] &&
        pixel_hsv[2] >= lower_red[2] && pixel_hsv[2] <= upper_red[2]){
            // Der Pixel liegt im roten Bereich
        exit = 1;
        } 
    else if
       (pixel_hsv[0] >= lower_green[0] && pixel_hsv[0] <= upper_green[0] &&
        pixel_hsv[1] >= lower_green[1] && pixel_hsv[1] <= upper_green[1] &&
        pixel_hsv[2] >= lower_green[2] && pixel_hsv[2] <= upper_green[2]){
            // Der Pixel liegt im gruenen Bereich
            exit = 2;
        }
    else if (pixel_hsv[0] >= lower_blue[0] && pixel_hsv[0] <= upper_blue[0] &&
        pixel_hsv[1] >= lower_blue[1] && pixel_hsv[1] <= upper_blue[1] &&
        pixel_hsv[2] >= lower_blue[2] && pixel_hsv[2] <= upper_blue[2]) {
            // Der Pixel liegt im blauen Bereich
            exit = 3;
        }
    else if
       (pixel_hsv[0] >= lower_black[0] && pixel_hsv[0] <= upper_black[0] &&
        pixel_hsv[1] >= lower_black[1] && pixel_hsv[1] <= upper_black[1] &&
        pixel_hsv[2] >= lower_black[2] && pixel_hsv[2] <= upper_black[2]){
            // Der Pixel liegt im schwarzen Bereich
            exit = 0;
        }
        else 
        {
            cout << (int)pixel_hsv[0] << " " << (int)pixel_hsv[1] << " " << (int)pixel_hsv[2] << " " << endl;
            exit = -1;
        }


    return exit;
}

int rethsv(int x, int y, int hsv)
{
    Mat img_hsv;
    cvtColor(outputColour, img_hsv, COLOR_BGR2HSV);

    Vec3b pixel_hsv = img_hsv.at<Vec3b>(y, x);

    if(!(hsv < 0 || hsv > 3)){return pixel_hsv[hsv];}
    else {return (-1);}
    
}