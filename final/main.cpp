#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/imgproc.hpp>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <turtlesim/Pose.h>

#include <iostream>
#include <limits>

#include "control_window_params.hpp"
#include "square_detection.hpp"
#include "circle_detection.hpp"
#include "callback_functions.hpp"
#include "utility.hpp"
#include "constants.hpp"
#include "colour_detection.hpp"

using namespace cv;
using namespace std;

//Set in "constants.hpp"
const int destWidth = camera_consts::destWidth;
const int destHeight = camera_consts::destHeight;

const int inputWidth = camera_consts::inputWidth;
const int inputHeight = camera_consts::inputHeight;

//Actual Threshold Value for Adaptive Thresholding
int adaptiveThresholdLower = 9;
//Value averaging Pixels around out as "Kernel" -> Uneven
int adaptiveThresholdUpper = 15;

vector<Point> corners;

parameterController paramController;
/**     Descriptors     **/
parameterDescription thresholdMode      (0, 1, 0, "Threshold Mode (Binary)", callback_trackbar_thresholdMode);
parameterDescription blurMode           (0, 1, 0, "Blur Mode", callback_trackbar_BlurMode);
parameterDescription blurSize           (0, 7, 9, "Bluring Size (1:=3 ,2:=5 etc.)", callback_trackbar_BlurSize);
parameterDescription cannyLow           (0, 60, 12, "Canny Threshold Low", callback_trackbar_ThresholdCannyLow);
parameterDescription cannyHigh          (0, 180, 35, "Canny Threshold High", callback_trackbar_ThresholdCannyHigh);
parameterDescription displayWindowSize  (0, 50, 0, "Displaywindow Size (in %)", callback_trackbar_DisplayWindowSize);
parameterDescription thresholdLow       (0,255, 80, "Lower Threshold Value", callback_trackbar_ThresholdLow);
parameterDescription thresholdHigh      (0, 255, 255, "Higher Threshold Value", callback_trackbar_ThresholdHigh);
parameterDescription squareDetection    (0, 1, 1, "Square Detection (Calib. Mode)", callback_trackbar_squareDetection);
parameterDescription circleDetection    (0, 1, 1, "Clear Gray Circledetection Mode", callback_trackbar_circleDetection);
parameterDescription displayMode        (0, 1, 1, "Display Mode", callback_trackbar_displayMode);
parameterDescription thresholdAdaptive  (0, 1, 0, "Binary (0) or Adaptive (1) Threshold", callback_trackbar_adaptiveMode);

//Creating used "Images"
Mat image;
Mat imageGray;
Mat imageBlurred;
Mat imageCanny;
extern Mat outputColour;    //defined in colour_detection.cpp

//Optional image -> used if slider acitvated use
Mat imageBlurredGray;

//--------------------

//vector for detected circles --> function findCircles() in main
vector<Vec3f> circlesSmall;
vector<Vec3f> circlesLarge;

//array --> how many robots are active
bool rob_array_active[4] = {0, 0, 0, 0};

//current and previous positions of the robots (casex, manual setup)
int pos_array_curr[4][2] = {-1,-1, -1,-1, -1,-1, -1,-1};
int pos_array_prev[4][2] = {-1,-1, -1,-1, -1,-1, -1,-1};

//counter for switching from "detected" to "not configured"
int counter_imageloss[4] = {0,0,0,0};
const int COUNTER_IMAGELOSS = 20;

//angle of the robots [-pi,pi]
double ang_array_curr[4] = {-1, -1, -1, -1};

//constants for manual detection or casex
const int BIGBIG = 20;      //distance between the middle of two big circles
const int BIGSMALL = 20;    //distance between the middle of a small and a big circle

//variable shows assignment of small and big circle
bool assignsmallbig = false;

//if colourdetection is enabled --> set with casek
bool colourdetection = false;


//--------------------
/*--formation variables--*/


int formation_array_selection = 0;          //selection of formation-type --> selction of array in formation_selection
int formation_lead_fol[4] = {0, 0, 0, 0};   //status of robot in formation {0:not part of, 1:leader, 2:follower}

//general variables for setup, start ...
bool formation_configuration = false;       //true if formation configure is possible
bool formation_start = false;               //if formation is started
bool formation_setup = false;               //true if first values of formation are published
bool follower_config = false;               //not in this version --> comment
bool follower_ready = false;                //not in this version --> comment
bool formation_simulation = true;           //if false => formation only possible if robots are active

//goal positions of the robots in the formation, steps of each robot in the formation
int formation_goal_pos[4][2] = {-1,-1, -1,-1, -1,-1, -1,-1};
int formation_step[4] = {0,0,0,0};

//distance between goalpoint and publishing new goalpoint
const int FORMATIONBIG = 70;

//variable for percentage display
int formation_max_value = 0;

//not in this version --> comment (different concept of leader-follower)
int follower_count_main = 0;                //not in this version --> comment  
const int follower_count_max = 50;          //not in this version --> comment

//order of the follower stored in an array
int follower_order[3] = {-1, -1, -1};

//distance between leader and follower
const int DISTANCEFOL = 150;

//--------------------
/* -- variables for casex --*/

//variable, which robot is configured at the moment [-1;4]
int robot_config_casex = (-10);     //-1 setup

//images from before and after driving --> compare --> assign robot
int robot_pose_casex_saved[4][2] =  {-100,-100, -100,-100, -100,-100, -100,-100};
int robot_pose_casex_var[4][2] =    {-100,-100, -100,-100, -100,-100, -100,-100};

//variables for publishing casex positions and goal-positions
int counter_casex = 0;
const int counter_casex_max = 50;
int counter_casex_goal = (-10);

//---------------------
/* -- topics for goalpoint (ros) --> constants -- */

extern ros::Publisher goal_pub_r1;
extern ros::Publisher goal_pub_r2;
extern ros::Publisher goal_pub_r3;
extern ros::Publisher goal_pub_r4;


//--------------------
/* -- arrays for formation -- */
//--> see doku

const char formation_arr_triangle_text[] = "triangle";
const int formation_arr_triangle[5][2] =
{4, 0,
400, 200,
200, 600,
400, 600,
600, 600};

const char formation_arr_square_text[] = "square 1";
const int formation_arr_square[5][2] =
{4, 0,
400, 150,
150, 400,
400, 650,
650, 400};

const char formation_arr_square_2_text[] = "square 2";
const int formation_arr_square_2[5][2] =
{4, 0,
200, 200,
200, 600,
600, 600,
600, 200 };

const char formation_arr_heart_text[] = "heart";
const int formation_arr_heart[17][2] =
{16, 0,  
350, 200,
275, 300,
200, 400,
150, 500,
170, 600,
240, 650,
300, 635,
350, 550,
450, 550,
500, 635,
560, 650,
630, 600,
650, 500,  
600, 400,
525, 300,
450, 200 };

const char formation_arr_cross_text[] = "cross";
const int formation_arr_cross[13][2] =
{12, 0,
350, 150,
350, 350,
150, 350,
150, 450,
350, 450,
350, 650,
450, 650,
450, 450,
650, 450,
650, 350,
450, 350,
450, 150 };

const char formation_arr_circle_text[] = "circle";
const int formation_arr_circle[13][2] =
{12, 0,
350, 150,  
225, 225,  
150, 350,  
150, 450,  
225, 575,  
350, 650,  
450, 650,  
575, 575,  
650, 450,  
650, 350,  
575, 225,  
450, 150 };

//assign arrays to selection "selection_array"
const int* formation_selection[6] = {   (int*)formation_arr_triangle,
                                        (int*)formation_arr_square,
                                        (int*)formation_arr_square_2,
                                        (int*)formation_arr_heart,
                                        (int*)formation_arr_cross,
                                        (int*)formation_arr_circle };

//assign description to "selection_array"
const char* formation_selection_text[6] = { (char*)formation_arr_triangle_text,
                                            (char*)formation_arr_square_text,
                                            (char*)formation_arr_square_2_text,
                                            (char*)formation_arr_heart_text,
                                            (char*)formation_arr_cross_text,
                                            (char*)formation_arr_circle_text };

//--------------------
/* -- log -- */

//function for logs to get time data
string logtime()
{
    string time_str;
    time_t now;
    now = time(0);
    char timestamp[25];
    strncpy(timestamp, ctime(&now), 24);
    timestamp[24] = '\0';
    time_str = timestamp;

    return time_str;
}

//function for signallog
bool log_signal_active = false;

//--------------------
/* -- displayed digits in the window -- */
const int DISPLAYDIGITS = 3;






int main(int argc, char** argv) {
    ros::init(argc, argv, "RobotDetection");

    //create ros_topics for goalpoint and position
    ros::NodeHandle nodeHandle;

    ros::Publisher sim_pub = nodeHandle.advertise<turtlesim::Pose>(ros_consts::sim_topic_name, 1000);
    
    ros::Publisher pose_pub_r1 = nodeHandle.advertise<turtlesim::Pose>(ros_consts::pose_topic_name_r1, 1000);
    ros::Publisher goal_pub_r1 = nodeHandle.advertise<geometry_msgs::Point>(ros_consts::goal_topic_name_r1, 1000);
    ros::Publisher pose_pub_r2 = nodeHandle.advertise<turtlesim::Pose>(ros_consts::pose_topic_name_r2, 1000);
    ros::Publisher goal_pub_r2 = nodeHandle.advertise<geometry_msgs::Point>(ros_consts::goal_topic_name_r2, 1000);
    ros::Publisher pose_pub_r3 = nodeHandle.advertise<turtlesim::Pose>(ros_consts::pose_topic_name_r3, 1000);
    ros::Publisher goal_pub_r3 = nodeHandle.advertise<geometry_msgs::Point>(ros_consts::goal_topic_name_r3, 1000);
    ros::Publisher pose_pub_r4 = nodeHandle.advertise<turtlesim::Pose>(ros_consts::pose_topic_name_r4, 1000);
    ros::Publisher goal_pub_r4 = nodeHandle.advertise<geometry_msgs::Point>(ros_consts::goal_topic_name_r4, 1000);
    
    //name windows
    namedWindow("Control Window", WINDOW_AUTOSIZE);
    namedWindow("Normal Image", WINDOW_AUTOSIZE);
    namedWindow("warped", WINDOW_AUTOSIZE);

    paramController.addParam(&thresholdMode);
    paramController.addParam(&blurMode);
    paramController.addParam(&blurSize);
    paramController.addParam(&cannyLow);
    paramController.addParam(&cannyHigh);
    paramController.addParam(&displayWindowSize);
    paramController.addParam(&thresholdLow);
    paramController.addParam(&thresholdHigh);
    paramController.addParam(&squareDetection);
    paramController.addParam(&circleDetection);
    paramController.addParam(&displayMode);
    paramController.addParam(&thresholdAdaptive);

    paramController.createTrackbars();

    //Attach callback function for waypoint Publishing to Warped Img
    setMouseCallback("warped", callback_mouse_doubleclicked_def, &goal_pub_r1);

    //detected corners
    Point2f cornersFloat[3];
    
    //Corners the detected Corners are mapped to
    //Corners are at Top Left, Bottom Left and Bottom Right (Thats the Layout of Detection Bounds)
    Point2f destCorners[3] = {Point(0, 0), Point(0, destHeight), Point(destWidth, destHeight)};

    Mat outputWarped;
    //Warp Matrix for Aplication on the Image
    Mat warpMat;

    //Initialize "Detected Points" in Center that they can be re-arranged
    corners.push_back(Point(inputWidth/2, inputHeight/2));
    corners.push_back(Point(inputWidth/2, inputHeight/2));
    corners.push_back(Point(inputWidth/2, inputHeight/2));

    //Open Webcam
    VideoCapture cap;
    cap.open(-1);
 
    cap.set(cv::CAP_PROP_FRAME_WIDTH, inputWidth);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, inputHeight);
      
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(CAP_PROP_FPS, 30);

    cap >> image;

    //Start and End Variable for FPS Calculation
    clock_t start, end;

    //Number of Frames Processed in given Time
    int numFrames = 1;
    double msBetweenFrames, fpsLive;

    //Configure errorlog- and log-files (log, formationlog, keylog, signallog)
    ofstream flog_error;
    flog_error.open("errorlog.txt");
    if(flog_error.is_open())
    {
            cout << "Error-Log active" << endl;
            cout << "Incidents and Problems are reported to 'errorlog.txt'" << endl;

            flog_error << logtime() << " | " << "Error-Log started" << endl;
    }
    else
    {
            cout << "Error-Log inactive" << endl;
            cout << "File 'errorlog.txt' could not be configured correctly" << endl;
    }


    ofstream flog_log;
    flog_log.open("log.txt");
    if(flog_log.is_open())
    {
            cout << "Log active" << endl;
            cout << "Actions in the program are reported to 'log.txt'" << endl;
    
            flog_log << logtime() << " | " << "Log started" << endl;
    }
    else
    {
            cout << "Log inactive" << endl;
            cout << "File 'log.txt' could not be configured correctly" << endl;
            if(flog_error.is_open()){flog_error << logtime() << " | " << "'log.txt' could not be configured correctly" << endl;}
    }

    ofstream flog_formation;
    flog_formation.open("formationlog.txt");
    if(flog_formation.is_open())
    {
        cout << "Formation-Log active" << endl;
        cout << "Events of the formation are reported to 'formationlog.txt'" << endl;
        flog_formation << logtime() << " | " << "Formation-Log started" << endl;
    }
    else
    {
        cout << "Formation-Log inactive" << endl;
        cout << "File 'formationlog.txt' could not be configured correctly" << endl;
        if(flog_error.is_open()){flog_error << logtime() << " | " << "'formationlog.txt' could not be configured correctly" << endl;}
    }

    ofstream flog_keylog;
    flog_keylog.open("keylog.txt");
    if(flog_keylog.is_open())
    {
        cout << "Key-Log active" << endl;
        cout << "Any keys that have been pressed are reported to 'keylog.txt'" << endl;
        flog_keylog << logtime() << " | " << "Key-Log started" << endl;
    }
    else
    {
        cout << "Key-Log inactive" << endl;
        cout << "File 'keylog.txt' could not be configured correctly" << endl;
        if(flog_error.is_open()){flog_error << logtime() << " | " << "'keylog.txt' could not be configured correctly" << endl;}
    }

    ofstream flog_signal;
    flog_signal.open("signallog.txt");
    if(flog_signal.is_open())
    {
            cout << "Signal-Log active" << endl;
            cout << "goalpoints and current positions are published to 'signallog.txt'" << endl;

            flog_signal << logtime() << " | " << "Signal-Log started" << endl;
    }
    else
    {
            cout << "Signal-Log inactive" << endl;
            cout << "File 'signallog.txt' could not be configured correctly" << endl;
    }

    if(debug_consts::displayBuildInformations) cerr << getBuildInformation();
    


    //loop, where the program is running
    while(ros::ok()) 
    {
        //Start Clock for FPS Counter
        start = clock();

        //Putting current Camera Image from Camerastream into Image Mat
        cap >> image;

        //Resising Image Mat
        resize(image, image, cv::Size(), (100-displayWindowSize.getValue())/100.0, (100-displayWindowSize.getValue())/100.0);

        //Processing resized Image i.e. turing it gray etc
        cvtColor(image, imageGray, COLOR_BGR2GRAY);
        if(thresholdMode.getValue() && !thresholdAdaptive.getValue()) threshold(imageGray, imageGray, thresholdLow.getValue(), thresholdHigh.getValue(), THRESH_BINARY);
        if(thresholdMode.getValue() && thresholdAdaptive.getValue()) adaptiveThreshold(imageGray, imageGray, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, adaptiveThresholdUpper, adaptiveThresholdLower);

        imageBlurred = getBlurred(imageGray, blurMode, blurSize);
        imageCanny = getCanny(imageBlurred, cannyLow, cannyHigh);

        //Only Blur Gray Image if really needed
        if(!circleDetection.getValue()) imageBlurredGray = getBlurred(imageGray, blurMode, blurSize);

        //When Slider for Calibration Mode is activated Programm tries to find Corners -> Saving them in "corners"
        if(squareDetection.getValue()) findSquares(imageCanny, image, corners);

        //COLOUR changed by Scalar -> Order is B-G-R
        cv::putText(image, "FPS: " + to_string(fpsLive), {50, image.rows-50}, FONT_HERSHEY_COMPLEX, 1.5, Scalar(153, 153, 0), 2);

        //Setting "cornersFloat" to their equivalent in "corners" -> implicit cast to float
        cornersFloat[0] = corners.at(0);
        cornersFloat[1] = corners.at(1);
        cornersFloat[2] = corners.at(2);
            
        warpMat = getAffineTransform(cornersFloat, destCorners);
        //Warping either blurred Image or normal image
        if(circleDetection.getValue()) 
        {
            warpAffine(imageGray, outputWarped, warpMat, Size(destWidth, destHeight));
            warpAffine(image, outputColour, warpMat, Size(destWidth, destHeight));
        } 
        else 
        {
            warpAffine(imageBlurredGray, outputWarped, warpMat, Size(destWidth, destHeight));
        }

        if(!squareDetection.getValue()) 
        {
            circlesSmall = findCircles(outputWarped, outputWarped, camera_consts::innerSize, cv::Scalar(120, 0, 120));
            circlesLarge = findCircles(outputWarped, outputWarped, camera_consts::outerSize, cv::Scalar(0, 130, 0));

            //if a big circle (robot) is detected 
            if(circlesLarge.size() != 0) 
            {


                /* -- colour-detection -- */

                if(colourdetection == true)
                {
                    for(int circlarge = 0; circlarge < (int)circlesLarge.size(); circlarge++)
                    {
                        for(int circsmall = 0; circsmall < (int)circlesSmall.size(); circsmall++)
                        {
                            if(abs(circlesLarge[circlarge][0] - circlesSmall[circsmall][0]) < BIGSMALL && abs(circlesLarge[circlarge][1] - circlesSmall[circsmall][1]) < BIGSMALL)
                                {
                                    //get colour of small circle --> colour_detection.hpp/cpp
                                    int robot_colour = colour(circlesSmall[circsmall][0], circlesSmall[circsmall][1]);

                                    if(robot_colour != (-1))
                                    {
                                        if(rob_array_active[robot_colour] == false)
                                        {
                                            rob_array_active[robot_colour] = true;

                                            if(flog_log.is_open()){flog_log << logtime() << " | " << "robot_" << robot_colour+1 << " set active" << endl;}
                                        }
                                        //assign circles large to pos_array_curr
                                        pos_array_curr[robot_colour][0] = circlesLarge[circlarge][0];
                                        pos_array_curr[robot_colour][1] = circlesLarge[circlarge][1];
                                        circlesLarge[circlarge][0] = -100;
                                        circlesLarge[circlarge][1] = -100;
                                        
                                        //get the angle of the robot
                                        ang_array_curr[robot_colour] = getAngleRobot(cv::Point(pos_array_curr[robot_colour][0], pos_array_curr[robot_colour][1]),
                                                        cv::Point(circlesSmall[circsmall][0], circlesSmall[circsmall][1]), false);
                                        circlesSmall[circsmall][0] = -100;
                                        circlesSmall[circsmall][1] = -100;

                                        break;
                                    }
                                    else
                                    {
                                        //robot not detected --> problem: colour
                                        if(flog_error.is_open())
                                        {
                                            int x = circlesSmall[circsmall][0], y = circlesSmall[circsmall][1];
                                            flog_error << logtime() << " | " << "colour not detected, wrong HSV-value: ";
                                            for(int i = 0; i<=2; i++){flog_error << rethsv(x, y, i) << " ";}
                                            flog_error << endl;
                                        }
                                    }
                                    
                                }
                        }
                    }
                }

                /* -- colour-detection not configured --> manual setup/casex -- */
                else if(colourdetection == false)
                {
                    
                    //compare prev circles with current circles
                    for(int robotcount = 0; robotcount < 4; robotcount++)
                    {
                        if(rob_array_active[robotcount] == true)
                        {
                            for(int circlarge = 0; circlarge < (int)circlesLarge.size(); circlarge++)
                            {
                                //assign foundCircles(large) to array prev circles -->  array curr circles
                                if(abs(pos_array_prev[robotcount][0] - circlesLarge[circlarge][0]) < BIGBIG && abs(pos_array_prev[robotcount][1] - circlesLarge[circlarge][1]) < BIGBIG)
                                {
                                    pos_array_curr[robotcount][0] = circlesLarge[circlarge][0];
                                    pos_array_curr[robotcount][1] = circlesLarge[circlarge][1];
                                    circlesLarge[circlarge][0] = -100;
                                    circlesLarge[circlarge][1] = -100;
                                    break;
                                }
                            }
                        }
                        
                        
                    }
                
                    //assign small circle to big circle --> angle
                    for(int robotcount = 0; robotcount < 4; robotcount++)
                    {
                        if(rob_array_active[robotcount] == true)
                        {
                            assignsmallbig = false;
                            for(int circsmall = 0; circsmall < (int)circlesSmall.size(); circsmall++)
                            {
                                //assign foundCircles(small) to array curr --> angle
                                if(abs(pos_array_curr[robotcount][0] - circlesSmall[circsmall][0]) < BIGSMALL && abs(pos_array_curr[robotcount][1] - circlesSmall[circsmall][1]) < BIGSMALL)
                                {
                                    ang_array_curr[robotcount] = getAngleRobot(cv::Point(pos_array_curr[robotcount][0], pos_array_curr[robotcount][1]),
                                                    cv::Point(circlesSmall[circsmall][0], circlesSmall[circsmall][1]), false);
                                    circlesSmall[circsmall][0] = -100;
                                    circlesSmall[circsmall][1] = -100;

                                    assignsmallbig = true;
                                    break;
                                }
                            }
                            
                            //small circle could not have been assigned to big circle
                            if(assignsmallbig == false)
                            {
                                if(flog_error.is_open()){flog_error << logtime() << " | " << "small circle was not assigned to big circle (" << robotcount+1 << ")" << endl;}
                            }
                        }
                    }
                }
                
            }

            /* -- publishing pose of the robots in ros-topics -- */
            for(int robotcount = 0; robotcount < 4; robotcount++)
                {
                    //if robot is active
                    if(rob_array_active[robotcount] == true && pos_array_curr[robotcount][0]  != (-1) && pos_array_curr[robotcount][1] != (-1))
                    {
                            turtlesim::Pose pose_msg;
                            //Getting Position in m
                            pair<double, double> normalizedPosition = getNormalizedPosition
                            (cv::Point(pos_array_curr[robotcount][0], pos_array_curr[robotcount][1]),
                                    pair<int, int>(220, 220), pair<int, int>(camera_consts::destWidth, camera_consts::destHeight));                        
                            pose_msg.x = normalizedPosition.first;
                            pose_msg.y = normalizedPosition.second;
                            pose_msg.theta = ang_array_curr[robotcount];
            
                            if(robotcount == 0)      {pose_pub_r1.publish(pose_msg);}
                            else if(robotcount == 1) {pose_pub_r2.publish(pose_msg);}
                            else if(robotcount == 2) {pose_pub_r3.publish(pose_msg);}
                            else if(robotcount == 3) {pose_pub_r4.publish(pose_msg);}
            
            
                            if(ros_consts::sim) sim_pub.publish(pose_msg);
            
                            if(debug_consts::printPublishedValues) cout << "robot_"<< robotcount+1 << ": " << setprecision(DISPLAYDIGITS) << pose_msg.x << "|" << pose_msg.y << "    " << pose_msg.theta << endl;
                            if(flog_signal.is_open() && log_signal_active == true && formation_start == true){flog_signal << logtime() << " | " << "robot_" << robotcount+1 << ": " << pose_msg.x << " " << pose_msg.y << "\t (current position)" << endl;}


                        //configuration for casex
                        if(robot_config_casex-1 >= 0 && robot_config_casex-1 <= 3 && counter_casex_goal == 0) 
                        {
                            geometry_msgs::Point goal_msg;
                            std::pair<double, double> normalizedPos = getNormalizedPosition(cv::Point(pos_array_curr[robot_config_casex-1][0], pos_array_curr[robot_config_casex-1][1]), std::pair<int, int>(220, 220), std::pair<int, int>(800, 800));
                            goal_msg.x = normalizedPos.first;
                            goal_msg.y = normalizedPos.second;
                            goal_msg.z = 0;

                            if      (robot_config_casex-1 == 0)     {goal_pub_r1.publish(goal_msg);}
                            else if (robot_config_casex-1 == 1)     {goal_pub_r2.publish(goal_msg);}
                            else if (robot_config_casex-1 == 2)     {goal_pub_r3.publish(goal_msg);}
                            else if (robot_config_casex-1 == 3)     {goal_pub_r4.publish(goal_msg);}

                            counter_casex_goal = (-10);
                        }
                    }
                    else if(rob_array_active[robotcount] == true && pos_array_curr[robotcount][0] == (-1))
                    {
                        cout << "robot_"<< robotcount+1 << ": not detected" << endl;
                    }
                    else if(rob_array_active[robotcount] == false)
                    {
                        cout << "robot_"<< robotcount+1 << ": not configured" << endl;
                    }
                }
                cout << "-------------------------------------" << endl;       
        }


        /* -- CASEX -- */
        if(robot_config_casex >= (-1))
        {
            //setup counter and robot_configuration
            if(robot_config_casex == (-1))
            {
                counter_casex = 0;
                robot_config_casex = 0;
            }
            //configuration of the robots [0;3] bzw "[1;4]"
            else if(robot_config_casex >= 0 && robot_config_casex <= 3)
            {
                //publish pose (modulo10 --> without too fast)
                if(counter_casex %10 == 0 || counter_casex == 1)
                {
                    //publish Pose-message (0,0)
                    turtlesim::Pose pose_msg;
                    pair<double, double> normalizedPosition = getNormalizedPosition
                    (cv::Point(0, 0), pair<int, int>(220, 220), pair<int, int>(camera_consts::destWidth, camera_consts::destHeight));                        
                    pose_msg.x = normalizedPosition.first;
                    pose_msg.y = normalizedPosition.second;
                    pose_msg.theta = 0;
            
                    if      (robot_config_casex == 0) {pose_pub_r1.publish(pose_msg);}
                    else if (robot_config_casex == 1) {pose_pub_r2.publish(pose_msg);}
                    else if (robot_config_casex == 2) {pose_pub_r3.publish(pose_msg);}
                    else if (robot_config_casex == 3) {pose_pub_r4.publish(pose_msg);}
                }
                

                //taking positions of the circles in the beginning and publishing "fiction" goalpoint
                if(counter_casex == 1)
                {
                    //Stocktaking with ..._saved
                    for(int index = 0; index < 4; index++)
                    {
                        robot_pose_casex_saved[index][0] = (-100);
                        robot_pose_casex_saved[index][1] = (-100);
                    }

                    int index = 0;
                    for(int circlarge = 0; circlarge < (int)circlesLarge.size(); circlarge++)
                    {
                        for(int circsmall = 0; circsmall < (int)circlesSmall.size(); circsmall++)
                        {
                            if(abs(circlesLarge[circlarge][0] - circlesSmall[circsmall][0]) < BIGSMALL && abs(circlesLarge[circlarge][1] - circlesSmall[circsmall][1]) < BIGSMALL)
                            {
                                robot_pose_casex_saved[index][0] = (int)circlesLarge[circlarge][0];
                                robot_pose_casex_saved[index][1] = (int)circlesLarge[circlarge][1];
                                index++;
                            }
                        }
                    }

                    //output of robot_pose_casex_saved in log.txt --> debug
                    //for(int i = 0; i < 4; i++){flog_log << "robot_pose_casex_saved " << i << ": " << robot_pose_casex_saved[i][0] << " " << robot_pose_casex_saved[i][1] << endl;}
                    
                    //publish one time goal_message: (200,0)
                    geometry_msgs::Point goal_msg;
                    std::pair<double, double> normalizedPos = getNormalizedPosition(cv::Point(200, 0), std::pair<int, int>(220, 220), std::pair<int, int>(800, 800));
                    goal_msg.x = normalizedPos.first;
                    goal_msg.y = normalizedPos.second;
                    goal_msg.z = 0;

                    if      (robot_config_casex == 0)     {goal_pub_r1.publish(goal_msg);}
                    else if (robot_config_casex == 1)     {goal_pub_r2.publish(goal_msg);}
                    else if (robot_config_casex == 2)     {goal_pub_r3.publish(goal_msg);}
                    else if (robot_config_casex == 3)     {goal_pub_r4.publish(goal_msg);}
                }
                
                //output percent
                cout << "casex-configuration" << endl;
                cout << "| ";
                for(int i = 0; i < robot_config_casex; i++)
                {
                    cout << "xxxxxxxxxxxxx";
                }
                for(int i = robot_config_casex; i <= 3; i++)
                {
                    cout << "             ";
                }
                cout << " | " << (float)(robot_config_casex)/(float)4*100 << " %" << endl;

                //output percent
                cout << "| ";
                for(int i = 0; i <= counter_casex; i++)
                {
                    cout << "x";
                }
                for(int i = counter_casex; i < counter_casex_max; i++)
                {
                    cout << " ";
                }
                cout << "  | " << "robot_" << robot_config_casex+1 << " " << (float)counter_casex/(float)counter_casex_max*100 << " %" << endl;
                

                //if the program runs through counter_case_max times --> check for a robot who has moved
                if(counter_casex >= counter_casex_max)
                {
                    //publish pose (200,0) to robot to stop
                    turtlesim::Pose pose_msg;
                    pair<double, double> normalizedPosition = getNormalizedPosition
                    (cv::Point(200, 0), pair<int, int>(220, 220), pair<int, int>(camera_consts::destWidth, camera_consts::destHeight));                        
                    pose_msg.x = normalizedPosition.first;
                    pose_msg.y = normalizedPosition.second;
                    pose_msg.theta = 0;
            
                    if      (robot_config_casex == 0) {pose_pub_r1.publish(pose_msg);}
                    else if (robot_config_casex == 1) {pose_pub_r2.publish(pose_msg);}
                    else if (robot_config_casex == 2) {pose_pub_r3.publish(pose_msg);}
                    else if (robot_config_casex == 3) {pose_pub_r4.publish(pose_msg);}

                    //stocktaking in ..._var
                    for(int index = 0; index < 4; index++)
                    {
                        robot_pose_casex_var[index][0] = (-100);
                        robot_pose_casex_var[index][1] = (-100);
                    }

                    int index = 0;
                    for(int circlarge = 0; circlarge < (int)circlesLarge.size(); circlarge++)
                    {
                        for(int circsmall = 0; circsmall < (int)circlesSmall.size(); circsmall++)
                        {
                            if(abs(circlesLarge[circlarge][0] - circlesSmall[circsmall][0]) < BIGSMALL && abs(circlesLarge[circlarge][1] - circlesSmall[circsmall][1]) < BIGSMALL)
                            {
                                robot_pose_casex_var[index][0] = (int)circlesLarge[circlarge][0];
                                robot_pose_casex_var[index][1] = (int)circlesLarge[circlarge][1];
                                index++;
                            }
                        }
                    }
                    
                    //output of robot_pose_casex_var in log.txt --> before manipulation --> debug
                    //for(int i = 0; i < 4; i++){flog_log << "robot_pose_casex_var_NO.1 " << i << " :" << robot_pose_casex_var[i][0] << " " << robot_pose_casex_var[i][1] << endl;}
                    
                    //manipulate the values of robot_pose_casex_var
                    //if the circle is in the radar --> write (-100) --> circle != (-100) --> robot was moved
                    for(int indexvar = 0; indexvar < 4; indexvar++)
                    {
                        for(int indexsaved = 0; (indexsaved < 4) && (robot_pose_casex_var[indexvar][0] > 0); indexsaved++)
                        {
                            if(robot_pose_casex_saved[indexsaved][0] != -100)
                            {
                                //distance in log.txt --> debug
                                //flog_log << "distance: " << sqrt(pow((robot_pose_casex_var[indexvar][0] - robot_pose_casex_saved[indexsaved][0]), 2) + pow((robot_pose_casex_var[indexvar][1] - robot_pose_casex_saved[indexsaved][1]), 2)) << " indexvar:" << indexvar << " indexsaved: " << indexsaved << endl;
                                
                                //if no difference between saved circles(robot_pose_casex_var bzw. robot_pose_casex_saved) --> delete in robot
                                if(sqrt(pow((robot_pose_casex_var[indexvar][0] - robot_pose_casex_saved[indexsaved][0]), 2) + pow((robot_pose_casex_var[indexvar][1] - robot_pose_casex_saved[indexsaved][1]), 2)) < 15)
                                {
                                    robot_pose_casex_var[indexvar][0] = (-100);
                                    robot_pose_casex_var[indexvar][1] = (-100);
                                    if(flog_log.is_open()){flog_log << logtime() << " | " << "robot_" << robot_config_casex+1 << " not moved (execution case x)" << endl;}
                                }
                            }
                            
                        }
                    }

                    //output in "log.h" --> which robot was detected
                    for(int index = 0; index < 4; index++)
                    {
                        if(robot_pose_casex_var[index][0] != (-100))
                        {
                            if(flog_log.is_open()){flog_log << logtime() << " | " << "robot_" << robot_config_casex+1 << " detected (execution case x)" << endl;}
                        }
                    }

                    //output of robot_pose_casex_var in log.txt --> before manipulation --> debug
                    //for(int i = 0; i < 4; i++){flog_log << "robot_pose_casex_var" << i << " :" << robot_pose_casex_var[i][0] << " " << robot_pose_casex_var[i][1] << endl;}


                    int testsuccess = 0;
                    //circle != (-100) --> robot was moved --> assign to robot_x
                    for(int index = 0; index < 3; index++)
                    {
                        if(robot_pose_casex_var[index][0] > 0)
                        {
                            rob_array_active[robot_config_casex]  = true;
                            pos_array_curr[robot_config_casex][0] = robot_pose_casex_var[index][0];                            
                            pos_array_curr[robot_config_casex][1] = robot_pose_casex_var[index][1];
                            pos_array_prev[robot_config_casex][0] = robot_pose_casex_var[index][0];                            
                            pos_array_prev[robot_config_casex][1] = robot_pose_casex_var[index][1];
                            counter_imageloss[robot_config_casex] = 0;

                            if(flog_log.is_open()){flog_log << logtime() << " | " << "robot_" << robot_config_casex+1 << " set active (execution case x)" << endl;}
                            cout << "robot_" << robot_config_casex+1 << " set active (execution case x)" << endl;

                            testsuccess++;
                        }
                    }
                    //tracking the position from now on --> if(colourdetection == false) ...

                    if(testsuccess == 0)        {if(flog_log.is_open()){flog_log << logtime() << " | " << "no robot_" << robot_config_casex+1 << " detected (execution case x)" << endl;}}
                    else if(testsuccess == 1)   {if(flog_log.is_open()){flog_log << logtime() << " | " << "robot_" << robot_config_casex+1 << " configured successfully (execution case x)" << endl;}}
                    else                        
                    {
                        if(flog_log.is_open()){flog_log << logtime() << " | " << "ERROR: configuration of robot_" << robot_config_casex+1 << " (multiple robots detected, execution casex)" << endl;}
                        if(flog_error.is_open()){flog_error << logtime() << " | " << "ERROR: configuration of robot_" << robot_config_casex+1 << " (multiple robots detected, execution casex)" << endl;}
                    }

                    //prepare for the next robot
                    counter_casex = 0;
                    robot_config_casex++;
                    if(flog_log.is_open()){flog_log << logtime() << " | " << "next robot config (execution case x)" << endl;}
                    if(flog_log.is_open()){flog_log << "--------------------------------------------------------------------------" << endl;}
                    
                    counter_casex_goal = 0;
                }
            }
            else if(robot_config_casex == 4)
            {
                robot_config_casex = (-10);
                if(flog_log.is_open()){flog_log << logtime() << " | " << "configuration closed (execution case x)" << endl;}
                if(flog_log.is_open()){flog_log << "--------------------------------------------------------------------------" << endl;}
            }
            
            counter_casex++;
        }


        /* -- statistics for formation -- */
        int count_fol = 0, count_lead = 0;
        for(int robotcount = 0; robotcount < 4; robotcount++)
        {
                switch(formation_lead_fol[robotcount])
            {
                case 0:     break;
                case 1:     count_lead++;
                            break;
                case 2:     count_fol++;
                            break;
                default:    break;
            }
        }
        int count_formation = count_lead + count_fol;

        if(count_fol > 0 && formation_configuration == true)
        {
            int temp = 0;
            for(int robotcount = 0; robotcount < 4; robotcount++)
            {
                if(formation_lead_fol[robotcount] == 2){follower_order[temp++] = robotcount;}
            }
            while(temp < 3)
            {
                follower_order[temp++] = (-1);
            }
        }


        /* -- formation realisation -- */
        if(formation_configuration == true && formation_start == true)
        {
            //no control with mouse-doubleclick
            setMouseCallback("warped", callback_mouse_doubleclicked_def, &goal_pub_r1);

            //setup of formation
            if(formation_setup == false)
            {
                if (count_lead > 0  && count_fol >= 0)
                {
                    
                    int step = formation_selection[formation_array_selection][0*0+0]/count_formation;
                    int count_distribution = 0;

                    //steps are splitted up for the robots
                    for(int robotcount = 0; robotcount < 4; robotcount++)
                    {
                        if(formation_lead_fol[robotcount] == 1)
                        {
                            formation_goal_pos[robotcount][0] = formation_selection[formation_array_selection][count_distribution*step*2+2+0];
                            formation_goal_pos[robotcount][1] = formation_selection[formation_array_selection][count_distribution*step*2+2+1];
                            formation_step[robotcount] = (count_distribution*step*2+2)/2;
                            count_distribution++;
                        }
                        else if(formation_lead_fol[robotcount] == 0) {}
                        else if(formation_lead_fol[robotcount] == 2) {}
                    }

                    //distribution completed --> assign goalpoints to leaders
                    if(count_distribution == count_lead)
                    {
                        cout << "Distribution successful" << endl;
                        for(int robotcount = 0; robotcount < 4; robotcount++)
                        {
                            geometry_msgs::Point goal_msg;
                            std::pair<double, double> normalizedPos = getNormalizedPosition(cv::Point(formation_goal_pos[robotcount][0], formation_goal_pos[robotcount][1]), std::pair<int, int>(220, 220), std::pair<int, int>(800, 800));
                            goal_msg.x = normalizedPos.first;
                            goal_msg.y = normalizedPos.second;
                            goal_msg.z = 0;

                            if      (robotcount == 0 && formation_lead_fol[0] == 1)     {goal_pub_r1.publish(goal_msg);}
                            else if (robotcount == 1 && formation_lead_fol[1] == 1)     {goal_pub_r2.publish(goal_msg);}
                            else if (robotcount == 2 && formation_lead_fol[2] == 1)     {goal_pub_r3.publish(goal_msg);}
                            else if (robotcount == 3 && formation_lead_fol[3] == 1)     {goal_pub_r4.publish(goal_msg);}                    
                                
                            if(formation_lead_fol[robotcount] == 1){cout << "Published new position (" << goal_msg.x << "," << goal_msg.y << ") for robot_" << robotcount+1 << endl;}
                        
                            if(flog_signal.is_open() && log_signal_active == true && formation_lead_fol[robotcount] != 0){flog_signal << "-------------------------------------------------------------------" << endl;}
                            if(flog_signal.is_open() && log_signal_active == true && formation_lead_fol[robotcount] != 0){flog_signal << logtime() << " | " << "robot_" << robotcount+1 << ": " << normalizedPos.first << " " << normalizedPos.second << "\t (goalpoint)" << endl;}
            
                        }
                        if(flog_formation.is_open()){flog_formation << logtime() << " | " << "Leader-Formation-Setup completed" << endl;}
                        formation_setup = true;
                      

                        //assign goalpoints to follower depending on the leader
                        if(count_fol > 0)
                            {
                                //statistics who is leader -->leader_number
                                int leader_number = (-1);
                                for(int robotcount = 0; robotcount < 4; robotcount++)
                                {
                                    if(formation_lead_fol[robotcount] == 1)
                                    {
                                        leader_number = robotcount;
                                    }
                                }

                                int index = 0;
                                for(index = 0; index < 3 && follower_order[index] != (-1); index++)
                                {
                                    geometry_msgs::Point goal_msg;
                                    std::pair<double, double> normalizedPos;
                                    if(index == 0)
                                    {
                                        normalizedPos = getNormalizedPosition(cv::Point(formation_goal_pos[leader_number][0]+DISTANCEFOL,formation_goal_pos[leader_number][1]), std::pair<int, int>(220, 220), std::pair<int, int>(800, 800));
                                        formation_goal_pos[follower_order[index]][0] = formation_goal_pos[leader_number][0]+DISTANCEFOL;
                                        formation_goal_pos[follower_order[index]][1] = formation_goal_pos[leader_number][1];
                                        if(flog_formation.is_open()){flog_formation << logtime() << " | " << "Follower No.1 completed" << endl;}
                                    }
                                    else if(index == 1)
                                    {
                                        normalizedPos = getNormalizedPosition(cv::Point(formation_goal_pos[leader_number][0]-DISTANCEFOL,formation_goal_pos[leader_number][1]), std::pair<int, int>(220, 220), std::pair<int, int>(800, 800));
                                        formation_goal_pos[follower_order[index]][0] = formation_goal_pos[leader_number][0]-DISTANCEFOL;
                                        formation_goal_pos[follower_order[index]][1] = formation_goal_pos[leader_number][1];
                                        if(flog_formation.is_open()){flog_formation << logtime() << " | " << "Follower No.2 completed" << endl;}
                                    }
                                    else if(index == 2)
                                    {
                                        normalizedPos = getNormalizedPosition(cv::Point(formation_goal_pos[leader_number][0]+DISTANCEFOL,formation_goal_pos[leader_number][1]+DISTANCEFOL), std::pair<int, int>(220, 220), std::pair<int, int>(800, 800));
                                        formation_goal_pos[follower_order[index]][0] = formation_goal_pos[leader_number][0]+DISTANCEFOL;
                                        formation_goal_pos[follower_order[index]][1] = formation_goal_pos[leader_number][1]-DISTANCEFOL;
                                        if(flog_formation.is_open()){flog_formation << logtime() << " | " << "Follower No.3 completed" << endl;}
                                    }
                                                
                                    goal_msg.x = normalizedPos.first;
                                    goal_msg.y = normalizedPos.second;
                                    goal_msg.z = 0;

                                    if      (follower_order[index] == 0)     {goal_pub_r1.publish(goal_msg);}
                                    else if (follower_order[index] == 1)     {goal_pub_r2.publish(goal_msg);}
                                    else if (follower_order[index] == 2)     {goal_pub_r3.publish(goal_msg);}
                                    else if (follower_order[index] == 3)     {goal_pub_r4.publish(goal_msg);}

                                    if(flog_signal.is_open() && log_signal_active == true && formation_lead_fol[follower_order[index]] != 0){flog_signal << "-------------------------------------------------------------------" << endl;}
                                    if(flog_signal.is_open() && log_signal_active == true && formation_lead_fol[follower_order[index]] != 0){flog_signal << logtime() << " | " << "robot_" << follower_order[index]+1 << ": " << normalizedPos.first << " " << normalizedPos.second << "\t (goalpoint)" << endl;}
            
                                }
                            }
                    }

                    //getting statistics for percentage output
                    formation_max_value = 0;
                    //Get starting point for displayed value % --> max value between curr and goalpos
                    for(int robotcount = 0; robotcount < 4; robotcount++)
                    {
                        if(formation_lead_fol[robotcount] !=0)
                        {
                            if(formation_max_value == 0){formation_max_value = sqrt(pow((formation_goal_pos[robotcount][0] - pos_array_curr[robotcount][0]),2) + pow((formation_goal_pos[robotcount][1] - pos_array_curr[robotcount][1]),2));}
                            else if (formation_max_value >= sqrt(pow((formation_goal_pos[robotcount][0] - pos_array_curr[robotcount][0]),2) + pow((formation_goal_pos[robotcount][1] - pos_array_curr[robotcount][1]),2))) {formation_max_value = sqrt(pow((formation_goal_pos[robotcount][0] - pos_array_curr[robotcount][0]),2) + pow((formation_goal_pos[robotcount][1] - pos_array_curr[robotcount][1]),2));}
                    
                        }
                    }
                }
            
            
            }
            else if(formation_setup == true)
            {
                cout << "Setup(true) formation " << formation_selection_text[formation_array_selection] << endl;
                
                
                //statistics of the distance between robot and goalpoint

                int distance_percent = 0;
                //Get current distance for robots % --> max value between curr and goalpos
                for(int robotcount = 0; robotcount < 4; robotcount++)
                {
                    if(formation_lead_fol[robotcount] !=0)
                    {
                        if(distance_percent == 0){distance_percent = sqrt(pow((formation_goal_pos[robotcount][0] - pos_array_curr[robotcount][0]),2) + pow((formation_goal_pos[robotcount][1] - pos_array_curr[robotcount][1]),2));}
                        else if (distance_percent >= sqrt(pow((formation_goal_pos[robotcount][0] - pos_array_curr[robotcount][0]),2) + pow((formation_goal_pos[robotcount][1] - pos_array_curr[robotcount][1]),2))) {distance_percent = sqrt(pow((formation_goal_pos[robotcount][0] - pos_array_curr[robotcount][0]),2) + pow((formation_goal_pos[robotcount][1] - pos_array_curr[robotcount][1]),2));}
                    }
                }
                
                
                float percent = 1 - (float)(distance_percent-FORMATIONBIG)/(float)(formation_max_value-FORMATIONBIG);
                percent *= 100;
                if(percent < 0){percent = 0;}
                cout << "Goal reached: " << percent << " % ";

                cout << "| ";
                for(int sum = 0; sum < percent; sum+=2.5){cout << "x";}
                for(int sum = percent; sum < 100; sum+=2.5){cout << " ";}
                cout << " |" << endl;
                
                int formation_reach_leader = 0, formation_reach_fol = 0;
                for(int robotcount = 0; robotcount < 4; robotcount++)
                {
                    if(formation_lead_fol[robotcount] != 0)
                    {
                        cout << "robot_" << robotcount+1 << ": " << sqrt(pow((formation_goal_pos[robotcount][0] - pos_array_curr[robotcount][0]),2) + pow((formation_goal_pos[robotcount][1] - pos_array_curr[robotcount][1]),2)) << "/" << FORMATIONBIG << endl;
                    }
                    
                    
                    
                    //Check, if goalposition reached
                    if(sqrt(pow((formation_goal_pos[robotcount][0] - pos_array_curr[robotcount][0]),2) + pow((formation_goal_pos[robotcount][1] - pos_array_curr[robotcount][1]),2)) < FORMATIONBIG)
                    {
                        if(formation_lead_fol[robotcount] == 1){formation_reach_leader++;}
                        if(formation_lead_fol[robotcount] == 2){formation_reach_fol++;}
                    }
                }
                if(count_lead > 0){cout << "formation reached(leader):   " << formation_reach_leader << "  count lead: " << count_lead << endl;}
                if(count_fol > 0){cout << "formation reached(follower): " << formation_reach_fol << "  count fol:  " << count_fol << endl << "-------" << endl;}
                
                //Every leader reached its position
                if(formation_reach_leader == count_lead && formation_reach_fol == count_fol)
                {
                    for(int robotcount = 0; robotcount < 4; robotcount++)
                    {
                        if(formation_lead_fol[robotcount] == 1)
                        {
                            if(formation_step[robotcount] == formation_selection[formation_array_selection][0]) {formation_step[robotcount] = 0;}
                            formation_goal_pos[robotcount][0] = formation_selection[formation_array_selection][formation_step[robotcount]*2+2+0];
                            formation_goal_pos[robotcount][1] = formation_selection[formation_array_selection][formation_step[robotcount]*2+2+1];
                            formation_step[robotcount] = formation_step[robotcount] +1;
                            cout << "change destination step: " << formation_step[robotcount] << endl;
                            cout << "new values: " << formation_goal_pos[robotcount][0] << " " << formation_goal_pos[robotcount][1] << endl;
                        }
                    }

                    for(int robotcount = 0; robotcount < 4; robotcount++)
                    {
                        geometry_msgs::Point goal_msg;
                        std::pair<double, double> normalizedPos = getNormalizedPosition(cv::Point(formation_goal_pos[robotcount][0], formation_goal_pos[robotcount][1]), std::pair<int, int>(220, 220), std::pair<int, int>(800, 800));
                        goal_msg.x = normalizedPos.first;
                        goal_msg.y = normalizedPos.second;
                        goal_msg.z = 0;

                        if      (robotcount == 0 && formation_lead_fol[0] == 1)     {goal_pub_r1.publish(goal_msg);}
                        else if (robotcount == 1 && formation_lead_fol[1] == 1)     {goal_pub_r2.publish(goal_msg);}
                        else if (robotcount == 2 && formation_lead_fol[2] == 1)     {goal_pub_r3.publish(goal_msg);}
                        else if (robotcount == 3 && formation_lead_fol[3] == 1)     {goal_pub_r4.publish(goal_msg);}                    
                            
                        if(formation_lead_fol[robotcount] == 1){cout << "Published new position (" << goal_msg.x << "," << goal_msg.y << ") for robot_" << robotcount+1 << endl;}
                    
                        if(flog_signal.is_open() && log_signal_active == true && formation_lead_fol[robotcount] != 0){flog_signal << "-------------------------------------------------------------------" << endl;}
                        if(flog_signal.is_open() && log_signal_active == true && formation_lead_fol[robotcount] != 0){flog_signal << logtime() << " | " << "robot_" << robotcount+1 << ": " << normalizedPos.first << " " << normalizedPos.second << "\t (goalpoint)" << endl;}
            
                    }
                    if(flog_formation.is_open()){flog_formation << logtime() << " | " << "New goalpoints published" << endl;}



                    //statistics who is leader -->leader_number
                    int leader_number = (-1);
                    for(int robotcount = 0; robotcount < 4; robotcount++)
                    {
                        if(formation_lead_fol[robotcount] == 1)
                        {
                            leader_number = robotcount;
                        }
                    }

                    int index = (-1);
                    for(index = 0; index < 3 && follower_order[index] != (-1); index++)
                    {
                        geometry_msgs::Point goal_msg;
                        std::pair<double, double> normalizedPos;
                        if(index == 0)
                        {
                            normalizedPos = getNormalizedPosition(cv::Point(formation_goal_pos[leader_number][0]+DISTANCEFOL,formation_goal_pos[leader_number][1]), std::pair<int, int>(220, 220), std::pair<int, int>(800, 800));
                            formation_goal_pos[follower_order[index]][0] = formation_goal_pos[leader_number][0]+DISTANCEFOL;
                            formation_goal_pos[follower_order[index]][1] = formation_goal_pos[leader_number][1];
                        }
                        else if(index == 1)
                        {
                            normalizedPos = getNormalizedPosition(cv::Point(formation_goal_pos[leader_number][0]-DISTANCEFOL,formation_goal_pos[leader_number][1]), std::pair<int, int>(220, 220), std::pair<int, int>(800, 800));
                            formation_goal_pos[follower_order[index]][0] = formation_goal_pos[leader_number][0]-DISTANCEFOL;
                            formation_goal_pos[follower_order[index]][1] = formation_goal_pos[leader_number][1];
                        }
                        else if(index == 2)
                        {
                            normalizedPos = getNormalizedPosition(cv::Point(formation_goal_pos[leader_number][0]+DISTANCEFOL,formation_goal_pos[leader_number][1]-DISTANCEFOL), std::pair<int, int>(220, 220), std::pair<int, int>(800, 800));
                            formation_goal_pos[follower_order[index]][0] = formation_goal_pos[leader_number][0]+DISTANCEFOL;
                            formation_goal_pos[follower_order[index]][1] = formation_goal_pos[leader_number][1]-DISTANCEFOL;
                        }
                                    
                        goal_msg.x = normalizedPos.first;
                        goal_msg.y = normalizedPos.second;
                        goal_msg.z = 0;

                        if      (follower_order[index] == 0)     {goal_pub_r1.publish(goal_msg);}
                        else if (follower_order[index] == 1)     {goal_pub_r2.publish(goal_msg);}
                        else if (follower_order[index] == 2)     {goal_pub_r3.publish(goal_msg);}
                        else if (follower_order[index] == 3)     {goal_pub_r4.publish(goal_msg);}

                        if(flog_signal.is_open() && log_signal_active == true && formation_lead_fol[follower_order[index]] != 0){flog_signal << "-------------------------------------------------------------------" << endl;}
                        if(flog_signal.is_open() && log_signal_active == true && formation_lead_fol[follower_order[index]] != 0){flog_signal << logtime() << " | " << "robot_" << follower_order[index]+1 << ": " << normalizedPos.first << " " << normalizedPos.second << "\t (goalpoint)" << endl;}
                    }

                    //getting statistics for percentage output
                    formation_max_value = 0;
                    //Get starting point for displayed value % --> max value between curr and goalpos
                    for(int robotcount = 0; robotcount < 4; robotcount++)
                    {
                        if(formation_lead_fol[robotcount] !=0)
                        {
                            if(formation_max_value == 0){formation_max_value = sqrt(pow((formation_goal_pos[robotcount][0] - pos_array_curr[robotcount][0]),2) + pow((formation_goal_pos[robotcount][1] - pos_array_curr[robotcount][1]),2));}
                            else if (formation_max_value >= sqrt(pow((formation_goal_pos[robotcount][0] - pos_array_curr[robotcount][0]),2) + pow((formation_goal_pos[robotcount][1] - pos_array_curr[robotcount][1]),2))) {formation_max_value = sqrt(pow((formation_goal_pos[robotcount][0] - pos_array_curr[robotcount][0]),2) + pow((formation_goal_pos[robotcount][1] - pos_array_curr[robotcount][1]),2));}
                    
                        }
                    }
                }

            }

            for(int robotcount = 0; robotcount < 4; robotcount++)
            {
                if(formation_lead_fol[robotcount] != 0)
                {
                    std::pair<double, double> normalizedPos = getNormalizedPosition(cv::Point(formation_goal_pos[robotcount][0], formation_goal_pos[robotcount][1]), std::pair<int, int>(220, 220), std::pair<int, int>(800, 800));     
                    if(formation_lead_fol[robotcount] != 0){cout << "robot_" << robotcount+1 << "-goalpoint: " << normalizedPos.first << " | " << normalizedPos.second << endl;}
                }
            }
            cout << "--------" << endl;
        }

            //long long time ago there was a usecase of this variable
            follower_ready = true;


/* -- other concept of formation --> follower directly dependent on the leader --> robots too slow to regulate position and speed -- */

/*        else if(count_fol > 0)
        {
            //statistics who is leader -->leader_number
            int leader_number = (-1);
            for(int robotcount = 0; robotcount < 4; robotcount++)
            {
                if(formation_lead_fol[robotcount] == 1)
                {
                    leader_number = robotcount;
                }
            }

            if((follower_config == false && follower_ready == false))
            {
                //Leader drives to the centre
                if(formation_goal_pos[leader_number][0] != 400 && formation_goal_pos[leader_number][1] != 400)
                {
                    geometry_msgs::Point goal_msg;
                    std::pair<double, double> normalizedPos = getNormalizedPosition(cv::Point(400,400), std::pair<int, int>(220, 220), std::pair<int, int>(800, 800));
                    goal_msg.x = normalizedPos.first;
                    goal_msg.y = normalizedPos.second;
                    goal_msg.z = 0;

                    if      (leader_number == 0)     {goal_pub_r1.publish(goal_msg);}
                    else if (leader_number == 1)     {goal_pub_r2.publish(goal_msg);}
                    else if (leader_number == 2)     {goal_pub_r3.publish(goal_msg);}
                    else if (leader_number == 3)     {goal_pub_r4.publish(goal_msg);}                    
                                    
                    if(formation_lead_fol[leader_number] == 1){cout << "Published new position (" << goal_msg.x << "," << goal_msg.y << ") for leader: robot_" << leader_number+1 << endl;}
                                
                    formation_goal_pos[leader_number][0] = 400;
                    formation_goal_pos[leader_number][1] = 400;

                    if(flog_formation.is_open()){flog_formation << logtime() << " | " << "Setup-leader-follower: leader coordinates published" << endl;}
                    if(flog_log.is_open()){flog_log << logtime() << " | " << "Setup-leader-follower: leader coordinates published" << endl;}
                }
                cout << "GOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO" << endl;
                cout << pos_array_curr[leader_number][0] << "-----" << pos_array_curr[leader_number][1] << endl;

                if(     pos_array_curr[leader_number][0] > 370
                    &&  pos_array_curr[leader_number][0] < 430
                    &&  pos_array_curr[leader_number][1] > 370
                    &&  pos_array_curr[leader_number][1] < 430)
                    {
                        follower_config = true;
                        follower_count_main = (-10);
                        cout << "TEEEEEEEEEEEEEEEEEEEEEEEEEESSSSSSSSSSSSSSSSSSSSSTTTTTTTTTTTTT" << endl;
                    }
            }
                        
            else if(follower_ready == true || (follower_config == true && follower_ready == false))
            {  
                cout << "SSSSSSSSSSSSSSSSSSSSUUUUUUUUUUUUUUUUUUUUUUUUIIIIIIIIIIIIIIIIIIIII" << endl;
                int follower_number = 0;
                for(int robotcount = 0; robotcount < 4; robotcount++)
                {
                    if(formation_lead_fol[robotcount] == 2)
                    {
                        if(follower_count_main >= follower_count_max || (follower_config == true && follower_ready == false && follower_count_main == (-10)))
                        {
                            follower_count_main = 0;
                            
                            geometry_msgs::Point goal_msg;
                            std::pair<double, double> normalizedPos;
                            if(follower_number == 0)
                            {
                                normalizedPos = getNormalizedPosition(cv::Point(pos_array_curr[leader_number][0]+100,pos_array_curr[leader_number][1]), std::pair<int, int>(220, 220), std::pair<int, int>(800, 800));
                                formation_goal_pos[robotcount][0] = pos_array_curr[leader_number][0]+100;
                                formation_goal_pos[robotcount][1] = pos_array_curr[leader_number][1];
                                follower_number++;
                            }
                            else if(follower_number == 1)
                            {
                                normalizedPos = getNormalizedPosition(cv::Point(pos_array_curr[leader_number][0]-100,pos_array_curr[leader_number][1]), std::pair<int, int>(220, 220), std::pair<int, int>(800, 800));
                                formation_goal_pos[robotcount][0] = pos_array_curr[leader_number][0]-100;
                                formation_goal_pos[robotcount][1] = pos_array_curr[leader_number][1];
                                follower_number++;
                            }
                            else if(follower_number == 2)
                            {
                                normalizedPos = getNormalizedPosition(cv::Point(pos_array_curr[leader_number][0]+100,pos_array_curr[leader_number][1]-100), std::pair<int, int>(220, 220), std::pair<int, int>(800, 800));
                                formation_goal_pos[robotcount][0] = pos_array_curr[leader_number][0]+100;
                                formation_goal_pos[robotcount][1] = pos_array_curr[leader_number][1]-100;
                                follower_number++;
                            }

                            goal_msg.x = normalizedPos.first;
                            goal_msg.y = normalizedPos.second;
                            goal_msg.z = 0;

                            if      (robotcount == 0)     {goal_pub_r1.publish(goal_msg);}
                            else if (robotcount == 1)     {goal_pub_r2.publish(goal_msg);}
                            else if (robotcount == 2)     {goal_pub_r3.publish(goal_msg);}
                            else if (robotcount == 3)     {goal_pub_r4.publish(goal_msg);}                    
                                            
                            if(formation_lead_fol[robotcount] == 1){cout << "Published new position (" << goal_msg.x << "," << goal_msg.y << ") for robot_" << robotcount+1 << endl;}

                            if(flog_formation.is_open()){flog_formation << logtime() << " | " << "New goalpoints published" << endl;}
                        }
                    }
                    
                }

                cout << "follower_config:" << follower_config << " follower_ready:" << follower_ready << endl;
                if(follower_config == true && follower_ready == false)
                {
                    cout << "NEEEEEEEEEEEEEEEEEEEEEEEEXXXXXXXXXXXXXXXXXXTTTTTTTTTTTTTTTTt" << endl;
                    int follower_reach = 0;
                    for(int robotcount = 0; robotcount < 4; robotcount++)
                    {
                        //Check, if goalposition reached
                        if(abs(formation_goal_pos[robotcount][0] - pos_array_curr[robotcount][0]) < FORMATIONBIG && abs(formation_goal_pos[robotcount][1] - pos_array_curr[robotcount][1]) < FORMATIONBIG)
                        {
                            if(formation_lead_fol[robotcount] == 2) {follower_reach++;}
                        }
                    }

                    cout << "follower reached: " << follower_reach << "  count lead: " << count_fol << endl;

                    if(follower_reach == count_fol){follower_ready = true;}
                }

                if(follower_ready == true) {follower_count_main++;}
            }
            
                  
            
        }
        */


        end = clock();
 
        msBetweenFrames = (double(end) - double(start)) / double(CLOCKS_PER_SEC);
        
        fpsLive = double(numFrames) / double(msBetweenFrames);

        //show pictures
        if(displayMode.getValue()) imshow("Normal Image", image);
        if(displayMode.getValue()) imshow("Canny Image", imageCanny);
        if(displayMode.getValue()) imshow("warped", outputWarped);  

        //switch case for selection modes
        /*
        * q: close
        * p: print current config
        * m: Fast Switch between Detection/Calibration Mode
        * l: Load config
        * c: clear calibrated corners
        * +: Increasing Adaptive Threshold Values
        * -: Decreasing Adaptive Threshold Values
        * r: manual robot-configuration
        * d: drive robots manually --> inout: coordinates
        * 1: drive robot_1 --> doubleclick
        * 2: drive robot_2 --> doubleclick
        * 3: drive robot_3 --> doubleclick
        * 4: drive robot_4 --> doubleclick
        * k: colour-detection enable
        * f: formation-configuration
        * s: formation start/stop
        * a: publishing goalpoints for formation again
        * z: Luggas ist Semestersprecher
        * x: cassex
        * 0: start recording ofsignallog
        * o: get ASCCII code of keyboard
        */
        char option = waitKey(5 * !(debug_consts::singleCameraSteps));
        switch(option) {
            case 'q' :
                if(flog_keylog.is_open()){flog_keylog << logtime() << " | " << option << " (selected mode)" << endl;}
                cout << "Q has been pressed -> Exiting ..." << endl;

                //close logs
                if(flog_log.is_open())
                {
                    flog_log << logtime() << " | " << "Log stopped" << endl;
                    flog_log.close();
                }
                else{if(flog_error.is_open()){flog_error << logtime() << " | " << "'log.txt' could not be closed" << endl;}}
                
                if(flog_formation.is_open())
                {
                    flog_formation << logtime() << " | " << "Formation-Log stopped" << endl;
                    flog_formation.close();
                }
                else{if(flog_error.is_open()){flog_error << logtime() << " | " << "'formationlog.txt' could not be closed" << endl;}}

                if(flog_keylog.is_open())
                {
                    flog_keylog << logtime() << " | " << "Key-Log stopped" << endl;
                    flog_keylog.close();
                }
                else{if(flog_error.is_open()){flog_error << logtime() << " | " << "'keylog.txt' could not be closed" << endl;}}
                
                if(flog_signal.is_open())
                {
                    flog_signal << logtime() << " | " << "Signal-Log stopped" << endl;
                    flog_signal.close();
                }
                else{if(flog_error.is_open()){flog_error << logtime() << " | " << "'signallog.txt' could not be closed" << endl;}}

                if(flog_error.is_open())
                {
                    flog_error << logtime() << " | " << "Error-Log stopped" << endl;
                    flog_error.close();
                }


                return 10;
                break;
            case 'p' :
                if(flog_keylog.is_open()){flog_keylog << logtime() << " | " << option << " (selected mode)" << endl;}
                cout << "P has been pressed -> Printing Current Config ...\n\n\n" << endl;
                paramController.printCurrentConfig();
                cout << "\n" << endl;
                break;
            case 'm' :
                if(flog_keylog.is_open()){flog_keylog << logtime() << " | " << option << " (selected mode)" << endl;}
                cout << "M has been pressed -> Fast Switch between Detection/Calibration Mode ... \n\n" << endl;
                squareDetection.selectedValue = !squareDetection.selectedValue;
                break;
            case 'l' :
                {
                    if(flog_keylog.is_open()){flog_keylog << logtime() << " | " << option << " (selected mode)" << endl;}
                    char loadKey = waitKey(0);
                    if(flog_keylog.is_open()){flog_keylog << logtime() << " | " << loadKey << " (loaded config(case l))" << endl;}
                    cout << "\n\n\nTrying to load Config Nr. " << loadKey << "... \n\n" << endl;
                    paramController.loadConfig((int) loadKey - 48);

                    if(flog_log.is_open()){flog_log << logtime() << " | " << "Load Config No. " << loadKey << endl;}

                    break;
                }
            case 'c' :
                if(flog_keylog.is_open()){flog_keylog << logtime() << " | " << option << " (selected mode)" << endl;}
                cout << "Clearing calibrated Corners ... \n\n" << endl;
                corners.clear();
                corners.push_back(Point(inputWidth/2, inputHeight/2));
                corners.push_back(Point(inputWidth/2, inputHeight/2));
                corners.push_back(Point(inputWidth/2, inputHeight/2));

                if(flog_log.is_open()){flog_log << logtime() << " | " << "Calibrate corners" << endl;}

                break;
            case '+' :
                if(flog_keylog.is_open()){flog_keylog << logtime() << " | " << option << " (selected mode)" << endl;}
                cout << "Increasing Adaptive Threshold Values ... \n" << endl;
                adaptiveThresholdLower += 2;
                break;
            case '-' :
                flog_keylog << logtime() << " | " << option << " (selected mode)" << endl;
                cout << "Decreasing Adaptive Threshold Values ... \n" << endl;
                adaptiveThresholdLower -= 2;
                break;
            case 'r' :
            {
                if(flog_keylog.is_open()){flog_keylog << logtime() << " | " << option << " (selected mode)" << endl;}
                            setMouseCallback("warped", callback_mouse_doubleclicked_def, &goal_pub_r1);
                
                cout << "Manual Robot-Setup..." << endl;
                            cout << "Select robot to configure" << endl;

                if(colourdetection == true)
                {
                    if(flog_log.is_open()){flog_log << logtime() << " | " << "Colour-detection stopped" << endl;}
                    
                    colourdetection = false;
                }

                if(formation_start == true)
                {
                    if(flog_log.is_open()){flog_log << logtime() << " | " << "Formation stopped" << endl;}
                    
                    formation_start = false;
                }
                            
                            char robotnumber_char = waitKey(0);
                if(flog_keylog.is_open()){flog_keylog << logtime() << " | " << robotnumber_char << " (selected robotnumber(case r))" << endl;}
                            int robotnumber_int = robotnumber_char - 48;
                            
                            //numpad
                            if      (robotnumber_int == (-124)){robotnumber_int = 4;}
                            else if (robotnumber_int == (-125)){robotnumber_int = 3;}
                            else if (robotnumber_int == (-126)){robotnumber_int = 2;}
                            else if (robotnumber_int == (-127)){robotnumber_int = 1;}

                            if((robotnumber_int >= 1 && robotnumber_int <= 4))
                            {
                                    cout << endl << "robot_" << robotnumber_int << endl;
                                    cout << "Doubleclick on the screen to see the position of the robot to enter" << endl;
                                    cout << "Press any key to continue..." << endl << endl;
                                    waitKey(0);
                                    cout << "Please enter the position correctly!!" << endl;
                                    cout << "Maximum: (800 800)" << endl;
                                    cout << "Enter 'x y' and press return" << endl;
                                    
                                    
                                    int pos_input[2] = {0, 0};
                                    
                                    int zahl = 0;
                                    while(zahl < 2)
                                    {
                                            cin >> pos_input[zahl];
                                            if(flog_keylog.is_open()){flog_keylog << logtime() << " | " << pos_input[zahl] << " (selected coordinates(case r))" << endl;}
                                            if(cin.fail() || pos_input[zahl] > 800)
                                            {
                                                    cout << "Eingabefehler ab Zahl" << zahl+1 << ", Erneut: ";
                                                    if(flog_error.is_open()){flog_error << logtime() << " | " << "Input-error of the configuration oh the robot" << endl;}
                                                    cin.clear();
                                                    cin.ignore(numeric_limits<int>::max(), '\n');
                                            }
                                            else
                                            {
                                                    cout << "pos_input[" << zahl << "] = " << pos_input[zahl] << endl;
                                                    zahl++;
                                            }
                                    }
                                    
                                    for(int robotcount = 0; robotcount < 4; robotcount++)
                                    {
                                            if(pos_array_curr[robotcount][0]  != (-1) && pos_array_curr[robotcount][1] != (-1) && rob_array_active[robotcount] == true)
                                        {
                                                if(abs(pos_array_curr[robotcount][0] - pos_input[0]) < BIGBIG && abs(pos_array_curr[robotcount][1] - pos_input[1]) < BIGBIG)
                                                {
                                                    if(flog_log.is_open()){flog_log << logtime() << " | " << "robot_" << robotcount+1 << " set inactive" << endl;}
                                
                                                    rob_array_active[robotcount] = false;
                                                    pos_array_curr[robotcount][0] = (-1);                            
                                                    pos_array_curr[robotcount][1] = (-1);

                                                    //pos_array_curr[robotnumber_int-1][0] = (-1);                            
                                                    //pos_array_curr[robotnumber_int-1][1] = (-1);
                                                }
                                        }
                                    }
                                    
                                    if(flog_log.is_open()){flog_log << logtime() << " | " << "robot_" << robotnumber_int << " set active" << endl;}

                                    rob_array_active[robotnumber_int-1]  = true;
                                    pos_array_curr[robotnumber_int-1][0] = pos_input[0];                            
                                    pos_array_curr[robotnumber_int-1][1] = pos_input[1];
                                    counter_imageloss[robotnumber_int-1] = 0;
                                    
                
                            }
                            else
                            {
                                    cout << "Invalid robot number selected." << endl;
                    if(flog_error.is_open()){flog_error << logtime() << " | " << "User-error: Invalid robot number selected(case r)" << endl;}
                            }
                            break;
                    }
                    
        case 'd':
                {
            if(flog_keylog.is_open()){flog_keylog << logtime() << " | " << option << " (selected mode)" << endl;}
            setMouseCallback("warped", callback_mouse_doubleclicked_def, &goal_pub_r1);

                        if(formation_start == true)
            {
                if(flog_log.is_open()){flog_log << logtime() << " | " << "Formation stopped" << endl;}
                
                formation_start = false;
            }
            
            cout << "Robot-Drive..." << endl;
                cout << "Select robot to drive" << endl;
                
                char robotnumber_char = waitKey(0);
                if(flog_keylog.is_open()){flog_keylog << logtime() << " | " << robotnumber_char << " (selected driving robot (only 1-4)(case d))" << endl;}
                int robotnumber_int = robotnumber_char - 48;

                //numpad
                if      (robotnumber_int == (-124)){robotnumber_int = 4;}
                else if (robotnumber_int == (-125)){robotnumber_int = 3;}
                else if (robotnumber_int == (-126)){robotnumber_int = 2;}
                else if (robotnumber_int == (-127)){robotnumber_int = 1;}

                if(robotnumber_int >= 1 && robotnumber_int <= 4)
                {
                        cout << endl << "robot_" << robotnumber_int << endl;
                        cout << "Doubleclick on the screen to see the position of the robot to enter" << endl;
                        cout << "Press any key to continue..." << endl << endl;
                        waitKey(0);
                        cout << "Please enter the position correctly!!" << endl;
                        cout << "Maximum: (800 800)" << endl;
                        cout << "Enter 'x y' and press return" << endl;
                        
                        
                        int pos_input[2] = {0, 0};
                        
                        int zahl = 0;
                        while(zahl < 2)
                        {
                                cin >> pos_input[zahl];
                                if(cin.fail() || pos_input[zahl] > 800)
                                {
                                        cout << "Eingabefehler ab Zahl " << zahl+1 << ", Erneut: " << endl;
                                        if(flog_error.is_open()){flog_error << logtime() << " | " << "Input-error of the configuration oh the robot" << endl;}
                                        cin.clear();
                                        cin.ignore(numeric_limits<int>::max(), '\n');
                                }
                                else
                                {
                                        cout << "pos_input[" << zahl << "] = " << pos_input[zahl] << endl;
                                        zahl++;
                                }
                        }
                        
                    geometry_msgs::Point goal_msg;
                    std::pair<double, double> normalizedPos = getNormalizedPosition(cv::Point(pos_input[0], pos_input[1]), std::pair<int, int>(220, 220), std::pair<int, int>(800, 800));
                    goal_msg.x = normalizedPos.first;
                    goal_msg.y = normalizedPos.second;
                    goal_msg.z = 0;

                    if      (robotnumber_int == 1)     {goal_pub_r1.publish(goal_msg);}
                    else if (robotnumber_int == 2)     {goal_pub_r2.publish(goal_msg);}
                    else if (robotnumber_int == 3)     {goal_pub_r3.publish(goal_msg);}
                    else if(robotnumber_int == 4)      {goal_pub_r4.publish(goal_msg);}                    
                    
                    cout << "Published new position (" << goal_msg.x << "," << goal_msg.y << ") for robot_" << robotnumber_int << endl;
                }
                else
                {
                    cout << "Invalid robot number selected." << endl;
                    if(flog_error.is_open()){flog_error << logtime() << " | " << "User-Error: Invalid robotnumber selected" << endl;}
                }
                            
                break;
                }
    case '1':
    case (-79):
        {
            if(flog_keylog.is_open()){flog_keylog << logtime() << " | " << option << " (selected mode)" << endl;}
            if(formation_start == true)
            {
                if(flog_log.is_open()){flog_log << logtime() << " | " << "Formation stopped" << endl;}
                formation_start = false;
            }

            if(flog_log.is_open()){flog_log << logtime() << " | " << "Controlling robot_1" << endl;}
            cout << "Controlling robot_1" << endl;
            setMouseCallback("warped", callback_mouse_doubleclicked, &goal_pub_r1);
            break;
        }

    case '2':
    case (-78):
        {
            if(flog_keylog.is_open()){flog_keylog << logtime() << " | " << option << " (selected mode)" << endl;}
            if(formation_start == true)
            {
                if(flog_log.is_open()){flog_log << logtime() << " | " << "Formation stopped" << endl;}
                formation_start = false;
            }

            if(flog_log.is_open()){flog_log << logtime() << " | " << "Controlling robot_2" << endl;}
            cout << "Controlling robot_2" << endl;
            setMouseCallback("warped", callback_mouse_doubleclicked, &goal_pub_r2);
            break;
        }

    case '3':
    case (-77):
        {
            if(flog_keylog.is_open()){flog_keylog << logtime() << " | " << option << " (selected mode)" << endl;}
            if(formation_start == true)
            {
                if(flog_log.is_open()){flog_log << logtime() << " | " << "Formation stopped" << endl;}
                formation_start = false;
            }

            if(flog_log.is_open()){flog_log << logtime() << " | " << "Controlling robot_3" << endl;}
            cout << "Controlling robot_3" << endl;
            setMouseCallback("warped", callback_mouse_doubleclicked, &goal_pub_r3);
            break;
        }

    case '4':
    case (-76):
        {
            if(flog_keylog.is_open()){flog_keylog << logtime() << " | " << option << " (selected mode)" << endl;}
            if(formation_start == true)
            {
                if(flog_log.is_open()){flog_log << logtime() << " | " << "Formation stopped" << endl;}
                formation_start = false;
            }

            if(flog_log.is_open()){flog_log << logtime() << " | " << "Controlling robot_4" << endl;}
            cout << "Controlling robot_4" << endl;
            setMouseCallback("warped", callback_mouse_doubleclicked, &goal_pub_r4);
            break;
        }
    case 'k':       //k for kolour-detection
        {
            if(flog_keylog.is_open()){flog_keylog << logtime() << " | " << option << " (selected mode)" << endl;}
            setMouseCallback("warped", callback_mouse_doubleclicked_def, &goal_pub_r1);

            cout << "Automatic Robot-Setup..." << endl;
            cout << "robot_1: black" << endl;
            cout << "robot_2: red" << endl;
            cout << "robot_3: green" << endl;
            cout << "robot_4: blue" << endl;
            cout << "-------------------------------------" << endl;
            cout << "Press any key to continue..." << endl;
            
            if(flog_log.is_open()){flog_log << logtime() << " | " << "Colour-detection started" << endl;}

            colourdetection = true;

            for(int robotcount = 0; robotcount < 4; robotcount++)
            {
                if(rob_array_active[robotcount] == true)
                {
                    if(flog_log.is_open()){flog_log << logtime() << " | " << "robot_" << robotcount+1 << " set inactive" << endl;}
                    
                    rob_array_active[robotcount] = false;
                    pos_array_curr[robotcount][0] = (-1);                            
                                    pos_array_curr[robotcount][1] = (-1);
                }
                
                
            }

            waitKey(0);
            break;
        }
        case 'f':   //formation
        {
            if(flog_keylog.is_open()){flog_keylog << logtime() << " | " << option << " (selected mode)" << endl;}
            cout << "Formation-Setup..." << endl;

            if(flog_log.is_open()){flog_log << logtime() << " | " << "Formation-configure --> more information in formationlog.txt" << endl;}
            if(flog_formation.is_open()){flog_formation << logtime() << " | " << "Formation-configure" << endl;}
                
            //Input: robots part of formation
            int temp_formation_part = (-2);
            cout << "Which robots should be part of the formation?"<< endl;
            do{
                if(temp_formation_part == (-1)) {cout << "Again..." << endl << "-------------" << endl;}
                temp_formation_part = (-1);

                //Output: Configuration of formation
                cout << "-----" << endl << "Formation-Configuration" << endl;
                int count_fol = 0, count_lead = 0;
                for(int i = 0; i < 4; i++)
                {
                    cout << "robot_" << i+1 << ": ";
                    switch(formation_lead_fol[i])
                    {
                        case 0:     cout << "not part of formation" << endl;
                                    break;
                        case 1:     cout << "leader" << endl;
                                    count_lead++;
                                    break;
                        case 2:     cout << "follower" << endl;
                                    count_fol++;
                                    break;
                        default:    cout << "error" << endl;
                                    break;
                    }
                }
                if      (count_lead == 0 && count_fol == 0) {cout << "No formation possible --> activate robots" << endl;}
                else if (count_lead > 0  && count_fol == 0) {cout << "formation possible" << endl;}
                else if (count_lead == 1 && count_fol > 0 ) {cout << "formation possible" << endl;}
                else if (count_lead > 1  && count_fol > 0 ) {cout << "No formation possible --> only 1 leader with followers or multiple leaders without followers possible" << endl;}
                else if (count_lead == 0 && count_fol > 0 ) {cout << "No formation possible --> at least 1 leader required" << endl;}
                else                                        {cout << "Check code!!!" << endl;}
                cout << "-----" << endl << endl;

                //Input: robot_number
                cout << "Select the number of the robot(warped)" << endl << "-----" << "Press 5 to continue" << "-----" << endl;
                char temp_formation_part_c = waitKey(0);
                if(flog_keylog.is_open()){flog_keylog << logtime() << " | " << temp_formation_part_c << " (selected formation-robot(case f))" << endl;}
                temp_formation_part = temp_formation_part_c - 48;

                //numpad
                if      (temp_formation_part == (-124)){temp_formation_part = 4;}
                else if (temp_formation_part == (-125)){temp_formation_part = 3;}
                else if (temp_formation_part == (-126)){temp_formation_part = 2;}
                else if (temp_formation_part == (-127)){temp_formation_part = 1;}
                else if (temp_formation_part == (-123)){temp_formation_part = 5;}

                if(temp_formation_part_c == 's')
                {
                    formation_simulation = !formation_simulation;

                    if(formation_simulation == true)
                    {
                        cout << endl << "Set formation_simulation true (robots don't need to be active to start the formation)" << endl;
                    }
                    else if(formation_simulation == false)
                    {
                        cout << endl << "Set formation_simulation false (robots need to be active to start the formation)" << endl;
                    }

                    temp_formation_part = (-2);
                }
                else if(temp_formation_part <= 0 || temp_formation_part >= 6) {temp_formation_part = (-1);}

                if(temp_formation_part == (-1) || temp_formation_part == (-2) || temp_formation_part == 5){/*End of Configuration or Again*/}
                else
                {
                    int robot_selected = temp_formation_part-1;
                    int temp_formation_task = (-2);
                    do{
                        if(temp_formation_task == (-1)) {cout << "Again..." << endl << "-------------" << endl;}
                        temp_formation_task = (-1);

                        //Input: robot_task
                        cout << "Select the task of the robot (warped)" << endl;
                        cout << "0: not part of the formation" << endl;
                        cout << "1: leader" << endl;
                        cout << "2: follower" << endl;
                        char temp_formation_task_c = waitKey(0);
                        if(flog_keylog.is_open()){flog_keylog << logtime() << " | " << temp_formation_task_c << " (selected formation-task of the robot(case f))" << endl;}
                        temp_formation_task = temp_formation_task_c - 48;

                        cout << endl << temp_formation_task << endl;

                        if(temp_formation_task < 0 || temp_formation_task > 2) {temp_formation_task = (-1);}
                        else {formation_lead_fol[robot_selected] = temp_formation_task;}

                    }while(temp_formation_task == (-1));

                }


            }while(temp_formation_part != 5);
                

            do{
                if(formation_array_selection == (-1)) {cout << "Again..." << endl << "-------------" << endl;}
                formation_array_selection = (-1);

                cout << "Select the type of formation by pressing a number between 0 and 5 (warped)" << endl;

                for(int i = 0; i < 6; i++)
                {
                    cout << i << ": " << formation_selection_text[i] << endl;
                }

                char formation_array_selection_c = waitKey(0);
                if(flog_keylog.is_open()){flog_keylog << logtime() << " | " << formation_array_selection_c << " (selected formation(case f))" << endl;}
                formation_array_selection = formation_array_selection_c - 48;

                if      (formation_array_selection == (-124)){formation_array_selection = 4;}
                else if (formation_array_selection == (-125)){formation_array_selection = 3;}
                else if (formation_array_selection == (-126)){formation_array_selection = 2;}
                else if (formation_array_selection == (-127)){formation_array_selection = 1;}
                else if (formation_array_selection == (-123)){formation_array_selection = 5;}
                else if (formation_array_selection == (-128)){formation_array_selection = 0;}

                if(formation_array_selection < 0 || formation_array_selection > 5) {formation_array_selection = (-1);}

            }while(formation_array_selection == (-1));


            formation_start = false;

            //Conclusion formation
            //Output: Configuration of formation
            cout << "-----" << endl << "Formation-Configuration" << endl;
            int count_fol = 0, count_lead = 0;
            for(int i = 0; i < 4; i++)
            {
                cout << "robot_" << i+1 << ": ";
                if(flog_formation.is_open()){flog_formation << logtime() << " | " << "robot_" << i+1 << ": ";}

                switch(formation_lead_fol[i])
                {
                    case 0:     cout << "not part of formation" << endl;
                                if(flog_formation.is_open()){flog_formation << "not part of the formation" << endl;}
                                break;
                    case 1:     cout << "leader" << endl;
                                if(flog_formation.is_open()){flog_formation << "leader" << endl;}
                                count_lead++;
                                break;
                    case 2:     cout << "follower" << endl;
                                if(flog_formation.is_open()){flog_formation << "follower" << endl;}
                                count_fol++;
                                break;
                    default:    cout << "error" << endl;
                                if(flog_formation.is_open()){flog_formation << "FAIL --> errorlog" << endl;}
                                if(flog_error.is_open()){flog_error << logtime() << " | " << "FAIL: CASE_F: formation_lead_fol != [0;2]" << endl;}
                                break;
                }
            }
            if(formation_simulation == true)
            {
                cout << "simulation: true" << endl;
                if(flog_formation.is_open()){flog_formation << "simulation: true" << endl;}
            }
            else if(formation_simulation == false)
            {
                cout << "simulation: false" << endl;
                if(flog_formation.is_open()){flog_formation << "simulation: false" << endl;}
            }

            if      (count_lead == 0 && count_fol == 0)
            {
                cout << "No formation possible --> select robots" << endl;
                if(flog_formation.is_open()){flog_formation << logtime() << " | " << "No formation possible --> select robots" << endl;}
                formation_configuration = false;
            }
            else if (count_lead > 0  && count_fol == 0)
            {
                cout << "formation possible" << endl;
                if(flog_formation.is_open()){flog_formation << logtime() << " | " << "formation possible" << endl;}
                formation_configuration = true;
            }
            else if (count_lead == 1 && count_fol > 0 )
            {
                cout << "formation possible" << endl;
                if(flog_formation.is_open()){flog_formation << logtime() << " | " << "formation possible" << endl;}
                formation_configuration = true;
            }
            else if (count_lead > 1  && count_fol > 0 )
            {
                cout << "No formation possible --> only 1 leader with followers or multiple leaders without followers possible" << endl;
                if(flog_formation.is_open()){flog_formation << logtime() << " | " << "No formation possible --> only 1 leader with followers or multiple leaders without followers possible" << endl;}
                formation_configuration = false;
            }
            else if (count_lead == 0 && count_fol > 0 )
            {
                cout << "No formation possible --> at least 1 leader required" << endl;
                if(flog_formation.is_open()){flog_formation << logtime() << " | " << "No formation possible --> at least 1 leader required" << endl;}
                formation_configuration = false;
            }
            else                                        
            {
                cout << "Check code!!!" << endl;
                if(flog_error.is_open()){flog_error << logtime() << " | " << "FAIL:CASE_F count_fol, count_lead" << endl;}
                formation_configuration = false;
            }


            cout << "-----" << endl;
            cout << "formation selected: " << formation_selection_text[formation_array_selection] << endl;
            cout << "-------------------" << endl;
            if(flog_formation.is_open()){flog_formation << logtime() << " | " << "formation " << formation_array_selection << " selected" << endl;}

            follower_ready = false;
            follower_config = false;

            cout << "Press any key to continue..." << endl << endl;
                    waitKey(0);

            break;
        }
        case 's':
        {
            if(flog_keylog.is_open()){flog_keylog << logtime() << " | " << option << " (selected mode)" << endl;}
            if(formation_configuration == false)
            {
                cout << "Please configure the formation!!" << endl;
            }
            else if(formation_configuration == true && follower_ready == true)
            {
                int formation_ready = 0;
                if(formation_simulation == false)
                {
                    cout << "simulation deactivated --> all configured robots must be active !" << endl << endl;
                    for(int robotcount = 0; robotcount < 4; robotcount++)
                    {
                        if(formation_lead_fol[robotcount] != 0 && rob_array_active[robotcount] == true) {formation_ready++;}
                    }
                }
                else{formation_ready = count_formation;}
                
                
                if(formation_ready == count_formation)
                {
                    formation_start = !formation_start;

                    if(formation_start == true)
                    {
                        if(flog_log.is_open()){flog_log << logtime() << " | " << "Formation started" << endl;}
                        if(flog_formation.is_open()){flog_formation << logtime() << " | " << "Formation started" << endl;}
                        cout << "formation started" << endl;
                        formation_setup = false;
                        for(int robotcount= 0; robotcount < 4; robotcount++) {formation_step[robotcount] = 0;}
                    }
                    else
                    {
                        if(flog_log.is_open()){flog_log << logtime() << " | " << "Formation stopped" << endl;}
                        if(flog_formation.is_open()){flog_formation << logtime() << " | " << "Formation stopped" << endl;}
                        cout << "formation stopped" << endl;
                    }
                }
                else
                {
                    if(flog_formation.is_open()){flog_formation << logtime() << " | " << "formation could not be started --> set robots active!" << endl;}
                    if(flog_log.is_open()){flog_log << logtime() << " | " << "formation could not be started --> set robots active!" << endl;}
                }
                
            }
            else if(follower_ready == false)
            {
                if(flog_log.is_open()){flog_log << logtime() << " | " << "Wait for follower-configure" << endl;}
                if(flog_formation.is_open()){flog_formation << logtime() << " | " << "Wait for follower-configure" << endl;}
                cout << "Wait for follower-configure" << endl;
            }

            formation_max_value = 0;
            cout << "Press any key to continue..." << endl << endl;
                    waitKey(0);

            break;
        }
        case 'a':
        {
            if(flog_keylog.is_open()){flog_keylog << logtime() << " | " << option << " (selected mode)" << endl;}
            if((formation_configuration == true && formation_start == true && formation_setup == true )|| follower_config == true)
            {
                for(int robotcount = 0; robotcount < 4; robotcount++)
                {
                    if(formation_lead_fol[robotcount] == 1)
                    {
                        geometry_msgs::Point goal_msg;
                        std::pair<double, double> normalizedPos = getNormalizedPosition(cv::Point(formation_goal_pos[robotcount][0], formation_goal_pos[robotcount][1]), std::pair<int, int>(220, 220), std::pair<int, int>(800, 800));
                        goal_msg.x = normalizedPos.first;
                        goal_msg.y = normalizedPos.second;
                        goal_msg.z = 0;

                        if      (robotcount == 0)     {goal_pub_r1.publish(goal_msg);}
                        else if (robotcount == 1)     {goal_pub_r2.publish(goal_msg);}
                        else if (robotcount == 2)     {goal_pub_r3.publish(goal_msg);}
                        else if (robotcount == 3)     {goal_pub_r4.publish(goal_msg);}

                        cout << "Published position (" << goal_msg.x << "," << goal_msg.y << ") for robot_" << robotcount+1 << "again" << endl;
                    }
                }
                if(flog_formation.is_open()){flog_formation << logtime() << " | " << "Publishing goal_points for robot-formation again" << endl;}
            }
            break;
        }
    case 'z':
        {
            if(flog_keylog.is_open()){flog_keylog << logtime() << " | " << option << " (selected mode)" << endl;}
            if(flog_log.is_open()){flog_log << logtime() << " | " << "Luggas ist Semestersprecher. Habe Acht!" << endl;}
            break;
        }
    case 'x':
        {
            if(flog_keylog.is_open()){flog_keylog << logtime() << " | " << option << " (selected mode)" << endl;}

            if(colourdetection == true)
            {
                if(flog_log.is_open()){flog_log << logtime() << " | " << "Colour-detection stopped" << endl;}
                    
                colourdetection = false;
            }

            if(formation_start == true)
            {
                if(flog_log.is_open()){flog_log << logtime() << " | " << "Formation stopped" << endl;}
                    
                formation_start = false;
            }



            for(int robotcount = 0; robotcount < 4; robotcount++)
            {
                pos_array_prev[robotcount][0] = (-1);
                pos_array_prev[robotcount][0] = (-1);
                pos_array_curr[robotcount][0] = (-1);
                pos_array_curr[robotcount][0] = (-1);
                rob_array_active[robotcount] = false;
            }

            robot_config_casex = (-1);      //-1 setup
            if(flog_log.is_open()){flog_log << logtime() << " | " << "Wagner-configuration started" << endl;}
            cout << "Wagner-configuration started" << endl;

            counter_casex_goal = -10;

            break;
        }
    case '0':
        {
            log_signal_active = !log_signal_active;

            cout << "Signallog..." << endl;

            if(log_signal_active == true)
            {
                if(flog_signal.is_open()){flog_signal << logtime() << " | " << "Recording started ..." << endl;}
            }
            else
            {
                if(flog_signal.is_open()){flog_signal << logtime() << " | " << "Recording stopped ..." << endl;}
            }
            break;
        }
    case 'o':
        {
            if(flog_keylog.is_open()){flog_keylog << logtime() << " | " << option << " (selected mode)" << endl;}

            char input_c;
            do{
                cout << "debug" << endl;
                input_c = waitKey(0);

                if(flog_keylog.is_open()){flog_keylog << logtime() << " | char: '" << input_c << "', int: " << (int)input_c << " (case m)" << endl;}

            }while((int)input_c != 48);
            break;
        }



    }//End switch case
    
    for(int robotcount = 0; robotcount < 4; robotcount++)
    {
            if(counter_imageloss[robotcount] >= COUNTER_IMAGELOSS || rob_array_active[robotcount] == false){counter_imageloss[robotcount] = 0;}
        else if (counter_imageloss[robotcount] < COUNTER_IMAGELOSS && counter_imageloss[robotcount] >= 0 && pos_array_prev[robotcount][0] != (-1)) {counter_imageloss[robotcount] = counter_imageloss[robotcount] +1;}
        
        if( (pos_array_curr[robotcount][0] != (-1) && pos_array_curr[robotcount][1] != (-1)) ||
            (counter_imageloss[robotcount] == COUNTER_IMAGELOSS && pos_array_curr[robotcount][0] == (-1) && pos_array_prev[robotcount][0] != (-1)))
        {
            pos_array_prev[robotcount][0] = pos_array_curr[robotcount][0];
                pos_array_prev[robotcount][1] = pos_array_curr[robotcount][1];
            counter_imageloss[robotcount] = 0;

            if(pos_array_curr[robotcount][0] == (-1) && pos_array_curr[robotcount][1] == (-1))
            {
                rob_array_active[robotcount] = false;

                if(flog_log.is_open()){flog_log << logtime() << " | " << "robot_" << robotcount+1 << " set inactive (prev-curr)" << endl;}
            }
        }
        
            

            pos_array_curr[robotcount][0] = (-1);
            pos_array_curr[robotcount][1] = (-1);
    }
        
        circlesLarge.clear();
        circlesSmall.clear();
      
        //Allows for invoking Callback-Functions
        ros::spinOnce();
    }

    return 0;
}