#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#include <string>

#define DEV_MODE

//Inline ensures every instance of constant is using same place in memory
//Constants are seperated and collected in Namespaces

//Constants for Configuration of ROS Publisher System
namespace ros_consts {
    //Initializing constexpr string requires string_view
    //Initializing as const still sufficient

    inline const bool sim = true;
    inline const std::string sim_topic_name = "/Robot1/pose2DCam";

    inline const std::string pose_topic_name_r1 = "/robot_1/pose";
    inline const std::string pose_topic_name_r2 = "/robot_2/pose";
    inline const std::string pose_topic_name_r3 = "/robot_3/pose";
    inline const std::string pose_topic_name_r4 = "/robot_4/pose";
    
    inline const std::string goal_topic_name_r1 = "/robot_1/goal_point";
    inline const std::string goal_topic_name_r2 = "/robot_2/goal_point";
    inline const std::string goal_topic_name_r3 = "/robot_3/goal_point";
    inline const std::string goal_topic_name_r4 = "/robot_4/goal_point";
}

//Constants for Configuration of camera Detection
//constexpr because "compile time constant" -> value can be assigned to other const: "const x = (constexpr)...;""
namespace camera_consts {
    //Size of Initial Camera Input -> "Camera input Resolution"
    inline constexpr int inputWidth = 1280; //640
    inline constexpr int inputHeight = 960; //480

    //Size of Final Detection Window and its "Resolution"
    inline constexpr int destWidth = 800; //400
    inline constexpr int destHeight = 800;  //400

    inline constexpr float sensitivity = 0.02f;
    inline constexpr float sensitivityCorner = 0.4f;

    inline constexpr std::pair<int, int> innerSize(7, 20);
    inline constexpr std::pair<int, int> outerSize(20, 40);

    //Min and Max Size of the Detected Squares and their Contours
    inline constexpr int minSizeSquare = 100;//100//1000
    inline constexpr int maxSizeSquare = 3500;
}

//Constants for Debugging Purposes
namespace debug_consts {
    inline constexpr bool singleCameraSteps = false;
    inline constexpr bool displayBuildInformations = false;
    
    inline constexpr bool printPublishedValues = true;
    inline constexpr bool drawDetectedCircles = false; 
}

#endif
