#ifndef CONTROLL_WINDOW_HPP
#define CONTROLL_WINDOW_HPP

#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <string>
#include <vector>
#include <iostream>

#include "utility.hpp"

struct parameterController;
struct parameterDescription;

struct parameterController {
    private:
        std::string windowName;
    public:
        parameterController(std::string windowName = "Control Window");

        std::vector<parameterDescription*>  descriptors;

        void addParam(parameterDescription* param);
        
        void createTrackbars();

        void printCurrentConfig();
        void loadConfig(unsigned long configNr);
};

struct parameterDescription {
    private:
        const int minValue;
        const int maxValue;
        
        int startValue;
        
        const std::string name;
        cv::TrackbarCallback callbackFunction;
    public:
        int selectedValue = -1;
    
        parameterDescription(int minValue, int maxValue, int startValue, std::string name, cv::TrackbarCallback callbackFunction);

        std::string getName() const;
        cv::TrackbarCallback getCallbackFunction() const;
        int getValue() const;
        int getMaxValueForSlider() const;
}; 

#endif