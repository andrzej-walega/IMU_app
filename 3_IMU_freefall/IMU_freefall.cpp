#include <iostream>
#include "IMU_freefall.h"
#include "../0_IMU_main/IMU_main.h"

FreefallDetector::FreefallDetector(Imu& imu, uint32_t freefallTimeDuration, double freefallThreshold)
    : imu_(imu), timeThreshold_(freefallTimeDuration), threshold_(freefallThreshold)
{}

bool FreefallDetector::isDataReady() {
    return dataReady_;
}

void FreefallDetector::setDataReady(bool dataReady) {
    dataReady_ = dataReady;
}

void FreefallDetector::getAccelData(std::vector<double> accelData) {
    accelData_.resize(ACCEL_AXIS_NUMBER);
    accelData_[0] = accelData[0];
    accelData_[1] = accelData[1];
    accelData_[2] = accelData[2];
}

void FreefallDetector::setTimeThreshold(uint32_t timeThreshold) {
    timeThreshold_ = timeThreshold;
}

void FreefallDetector::setThreshold(double threshold) {
    threshold_ = threshold;
}

void FreefallDetector::analyze(const std::vector<double>& data) {

    bool isAccelThresholdDetected = false;
    bool isTimeThresholdDetected = false;
    const uint16_t MICROSEC_TO_MILLISEC = 1000;

    isAccelThresholdDetected =
        abs(data[0]) <= abs(threshold_)
        && abs(data[1]) <= abs(threshold_)
        && abs(data[2]) <= abs(threshold_) ;
    
    if (isAccelThresholdDetected == true)
    {
        if (isFreefallPrev_ == false) {
            isFreefallPrev_ = true;
            startTimeDuration_ = clock();
        }
        isTimeThresholdDetected = ((clock() - startTimeDuration_) / (CLOCKS_PER_SEC / MICROSEC_TO_MILLISEC)) >= timeThreshold_;

        if (isLastNotifyValid_ == false && isTimeThresholdDetected == true) {
            doNotification();
        }
    }
    if (isAccelThresholdDetected == false && isFreefallPrev_ == true)
    {
        isFreefallPrev_ = false;
        startTimeDuration_ = 0;
        isLastNotifyValid_ = false;
    }
}

void FreefallDetector::doNotification() {
    std::cout << "  >>> FREEFALL DETECTED <<<  " << std::endl;
    isLastNotifyValid_ = true; // prevent from serial notifications
}