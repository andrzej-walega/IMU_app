#ifndef IMU_FREEFALL_H_
#define IMU_FREEFALL_H_

#include <vector>
#include <ctime>
#include "../0_IMU_main/observer.h"
#include "../0_IMU_main/IMU_main.h"

class FreefallDetector: public IDataAnalyzer {

public:
    FreefallDetector(Imu& imu, uint32_t freefallTimeDuration, double freefallThreshold);

    bool isDataReady();
    void setDataReady(bool dataReady);
    void getAccelData(std::vector<double> accelData);
    void setTimeThreshold(uint32_t timeTreshold);
    void setThreshold(double threshold);
    void analyze(const std::vector<double>& data);
    void doNotification();

private:
    // void analyzeMotion();

    Imu& imu_;
    clock_t startTimeDuration_ {0};
    uint32_t timeThreshold_ {0};
    double threshold_ {0};
    std::vector<double> accelData_;
    double ax_ {0};
    double ay_ {0};
    double az_ {0};
    bool isLastNotifyValid_ { false };
    bool isFreefallPrev_ {false};
    bool isFreefallCurrent_ {false};
    bool dataReady_ {false};
};

#endif // IMU_FREEFALL_H_