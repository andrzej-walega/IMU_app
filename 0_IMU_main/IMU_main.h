#ifndef IMU_MAIN_H_
#define IMU_MAIN_H_

#include <vector>
#include "../1_IMU_driver/ICM_42670_P_driver.h"

constexpr uint8_t ACCEL_AXIS_NUMBER = 3;
constexpr uint8_t GYRO_AXIS_NUMBER = 3;

class Imu {

public:
    Imu(uint8_t address, uint16_t gyroFreq, uint16_t gyroRange, uint16_t accelFreq, uint8_t accelRange,
        uint8_t I2C_SDL, uint8_t I2C_SDC, uint8_t I2C_Hz);

    bool isInitialized() const;

    bool startAcquisition();
    bool stopAcquisition();

    bool setAccelFreq(uint16_t accelFreqRegVal);
    bool setAccelRange(uint8_t accelRangeRegVal);
    bool readAccelData(std::vector<double>& accelData);
    void showAccelData(const std::vector<double>& accelData);
    void getAccelData(std::vector<double>& accelData);

    bool setGyroFreq(uint16_t gyroFreqRegVal);
    bool setGyroRange(uint16_t gyroRangeRegVal);
    bool readGyroData(std::vector<double>& gyroData);
    void showGyroData(const std::vector<double>& gyroData);
    void getGyroData(std::vector<double>& gyroData);

    void setNewDataReady(bool dataReady);
    bool isNewDataReady();

private:
    uint8_t address_;
    uint16_t accelFreq_;
    uint8_t accelRange_;
    uint16_t gyroFreq_;
    uint16_t gyroRange_;
    std::vector<double> accelData_;
    std::vector<double> gyroData_;
    bool initialized_;
    bool acquisitionStarted_;
    bool dataReady_;
};

#endif // IMU_MAIN_H_