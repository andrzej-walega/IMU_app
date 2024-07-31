#include <iostream>
#include <memory>
#include "observer.h"
#include "IMU_main.h"
#include "../3_IMU_freefall/IMU_freefall.h"

int main() {

    const uint8_t i2cAddress = IMU_ADDRESS;
    const uint16_t gyroFreq = 50;
    const uint16_t gyroScale = 250;
    const uint16_t accelFreq = 50;
    const uint8_t accelScale = 2;
    const uint8_t I2C_SDL = 0;
    const uint8_t I2C_SDC = 0;
    const uint8_t I2C_HZ = 0;

    Imu imu(i2cAddress, gyroFreq, gyroScale, accelFreq, accelScale, I2C_SDL, I2C_SDC, I2C_HZ);

    // Create data provider
    DataProvider dataProvider;

    // Create analyzers
    uint32_t freefallTimeThresholdInitVal = 50; // milliseconds
    double freefallThresholdInitVal = 1.0;

    std::shared_ptr<IDataAnalyzer> freefallDetector = std::make_shared<FreefallDetector>(imu, freefallTimeThresholdInitVal, freefallThresholdInitVal);
    // here create another module

    // Register analyzers with the data provider
    dataProvider.registerAnalyzer(freefallDetector);
    // here register another module
    

    if (!imu.isInitialized()) {
        std::cerr << "Failed to initialize IMU" << std::endl;
        return -1;
    }

    if (!imu.startAcquisition()) {
        std::cerr << "Failed to start acquisition" << std::endl;
        return -1;
    }
    std::cout << "main0" <<  std::endl;
    while (1) {
        std::vector<double> gyroData;
        if (imu.readGyroData(gyroData)) {
        }

        std::vector<double> accelData;
        if (imu.readAccelData(accelData)) {
            dataProvider.setDataAndExecute(accelData);
            // here add another analyser module to execute with new data
        }


    }


    if (!imu.stopAcquisition()) {
        std::cerr << "Failed to stop acquisition" << std::endl;
        return -1;
    }

    return 0;
}

Imu::Imu(uint8_t address, uint16_t gyroFreq, uint16_t gyroRange, uint16_t accelFreq, uint8_t accelRange,
    uint8_t I2C_SDL, uint8_t I2C_SDC, uint8_t I2C_Hz)
    : address_(address), gyroFreq_(gyroFreq), gyroRange_(gyroRange), accelFreq_(accelFreq),
    accelRange_(accelRange), acquisitionStarted_(false), dataReady_(false) {

    initialized_ = IMU_init(address, gyroFreq, gyroRange, accelFreq, accelRange, I2C_SDL, I2C_SDC, I2C_Hz);
}

bool Imu::isInitialized() const {
    return initialized_;
}

bool Imu::startAcquisition() {
    if (IMU_start_acquisition()) {
        acquisitionStarted_ = true;
        return true;
    }
    return false;
}

bool Imu::stopAcquisition() {
    if (IMU_stop_acquisition()) {
        acquisitionStarted_ = false;
        return true;
    }
    return false;
}

bool Imu::setAccelFreq(uint16_t accelFreqRegVal) {
    return IMU_set_accel_freq(accelFreqRegVal);
}

bool Imu::setAccelRange(uint8_t accelRangeRegVal) {
    return IMU_set_accel_range(accelRangeRegVal);
}

bool Imu::readAccelData(std::vector<double>& accelData) {
    accelData.resize(ACCEL_AXIS_NUMBER);
    return IMU_read_accel_data(&accelData[0], &accelData[1], &accelData[2]);
}

void Imu::showAccelData(const std::vector<double>& accelData) {
    std::cout << "Accel data: ";
    for (const auto& data : accelData) {
        std::cout << data << " ";
    }
    std::cout << std::endl;
}

void Imu::getAccelData(std::vector<double>& accelData) {
    if (accelData.size() < ACCEL_AXIS_NUMBER) {
        accelData.resize(ACCEL_AXIS_NUMBER);
    }
    for (int i = 0; i < ACCEL_AXIS_NUMBER; i++) {
        accelData[i] = accelData_[i];
    }
}

bool Imu::setGyroFreq(uint16_t gyroFreqRegVal) {
    return IMU_set_gyro_freq(gyroFreqRegVal);
}

bool Imu::setGyroRange(uint16_t gyroRangeRegVal) {
    return IMU_set_gyro_range(gyroRangeRegVal);
}

bool Imu::readGyroData(std::vector<double>& gyroData) {
    gyroData.resize(GYRO_AXIS_NUMBER);
    return IMU_read_gyro_data(&gyroData[0], &gyroData[1], &gyroData[2]);
}

void Imu::showGyroData(const std::vector<double>& gyroData) {
    std::cout << "Gyro data: ";
    for (const auto& data : gyroData) {
        std::cout << data << " ";
    }
    std::cout << std::endl;
}

void Imu::getGyroData(std::vector<double>& gyroData) {
    if (gyroData.size() < GYRO_AXIS_NUMBER) {
        gyroData.resize(GYRO_AXIS_NUMBER);
    }
    for (int i = 0; i < GYRO_AXIS_NUMBER; i++) {
        gyroData[i] = gyroData_[i];
    }
}

void Imu::setNewDataReady(bool dataReady) {
    dataReady_ = dataReady;
}

bool Imu::isNewDataReady() {
    return dataReady_;
}