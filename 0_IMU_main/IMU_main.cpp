#include <iostream>
#include "IMU_main.h"
// #include "../3_IMU_freefall/IMU_freefall.h"

constexpr uint8_t ACEL_AXIS_NUMBER = 3;
constexpr uint8_t GYRO_AXIS_NUMBER = 3;

const uint8_t i2cAddress = IMU_ADDRESS;
const uint16_t gyroFreq = 50;
const uint16_t gyroFullScale = 250;
const uint16_t accelFreq = 50;
const uint8_t accelFullScale = 2;
const uint8_t I2C_SDL = 0;
const uint8_t I2C_SDC = 0;
const uint8_t I2C_HZ = 0;

int main() {

    Imu imu(i2cAddress, gyroFreq, gyroFullScale, accelFreq, accelFullScale, I2C_SDL, I2C_SDC, I2C_HZ);

    // Check if the IMU was initialized successfully
    if (!imu.isInitialized()) {
        std::cerr << "Failed to initialize IMU" << std::endl;
        return -1;
    }

    if (!imu.startAcquisition()) {
        std::cerr << "Failed to start acquisition" << std::endl;
        return -1;
    }

    while(1) {
        std::vector<double> gyroData;
        if (imu.readGyroData(gyroData)) {
            imu.showGyroData(gyroData);
        }
        else {
            std::cerr << "Reinitialisation" << std::endl;
            IMU_init(i2cAddress, gyroFreq, gyroFullScale, accelFreq, accelFullScale, I2C_SDL, I2C_SDC, I2C_HZ);
            imu.isInitialized();
            imu.startAcquisition();
        }

        std::vector<double> accelData;
        if (imu.readAccelData(accelData)) {
            imu.showAccelData(accelData);
        }
        else {
            std::cerr << "Reinitialisation" << std::endl;
            IMU_init(i2cAddress, gyroFreq, gyroFullScale, accelFreq, accelFullScale, I2C_SDL, I2C_SDC, I2C_HZ);
            imu.isInitialized();
            imu.startAcquisition();
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
    accelData.resize(ACEL_AXIS_NUMBER);
    return IMU_read_accel_data(&accelData[0], &accelData[1], &accelData[2]);
}

void Imu::showAccelData(const std::vector<double>& accelData) {
    std::cout << "Accel data: ";
    for (const auto& data : accelData) {
        std::cout << data << " ";
    }
    std::cout << std::endl;
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
