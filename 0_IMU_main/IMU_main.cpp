#include <iostream>
#include "IMU_main.h"
// #include "../3_IMU_freefall/IMU_freefall.h"

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

    std::vector<uint8_t> gyroData;
    if (imu.readGyroData(gyroData)) {
        std::cout << "Gyro data read successfully" << std::endl;
        imu.showGyroData(gyroData);
    }
    else {
        std::cerr << "Failed to read gyro data" << std::endl;
    }

    std::vector<uint8_t> accelData;
    if (imu.readAccelData(accelData)) {
        std::cout << "Accel data read successfully" << std::endl;
        imu.showAccelData(accelData);
    }
    else {
        std::cerr << "Failed to read accel data" << std::endl;
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

bool Imu::readAccelData(std::vector<uint8_t>& accelData) {
    accelData.resize(ACCEL_DATA_SIZE);
    return IMU_read_accel_data(accelData.data());
}

void Imu::showAccelData(const std::vector<uint8_t>& accelData) {
    std::cout << "Accel data: ";
    for (const auto& data : accelData) {
        std::cout << static_cast<int>(data) << " ";
    }
    std::cout << std::endl;
}

bool Imu::setGyroFreq(uint16_t gyroFreqRegVal) {
    return IMU_set_gyro_freq(gyroFreqRegVal);
}

bool Imu::setGyroRange(uint16_t gyroRangeRegVal) {
    return IMU_set_gyro_range(gyroRangeRegVal);
}

bool Imu::readGyroData(std::vector<uint8_t>& gyroData) {
    gyroData.resize(GYRO_DATA_SIZE);
    return IMU_read_gyro_data(gyroData.data());
}

void Imu::showGyroData(const std::vector<uint8_t>& gyroData) {
    std::cout << "Gyro data: ";
    for (const auto& data : gyroData) {
        std::cout << static_cast<int>(data) << " ";
    }
    std::cout << std::endl;
}
