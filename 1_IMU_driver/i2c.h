#ifndef I2C_H_
#define I2C_H_

#ifdef __cplusplus
    extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

    //************************************************************************************************
    // I2C Function prototypes for master
    //************************************************************************************************

    // Initialize I2C bus with SDA pin, SCL pin, and frequency in Hz
    // return true if successful
    bool i2c_init(uint8_t sda, uint8_t scl, uint8_t hz);

    // Deinitialize the I2C bus
    void i2c_deinit();
    
    // Set the I2C device address
    void i2c_set_device_addr(uint8_t device_addr);

    //************************************* I2C atomic transmission functions ************************
    // send ack signal
    void i2c_write_ack();

    // send nack signal
    void i2c_write_nack();

    // create start condition on I2C bus
    void i2c_write_start(void);

    // create stop condition on I2C bus
    void i2c_write_stop(void);

    // write a byte to I2C bus
    uint8_t i2c_write_byte(const uint8_t byte);

    // return true if ACK is received
    bool i2c_read_ack();

    // read a byte from I2C bus
    uint8_t i2c_read_byte();

    //************************************* I2C sequence transmission functions ************************

    // writting a byte of data (const uint8_t *data) to I2C bus
    // return true if successful
    bool i2c_write_data(uint8_t reg_addr, const uint8_t *data);

    // writting series of data bytes started at *data to I2C bus
    // start_repeated is true if don't want to send stop bit between sending bytes
    // return true if successful
    bool i2c_write_series(uint8_t reg_addr, const uint8_t *data, uint8_t length, bool start_repeated);

    // reading a byte of data (uint8_t *data) from I2C bus
    // return true if successful
    bool i2c_read_data(uint8_t reg_addr, uint8_t *data);

    // reading series of data bytes started at *data from I2C bus
    // start_repeated is true if don't want to send stop bit between sending bytes
    // return true if successful
    bool i2c_read_series(uint8_t reg_addr, uint8_t *data, uint8_t length, bool start_repeated);

    //************************************************************************************************

#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif /* I2C_H_ End of header guard */