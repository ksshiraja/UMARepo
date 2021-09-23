// Copyright 2019 UM::Autonomy

#ifndef UMA_CONTROLS_AHRS8_INTERFACE_H
#define UMA_CONTROLS_AHRS8_INTERFACE_H

#include <string>

class Ahrs8Interface
{
public:
    Ahrs8Interface() : data_period("1") {}

    virtual void write_baudrate(const std::string &baudrate) = 0;
    virtual void start_streaming_data() = 0;

    virtual void write_configuration() = 0;

    virtual bool block_until_new_data() = 0;

    virtual void compass_update_WMM() = 0;

    virtual void calibration_start_2d() = 0;
    virtual void calibration_capture_point() = 0;
    virtual void calibration_end_capture() = 0;
    virtual std::string calibration_get_error() = 0;
    virtual void calibration_end() = 0;

    float linear_acceleration[3];
    float angular_velocity[3];
    float orientation_quaternion[4];

protected:
    std::string data_period;
};

#endif  // UMA_CONTROLS_AHRS8_INTERFACE_H
