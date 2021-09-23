// Copyright 2019 UM::Autonomy

#ifndef UMA_CONTROLS_AHRS8_NMEA_H
#define UMA_CONTROLS_AHRS8_NMEA_H

#include "uma_controls/ahrs8_interface.h"
#include "serial/serial.h"
#include "uma_controls/NMEA_0183.h"
#include <string>
#include <vector>

#include "uma_controls/controls_error.h"
#include <iostream>

#define millideg_to_rad 3.14159 / 180.0 / 1000
#define millig_to_mps2 9.8 / 1000

class Ahrs8NMEA : public Ahrs8Interface
{
public:
    explicit Ahrs8NMEA(serial::Serial *s, const std::string &period) :
        data_period(period),
        sensor_serial(*s)
    {
        sensor_serial.setTimeout(100, 100, 1, 100, 1);
        sensor_serial.flush();
    }

    void compass_update_WMM() {}

    void write_configuration()
    {
        sensor_serial.write("$PSRFS,restorefactorycal,set,1\r\n");
        sensor_serial.write("$PSRFS,restoreFieldCal,set,1\r\n");
        sensor_serial.write("$PSRFS,orientation,set,0\r\n");
        sensor_serial.readline();
    }

    void write_baudrate(const std::string &baudrate)
    {
        sensor_serial.write("$PSPA,BAUD=" + baudrate + "\r\n");
    }

    void start_streaming_data()
    {
        sensor_serial.write("$PSRFS,quaternion,get,RPT=" + data_period + "\r\n");
        sensor_serial.write("$PSRFS,accelp,get,GLOM\r\n");
        sensor_serial.write("$PSRFS,gyrop,get,GLOM\r\n");
    }

    bool block_until_new_data()
    {
        std::vector<std::string> responses;

        for (int i = 0; i < 3; ++i)
            responses.push_back(sensor_serial.readline());

        for (const std::string &response : responses)
        {
            NMEAMessage msg(response);
            if (msg.get_command() == "$PSRFS")
            {
                if (msg.flag_present("gyrop"))
                {
                    if (msg.num_flags() == 5)
                    {
                        for (int i = 0; i < 3; ++i)
                            angular_velocity[i] = millideg_to_rad * stod(msg.get_flag(i+2));
                    }
                    else
                    {
                        return false;
                    }
                }
                else if (msg.flag_present("accelp"))
                {
                    if (msg.num_flags() == 5)
                    {
                        for (int i = 0; i < 3; ++i)
                            linear_acceleration[i] = millig_to_mps2 * stod(msg.get_flag(i+2));
                        }
                    else
                    {
                        return false;
                    }
                }
                else if (msg.flag_present("quaternion"))
                {
                    if (msg.num_flags() == 6)
                    {
                        for (int i = 0; i < 4; ++i)
                            orientation_quaternion[i] = stod(msg.get_flag(i+2));
                    }
                    else
                    {
                        return false;
                    }
                }
                else
                {
                    return false;
                }
            }
        }

        return true;
    }

    void calibration_start_2d()
    {
        sensor_serial.write("$PSPA,CAL=2D\r\n");
        sensor_serial.write("$PSPA,CAL_CMD=START_CAL\r\n");
    }

    void calibration_capture_point()
    {
        sensor_serial.write("$PSPA,CAL_CMD=CAPTURE\r\n");
    }

    void calibration_end_capture()
    {
        sensor_serial.write("$PSPA,CAL_CMD=END_CAPTURE\r\n");
    }

    std::string calibration_get_error()
    {
        sensor_serial.write("$PSPA,MAGERR,RPT=0.5\r\n");
        return sensor_serial.readline();
    }

    void calibration_end()
    {
        sensor_serial.write("$PSPA,CAL_CMD=END_CAL\r\n");
        sensor_serial.write("$PSPA,CAL=OFF\r\n");
    }

protected:
    serial::Serial &sensor_serial;
    std::string data_period;
};

#endif  // UMA_CONTROLS_AHRS8_NMEA_H
