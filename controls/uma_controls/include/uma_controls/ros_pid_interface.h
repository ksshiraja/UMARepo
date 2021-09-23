// Copyright 2019 UM::Autonomy

#ifndef UMA_CONTROLS_ROS_PID_INTERFACE_H
#define UMA_CONTROLS_ROS_PID_INTERFACE_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <string>

#include "uma_controls/pid_interface.h"

class RosPidInterface : public PidInterface
{
public:
    RosPidInterface(ros::NodeHandle *node, const std::string& name);
    virtual void setSetpoint(double setpoint);
    virtual void setState(double state);
    virtual double getEffort();

protected:
    virtual void effortCallback(const std_msgs::Float64::ConstPtr& msg);

private:
    std::string name_;

    double effort_;
    double setpoint_;
    double state_;

    ros::Publisher state_publisher_;
    ros::Publisher setpoint_publisher_;
    ros::Subscriber effort_subscriber_;
};
#endif  // UMA_CONTROLS_ROS_PID_INTERFACE_H
