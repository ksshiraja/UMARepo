// Copyright 2019 UM::Autonomy

#ifndef UMA_CONTROLS_PID_INTERFACE_H
#define UMA_CONTROLS_PID_INTERFACE_H

class PidInterface
{
public:
    PidInterface() {}
    virtual void setSetpoint(double setpoint) = 0;
    virtual void setState(double state) = 0;
    virtual double getEffort() = 0;
};

#endif  // UMA_CONTROLS_PID_INTERFACE_H
