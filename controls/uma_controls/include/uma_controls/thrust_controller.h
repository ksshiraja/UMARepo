// Copyright 2019 UM::Autonomy

#ifndef UMA_CONTROLS_THRUST_CONTROLLER_H
#define UMA_CONTROLS_THRUST_CONTROLLER_H

#include "uma_controls/controllerConfig.h"
#include "types/uma_point.h"
#include "types/uma_vector.h"


class ThrustController
{
public:
    virtual bool isDone() const = 0;
    virtual bool isActive() const = 0;

    virtual float getYaw() const = 0;
    virtual uma::Point getLoc() const = 0;

    virtual double getLeftThrust() const = 0;
    virtual double getRightThrust() const = 0;

    virtual void updateState() = 0;
    virtual void reset() = 0;

    virtual void setConfig(const uma_controls::controllerConfig& config, const uint32_t level) = 0;
    virtual void setPose(const uma::Point& loc, const float yaw) = 0;
    virtual void setWaypoint(const uma::Point& goal, const float yaw, const float linear_tolerance,
        const float angular_tolerance) = 0;
};

#endif  // UMA_CONTROLS_THRUST_CONTROLLER_H
