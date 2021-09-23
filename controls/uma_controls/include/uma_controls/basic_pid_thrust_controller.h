// Copyright 2019 UM::Autonomy

#ifndef UMA_CONTROLS_BASIC_PID_THRUST_CONTROLLER_H
#define UMA_CONTROLS_BASIC_PID_THRUST_CONTROLLER_H

#include "uma_controls/thrust_controller.h"
#include "uma_controls/pid_interface.h"
#include "uma_controls/controllerConfig.h"
#include "types/uma_point.h"
#include "types/uma_vector.h"

class BasicPidThrustController : public ThrustController
{
public:
    explicit BasicPidThrustController(std::unique_ptr<PidInterface> *pid);

    virtual bool isDone() const;
    virtual bool isActive() const;

    virtual double getLeftThrust() const;
    virtual double getRightThrust() const;

    virtual float getYaw() const;
    virtual uma::Point getLoc() const;

    virtual void updateState();
    virtual void reset();

    virtual void setConfig(const uma_controls::controllerConfig& config, const uint32_t level);
    virtual void setPose(const uma::Point& loc, const float yaw);
    virtual void setWaypoint(const uma::Point& goal, const float yaw, const float linear_tolerance,
        const float angular_tolerance);

protected:
    enum ThrustControllerMode
    {
        IDLE,
        TURN_INITIAL,
        DRIVE,
        TURN_FINAL,
        DONE
    };

    virtual void initialTurn();
    virtual void finalTurn();
    virtual void drive();
    virtual void idle();

    virtual void calculateThrust();

    std::unique_ptr<PidInterface> pid_;

    ThrustControllerMode mode_;
    uma_controls::controllerConfig config_;

    double thrust_left_;
    double thrust_right_;

    double yaw_setpoint_;
    double yaw_;
    double yaw_error_;
    double yaw_final_;

    uma::Point goal_;
    uma::Point loc_;
    uma::Vector error_;

    double linear_tolerance_;
    double angular_tolerance_;
};


#endif  // UMA_CONTROLS_BASIC_PID_THRUST_CONTROLLER_H
