#ifndef UMA_NAVIGATION_PLANNER_INPUT_H
#define UMA_NAVIGATION_PLANNER_INPUT_H

#include <costmap_2d/costmap_2d.h>

#include <memory>

struct Orientation
{
    double x;
    double y;
    double xVel;
    double yVel;
    double orientationAngle;
};

struct Target
{
    double x;
    double y;
    double velocity;
    double yaw;
    double error;
    double error_yaw;
    bool stop_command;
};

struct PlannerInput
{
    Orientation pose;  // current boat orientation
    Target target;  // current target set by task planner
};

#endif  // UMA_NAVIGATION_PLANNER_INPUT_H
