#ifndef UMA_NAVIGATION_WAYPOINT_H
#define UMA_NAVIGATION_WAYPOINT_H

#include <math.h>

template <class T>
struct Coordinate
{
    T x;
    T y;
};

struct Waypoint
{
    static constexpr double EQUAL_TOLERANCE = 0.001;
    double x, y;
    // this is an angle (in radians) counter-clockwise from east (costmap x axis)
    double yaw;
    bool travel_to;

    Waypoint(double x_, double y_, double yaw_ = 0.0, bool travel_to_ = true):
        x(x_),
        y(y_),
        yaw(yaw_),
        travel_to(travel_to_)
    {}
    Waypoint():
        x(0),
        y(0),
        yaw(0),
        travel_to(true)
    {}

    bool operator==(const Waypoint &other)
    {
        return fabs(x - other.x) <= EQUAL_TOLERANCE && fabs(y - other.y) <= EQUAL_TOLERANCE
                && travel_to == other.travel_to;
    }

    bool operator!=(const Waypoint &other)
    {
        return fabs(x - other.x) > EQUAL_TOLERANCE || fabs(y - other.y) > EQUAL_TOLERANCE
                || travel_to != other.travel_to;
    }

    friend bool operator==(const Waypoint &a, const Waypoint &b)
    {
        return fabs(a.x - b.x) <= EQUAL_TOLERANCE && fabs(a.y - b.y) <= EQUAL_TOLERANCE
                && a.travel_to == b.travel_to;
    }
};

/*
* Tells if a waypoint is within error bounds of its target
* @param target - the destination we're comparing the waypoint to 
* @param waypoint - waypoint we're interested in
* @param error/error_yaw - the amount of error we'll allow
* @param ignore_yaw - do we care about yaw?
*/

template <class TargetType, class WaypointType>
bool waypointWithinBounds(const TargetType &target, const WaypointType &waypoint,
        double error, double error_yaw, bool ignore_yaw = false)
{
    // if we don't care about the yaw value
    if (!ignore_yaw)
    {
        return pow(target.x - waypoint.x, 2) + pow(target.y - waypoint.y, 2) <= pow(error, 2);
    }

    // if this is testing final waypoint from task planning, so we care about yaw
    return pow(target.x - waypoint.x, 2) + pow(target.y - waypoint.y, 2) <= pow(error, 2) &&
        fabs(target.yaw - waypoint.yaw) <= error_yaw;
}

/*
* Takes some waypoint-derived input (FromWaypointType), and 
* converts it to another waypoint derived type (ToWaypointType)
* by re-assigning the components of a waypoint
*/

template <class ToWaypointType, class FromWaypointType>
ToWaypointType waypointFrom(const FromWaypointType &w)
{
    ToWaypointType res;
    res.x = w.x;
    res.y = w.y;
    res.yaw = w.yaw;
    return res;
}

#endif  // UMA_NAVIGATION_WAYPOINT_H
