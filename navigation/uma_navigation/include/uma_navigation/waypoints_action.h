#ifndef UMA_NAVIGATION_WAYPOINTS_ACTION_H
#define UMA_NAVIGATION_WAYPOINTS_ACTION_H

#include <uma_navigation/waypoint.h>
#include <uma_navigation_external_interface/WaypointsAction.h>


#include <vector>

using WaypointsAction = uma_navigation_external_interface::WaypointsAction;

using WaypointsGoal = uma_navigation_external_interface::WaypointsGoal;

using WaypointsGoalConstPtr = uma_navigation_external_interface::WaypointsGoalConstPtr;

using WaypointsFeedback = uma_navigation_external_interface::WaypointsFeedback;

using WaypointsFeedbackConstPtr = uma_navigation_external_interface::WaypointsFeedbackConstPtr;

using WaypointsResult = uma_navigation_external_interface::WaypointsResult;

using WaypointsResultConstPtr = uma_navigation_external_interface::WaypointsResultConstPtr;

/*
 * Converts the spline path and target information to the waypoints ROS message type
 * @param spline_waypoints - the spline path, an array of coordinates controls should go to
 * @param end_yaw - the target yaw the boat should end our path on.
 * @param end_pos_error - allowed pos error of boat's ending position.
 * @param end_yaw_error - allowed yaw error of boat's ending position.
 * @return bool - Returns true if waypoint reached within the specified error bounds
 */
WaypointsGoal convertToWaypointsGoal(const std::vector<Coordinate<double> >* spline_waypoints,
    double end_yaw, double end_pos_error, double end_yaw_error, double velocity, bool stop_command)
{
    WaypointsGoal res;
    res.x.resize(spline_waypoints->size());
    res.y.resize(spline_waypoints->size());
    for (size_t i = 0; i < spline_waypoints->size(); ++i)
    {
        res.x[i] = (*spline_waypoints)[i].x;
        res.y[i] = (*spline_waypoints)[i].y;
    }
    res.velocity = velocity;
    res.end_yaw = end_yaw;
    res.end_pos_error = end_pos_error;
    res.end_yaw_error = end_yaw_error;
    res.stop_command = stop_command;
    return res;
}

#endif  // UMA_NAVIGATION_WAYPOINTS_ACTION_H
