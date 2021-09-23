#ifndef UMA_NAVIGATION_WAYPOINT_ACTION_H
#define UMA_NAVIGATION_WAYPOINT_ACTION_H

#include <uma_navigation/waypoint.h>
#include <uma_navigation_external_interface/WaypointAction.h>

using WaypointAction = uma_navigation_external_interface::WaypointAction;

using WaypointGoal = uma_navigation_external_interface::WaypointGoal;

using WaypointGoalConstPtr = uma_navigation_external_interface::WaypointGoalConstPtr;

using WaypointFeedback = uma_navigation_external_interface::WaypointFeedback;

using WaypointFeedbackConstPtr = uma_navigation_external_interface::WaypointFeedbackConstPtr;

using WaypointResult = uma_navigation_external_interface::WaypointResult;

using WaypointResultConstPtr = uma_navigation_external_interface::WaypointResultConstPtr;

template <class ToWaypointType, class FromWaypointType>
ToWaypointType waypointFromWithGoal(const FromWaypointType &w)
{
    ToWaypointType res = waypointFrom<ToWaypointType, FromWaypointType>(w);
    res.error = w.error;
    res.error_yaw = w.error_yaw;
    res.velocity = w.velocity;
    res.stop_command = w.stop_command;
    return res;
}

// not currently used, but may be useful in th future
template <class ToWaypointType, class FromWaypointType>
ToWaypointType waypointFromWithGoal(const FromWaypointType &w, double error, double error_yaw,
                                    double velocity, bool stop_command)
{
    ToWaypointType res = waypointFrom<ToWaypointType, FromWaypointType>(w);
    res.error = error;
    res.error_yaw = error_yaw;
    res.velocity = velocity;
    res.stop_command = stop_command;
    return res;
}

#endif  // UMA_NAVIGATION_WAYPOINT_ACTION_H
