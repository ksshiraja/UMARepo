#ifndef UMA_NAVIGATION_CONTROL_CLIENT_H
#define UMA_NAVIGATION_CONTROL_CLIENT_H

#include <ros/ros.h>
#include <actionlib/client/action_client.h>
#include <actionlib/client/client_helpers.h>

#include <atomic>
#include <vector>
#include <string>

#include <boost/thread/barrier.hpp>

#include <uma_navigation/planner_input.h>
#include <uma_navigation/waypoint_action.h>
#include <uma_navigation/waypoints_action.h>

// sets up and runs action client to communicate with controls
class ControlActionClient
{
public:
    /*
     * @param name - name to register with the action client
     * @param nh - node handle of running planner
     */
    ControlActionClient(const std::string &name, ros::NodeHandle *nh):
        node_handle_(nh),
        action_client_(name),
        action_name_(name),
        run_(false)
    {}

    /*
     * Starts receiving and sending targets
     * Must be run on a separate thread from ros::spin
     */
    void start()
    {
        action_client_.waitForActionServerToStart();
        run_ = true;
    }

    /*
     * Cancel the paths sent to controls because they are invalid
     */
    void cancel()
    {
        action_client_.cancelAllGoals();
    }

    /*
     * Stop everything. Will cancel current paths and will stop receiving and sending targets.
     */
    void stop()
    {
        run_ = false;
        cancel();
    }

    /*
     * Sends target to controls and checks return state
     * @param spline_path - array of coordinates controls must go to
     * @param target - task planning target
     * @return bool - Returns true if waypoint reached within the specified error bounds
     */

    bool sendPathToControls(const std::vector<Coordinate<double> >* spline_path, const Target &target)
    {
        if (!run_)
            return false;
        // converts the spline and target information to the waypoints ROS message type
        WaypointsGoal goal = convertToWaypointsGoal(spline_path, target.yaw, target.error, target.error_yaw,
                                                    target.velocity, target.stop_command);

        // set timeout to 5 seconds, so if not arrived after 5 seconds, send another
        ROS_INFO("[PATH PLANNER] Sending path to controls");
        actionlib::ClientGoalHandle<WaypointsAction> goal_handle = action_client_.sendGoal(goal);
        // sleep for 5 seconds
        ros::Duration(5).sleep();
        auto comm_state = goal_handle.getCommState();

        // we might have been stopped while waiting
        if (!run_)
            return false;

        if (comm_state.toString() != "DONE")
        {
            ROS_INFO("[PATH PLANNER] Client timed-out before recieved result, sending another path");
            return false;
        }
        auto terminal_state = goal_handle.getTerminalState();

        if (terminal_state.toString() != "SUCCEEDED")
        {
            ROS_WARN("[PATH PLANNER] Controls failed to complete spline path with state %s.",
                     terminal_state.toString().c_str());
            return false;
        }

        ROS_INFO("[PATH PLANNER] Reached controls waypoint with state %s", terminal_state.toString().c_str());
        const WaypointsResultConstPtr &result = goal_handle.getResult();

        // check if arrived at task planning target
        Waypoint w(target.x, target.y, target.yaw);
        return waypointWithinBounds(w, *result, target.error, target.error_yaw, true);
    }
private:
    ros::NodeHandle *node_handle_;  // pointer to current node handle
    actionlib::ActionClient<WaypointsAction> action_client_;
    std::string action_name_;
    std::atomic<bool> run_;  // thread safe way to see if client is running or not
};

#endif  // UMA_NAVIGATION_CONTROL_CLIENT_H
