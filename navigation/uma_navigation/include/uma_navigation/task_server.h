#ifndef UMA_NAVIGATION_TASK_SERVER_H
#define UMA_NAVIGATION_TASK_SERVER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <string>

#include <lib/errors/errors.h>
#include <uma_navigation/planner_input.h>
#include <uma_navigation/waypoint_action.h>
#include <uma_navigation/task_server.h>


// sets up and runs action server to communicate with task planning
class TaskActionServer
{
public:
    /*
     * @param name - name to register with the action server
     * @param nh - node handle of running planner
     * @param run_planner - callback that is called when a new goal is received
     * @param stop_planner - callback that is called when a goal is cancelled to allow planner to stop and cleanup
     * @param set_target - callback that sets the planner's target and is called when a new goal is received
     */ 
    TaskActionServer(const std::string &name, ros::NodeHandle *nh, boost::function<void()> run_planner,
            boost::function<void()> stop_planner, boost::function<void(const Target &)> set_target):
        node_handle_(nh),
        action_server_(*nh, name, boost::bind(&TaskActionServer::executeCB, this, _1), false),
        action_name_(name),
        run_planner_(run_planner),
        stop_planner_(stop_planner),
        set_target_(set_target)
    {
        // setup and start action server
        action_server_.registerPreemptCallback(boost::bind(&TaskActionServer::preemptCB, this));
        action_server_.start();
    }

    template <class TargetType>
    void goalReached(const TargetType &t)
    {
        WaypointResult result = waypointFrom<WaypointResult, TargetType>(t);
        action_server_.setSucceeded(result);
    }

    template <class TargetType>
    void abort(const TargetType &t)
    {
        WaypointResult result = waypointFrom<WaypointResult, TargetType>(t);
        action_server_.setAborted(result);
    }

    template <class TargetType>
    void goalFeedback(const TargetType &t)
    {
        WaypointFeedback feedback = waypointFrom<WaypointFeedback, TargetType>(t);
        action_server_.publishFeedback(feedback);
    }

private:
    // called by the action server when a new goal is received
    void executeCB(const WaypointGoalConstPtr &goal)
    {
        Target t = waypointFromWithGoal<Target, WaypointGoal>(*goal);
        set_target_(t);
        try
        {
            run_planner_();
        }
        catch (const uma::UMAException &e)
        {
            ROS_ERROR("%s", e.what());
            abort(t);
        }
    }

    // called by the action server when a goal is cancelled
    void preemptCB()
    {
        stop_planner_();
        action_server_.setPreempted();
    }

    ros::NodeHandle *node_handle_;  // pointer to current node handle
    actionlib::SimpleActionServer<WaypointAction> action_server_;
    std::string action_name_;

    // callbacks
    boost::function<void()> run_planner_;  // runs planner when new goal is received
    boost::function<void()> stop_planner_;  // stops planner when goal is cancelled
    boost::function<void(const Target &)> set_target_;  // sets target for planner
};

#endif  // UMA_NAVIGATION_TASK_SERVER_H
