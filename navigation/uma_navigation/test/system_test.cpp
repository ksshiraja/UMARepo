#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/server/action_server.h>
#include <actionlib/server/server_goal_handle.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/action_client.h>
#include <actionlib/client/terminal_state.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>

#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <iostream>
#include <memory>
#include <vector>

#include <uma_navigation/planner_input.h>
#include <uma_navigation/waypoint_action.h>
#include <uma_navigation/waypoints_action.h>
#include <costmap_2d/cost_values.h>
#include <uma_navigation/path_gen.h>

using std::string;
using costmap_2d::Costmap2D;
using std::thread;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using std::shared_ptr;

using std::cout;
using std::endl;

std::atomic<bool> run(true);
std::mutex poseMutex;
nav_msgs::Odometry pose;

enum CostmapType
{
    LONG_DOCK,
    SHORT_DOCK,
    BUOY_FIELD
};

Costmap2D global_costmap(100, 100, 1.0, 0, 0);

void printCostmap(Costmap2D *costmap)
{
    for (int y = costmap->getSizeInCellsY() - 1; y >= 0; --y)
    {
        for (int x = 0; x < costmap->getSizeInCellsX(); ++x)
        {
            cout << static_cast<int>(costmap->getCost(x, y)) << '\t';
        }
        cout << '\n';
    }

    cout << endl;
}

void update_with_global_value(Costmap2D *local, Costmap2D *global)
{
    MapCell<unsigned int> ll_local_in_global;

    double origin_X_world = local->getOriginX();
    double origin_Y_world = local->getOriginY();

    double origin_X_global = global->getOriginX();
    double origin_Y_global = global->getOriginY();

    unsigned int origin_X_global_map;
    unsigned int origin_Y_global_map;

    global->worldToMap(origin_X_world, origin_Y_world, origin_X_global_map, origin_Y_global_map);

    ll_local_in_global.x = origin_X_global_map;
    ll_local_in_global.y = origin_Y_global_map;

    for (int x = 0; x < local->getSizeInCellsX(); ++x)
    {
        for (int y = 0; y < local->getSizeInCellsY(); ++y)
        {
            unsigned int x_map;
            unsigned int y_map;

            if (global->worldToMap(origin_X_world + x, origin_Y_world + y, x_map, y_map))
            {
                local->setCost(x, y, global->getCost(x_map, y_map));
            }
            else
            {
                local->setCost(x, y, NO_INFORMATION);
            }
        }
    }

    // cout << "Global: " << endl;
    // printCostmap(global);
    // cout << endl;
    // cout << "Local: " << endl;
    // printCostmap(local);
}

void set_pose(double x, double y, double yaw)
{
    tf2::Matrix3x3 m;
    m.setRPY(0, 0, yaw);
    tf2::Quaternion q;
    m.getRotation(q);

    std::lock_guard<std::mutex> lock(poseMutex);
    pose.pose.pose.position.x = x;
    pose.pose.pose.position.y = y;
    pose.pose.pose.orientation = tf2::toMsg(q);
}

template <class WaypointType>
void set_pose(const WaypointType &w)
{
    set_pose(w.x, w.y, w.yaw);
}

// void publish_pose()
// {
//     ros::NodeHandle nh;
//     ros::Publisher locPub = nh.advertise<nav_msgs::Odometry>("/odometry/filtered", 10);
//     ros::Rate r(10);
//     while (run)
//     {
//         std::lock_guard<std::mutex> lock(poseMutex);
//         locPub.publish(pose);
//         ros::spinOnce();
//         r.sleep();
//     }
// }

// sets up and runs action server to communicate with task planning
class ControlsActionServer
{
public:
    /*
     * @param name - name to register with the action server
     * @param nh - node handle of running planner
     * @param run_planner - callback that is called when a new goal is received
     * @param stop_planner - callback that is called when a goal is cancelled to allow planner to stop and cleanup
     * @param set_target - callback that sets the planner's target and is called when a new goal is received
     */ 
    ControlsActionServer(const std::string &name, ros::NodeHandle *nh,
                costmap_2d::Costmap2DPublisher *map_pub_in, costmap_2d::Costmap2D *costmap_in):
        node_handle_(nh),
        action_server_(*nh, name, false),
        action_name_(name),
        odom_pub(nh->advertise<nav_msgs::Odometry>("/odometry/filtered", 1000)),
        map_pub(map_pub_in),
        costmap(costmap_in)
    {
        // setup and start action server
        action_server_.registerGoalCallback(boost::bind(&ControlsActionServer::goalCB, this, _1));
        // TODO(will): a real server would probably want to do something at cancel
        // action_server_.registerCancelCallback(boost::bind(&ControlsActionServer::preemptCB, this));
        action_server_.start();
    }

private:
    // called by the action server when a new goal is received
    void goalCB(actionlib::ServerGoalHandle<WaypointsAction> goalHandle)
    {
        if (!goalHandleVector.empty())
        {
            auto prev_goal = goalHandleVector.back().getGoal();
            std::cout << "prevGoal start point: (" << prev_goal->x.front() << "," << prev_goal->x.front() << ")\n";
            std::cout << "prevGoal end point: (" << prev_goal->x.back() << "," << prev_goal->x.back() << ")\n";
            goalHandleVector.back().setCanceled();
        }
        else
        {
            std::cout << "this bitch empty, yeet\n";
        }
        int currentGoalIndex = goalHandleVector.size();
        goalHandleVector.push_back(goalHandle);
        auto goal = goalHandleVector.back().getGoal();
        goalHandleVector.back().setAccepted();
        std::cout << "goal start point: (" << goal->x.front() << "," << goal->x.front() << ")\n";
        std::cout << "goal end point: (" << goal->x.back() << "," << goal->x.back() << ")\n";

        Target t;
        // for (int i = 0; i< goal->x.size(); ++i)
        // {
        //     cout << "x: " << goal->x[i] << ", y: " << goal->y[i] << endl;
        // }
        t.x = goal->x.back();
        t.y = goal->y.back();
        t.yaw = goal->end_yaw;
        t.error = 0;
        t.error_yaw = 0;

        // helper variables
        ros::Rate r(10);
        bool success = true;

        // push_back the seeds for the fibonacci sequence
        feedback_.x = 0;
        feedback_.y = 0;
        feedback_.yaw = 0;

        static double x = 0;
        static double y = 0;
        static double yaw = 0;

        // publish info to the console for the user
        ROS_INFO("%s: Executing, heading to waypoint (%f, %f) with yaw = %f, error %f",
                action_name_.c_str(), goal->x.back(), goal->y.back(), goal->end_yaw, goal->end_pos_error);

        // start executing the action
        while (sqrt(pow(x - goal->x.back(), 2) + pow(y - goal->y.back(), 2)) > goal->end_pos_error
                || fabs(yaw - goal->end_yaw) > goal->end_pos_error)
        {
            // check that goal is still active (preempt/cancel hasn't been sent)
            std::cout << "currentIndex: " << currentGoalIndex << ", goalStatus: "
                    <<static_cast<unsigned>(goalHandleVector[currentGoalIndex].getGoalStatus().status) << "\n";
            if (goalHandleVector[currentGoalIndex].getGoalStatus().status != 1 || !ros::ok())
            {
                ROS_INFO("%s: Preempted", action_name_.c_str());
                success = false;
                break;
            }

            // move towards target
            if (x < goal->x.back()) x += 0.1;
            else if (x > goal->x.back()) x -= 0.1;

            if (y < goal->y.back()) y += 0.1;
            else if (y > goal->y.back()) y -= 0.1;

            if (yaw < goal->end_yaw) yaw += 0.1;
            else if (yaw > goal->end_yaw) yaw -= 0.1;

            feedback_.x = x;
            feedback_.y = y;
            feedback_.yaw = yaw;
            set_pose(feedback_);

            nav_msgs::Odometry msg;
            msg.pose.pose.position.x = x;
            msg.pose.pose.position.y = y;
            msg.pose.pose.position.z = 0;

            odom_pub.publish(pose);
            cout << "Boat at: (" << x << ", " << y << ")" << endl;
            costmap->resizeMap(costmap->getSizeInCellsX(), costmap->getSizeInCellsY(),
                costmap->getResolution(), x - 4, y - 4);
            update_with_global_value(costmap, &global_costmap);
            map_pub->publishCostmap();
            // printCostmap(costmap);

            // publish the feedback
            goalHandleVector.back().publishFeedback(feedback_);
            // this sleep is not necessary, the sequence is computed at 5 Hz for demonstration purposes
            r.sleep();
            ros::spinOnce();
        }

        if (success)
        {
            // set the action state to succeeded

            nav_msgs::Odometry msg;
            msg.pose.pose.position.x = x;
            msg.pose.pose.position.y = y;
            msg.pose.pose.position.z = 0;

            odom_pub.publish(pose);
            costmap->resizeMap(costmap->getSizeInCellsX(), costmap->getSizeInCellsY(),
                            costmap->getResolution(), x - 4, y - 4);
            update_with_global_value(costmap, &global_costmap);
            map_pub->publishCostmap();
            // printCostmap(costmap);

            result_ = waypointFrom<WaypointsResult, WaypointsFeedback>(feedback_);
            ROS_INFO("%s: Succeeded", action_name_.c_str());
            // set the action state to succeeded
            goalHandleVector.back().setSucceeded(result_);
            set_pose(result_);

            ros::spinOnce();
        }
    }

    ros::NodeHandle *node_handle_;  // pointer to current node handle
    std::vector<actionlib::ServerGoalHandle<WaypointsAction>> goalHandleVector;
    // controls server
    actionlib::ActionServer<WaypointsAction> action_server_;
    std::string action_name_;

    ros::Publisher odom_pub;
    costmap_2d::Costmap2DPublisher *map_pub;
    costmap_2d::Costmap2D *costmap;

    // action server feedback and result
    WaypointsFeedback feedback_;
    WaypointsResult result_;
};


void sendAction(actionlib::SimpleActionClient<uma_navigation_external_interface::WaypointAction> *ac,
    uma_navigation_external_interface::WaypointGoal goal)
{
    ac->sendGoal(goal);

    // wait for the action to return
    bool finished_before_timeout = ac->waitForResult();

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac->getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
    }
    else
        ROS_INFO("Action did not finish before the time out.");
}

void run_controls(ros::NodeHandle *nh, costmap_2d::Costmap2DPublisher *costmap_pub,
    shared_ptr<costmap_2d::Costmap2D> costmap)
{
    ControlsActionServer controls("/navigation/path", nh, costmap_pub, costmap.get());
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::waitForShutdown();
}

void initialize_global_costmap(CostmapType type)
{
    switch (type)
    {
        case CostmapType::LONG_DOCK:
            // global_costmap.setCost(6, 0, LETHAL_OBSTACLE);
            // global_costmap.setCost(6, 1, LETHAL_OBSTACLE);
            // global_costmap.setCost(6, 2, LETHAL_OBSTACLE);
            // global_costmap.setCost(6, 3, LETHAL_OBSTACLE);
            // global_costmap.setCost(6, 4, LETHAL_OBSTACLE);
            // global_costmap.setCost(6, 5, LETHAL_OBSTACLE);
            // global_costmap.setCost(6, 6, LETHAL_OBSTACLE);
            // global_costmap.setCost(6, 7, LETHAL_OBSTACLE);
            // global_costmap.setCost(6, 8, LETHAL_OBSTACLE);
            // global_costmap.setCost(6, 9, LETHAL_OBSTACLE);
            break;
        case CostmapType::SHORT_DOCK:
            // global_costmap.setCost(6, 4, LETHAL_OBSTACLE);
            // global_costmap.setCost(6, 5, LETHAL_OBSTACLE);
            // global_costmap.setCost(6, 6, LETHAL_OBSTACLE);
            // global_costmap.setCost(6, 7, LETHAL_OBSTACLE);
            // global_costmap.setCost(6, 8, LETHAL_OBSTACLE);
            // global_costmap.setCost(6, 9, LETHAL_OBSTACLE);
            break;
        case CostmapType::BUOY_FIELD:
            global_costmap.setCost(10, 10, LETHAL_OBSTACLE);
            global_costmap.setCost(11, 10, LETHAL_OBSTACLE);
            global_costmap.setCost(15, 13, LETHAL_OBSTACLE);
            global_costmap.setCost(16, 11, LETHAL_OBSTACLE);
            global_costmap.setCost(16, 18, LETHAL_OBSTACLE);
            global_costmap.setCost(18, 16, LETHAL_OBSTACLE);
            global_costmap.setCost(18, 15, LETHAL_OBSTACLE);
            global_costmap.setCost(25, 22, LETHAL_OBSTACLE);
            global_costmap.setCost(28, 18, LETHAL_OBSTACLE);
            global_costmap.setCost(24, 20, LETHAL_OBSTACLE);
            global_costmap.setCost(25, 20, LETHAL_OBSTACLE);
            break;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_server");
    ros::NodeHandle nh("~");

    // initialize costmap
    shared_ptr<Costmap2D> costmap(new Costmap2D(20, 20, 1.0, 0, 0));
    costmap_2d::Costmap2DPublisher costmap_pub = costmap_2d::Costmap2DPublisher(&nh,
                                        costmap.get(), "map", "/costmaps/local_costmap/costmap", true);

    initialize_global_costmap(CostmapType::BUOY_FIELD);
    update_with_global_value(costmap.get(), &global_costmap);

    thread t(run_controls, &nh, &costmap_pub, costmap);
    t.detach();

    costmap_pub.publishCostmap();

    // start sending pose
    set_pose(0, 0, 0);
    ros::Publisher odom_pub(nh.advertise<nav_msgs::Odometry>("/odometry/filtered", 1000));
    odom_pub.publish(pose);

    ros::spinOnce();

    // Task planning action client - sends waypoint goals
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<uma_navigation_external_interface::WaypointAction> ac("path_planning", true);

    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    ac.waitForServer();  // will wait for infinite time

    ROS_INFO("Action server started, sending goal.");

    // send a goal to the action
    uma_navigation_external_interface::WaypointGoal goal;

    goal.x = 22;
    goal.y = 15;
    goal.yaw = -2;
    goal.error = 0.25;
    goal.error_yaw = 0.5;
    sendAction(&ac, goal);
    cout << "Made it through test!";

    return 0;
}
