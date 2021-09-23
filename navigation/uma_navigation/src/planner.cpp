#include <ros/ros.h>
#include <uma_navigation_external_interface/WaypointAction.h>

#include <atomic>
#include <iostream>
#include <math.h>
#include <mutex>
#include <thread>
#include <future>
#include <string>
#include <vector>
#include <utility>

#include <boost/program_options.hpp>
#include <boost/thread/barrier.hpp>

#include <lib/parameters/params.h>
#include <uma_navigation/task_server.h>
#include <uma_navigation/control_client.h>
#include <uma_navigation/path_planner.h>

using std::string;

// handler to interface with control client on across threads
// the sending and waiting for waypoints is done on a separate thread
// this is so the changes to path validity can be monitored and the costmap can be updated at the same time
class PathHandler
{
public:
    PathHandler(ControlActionClient *control_client, const Target &goal):
        run_(true),
        control_client_(control_client),
        path_received_(false),
        set_path_result_(true),
        process_path_thread_(&PathHandler::processPath, this,  goal)
    {
    }

    ~PathHandler()
    {
        {
            std::lock_guard<std::mutex> lock(m_);
            run_ = false;
            target_cv_.notify_one();
        }
        process_path_thread_.join();
    }

    void setTargetPath(const std::vector<Coordinate<double> >* spline_path_in)
    {
        ROS_INFO("[PATH_PLANNER] Setting target path");
        std::lock_guard<std::mutex> lock(m_);
        spline_path_ = spline_path_in;
        path_received_ = true;
        target_cv_.notify_one();
    }

    bool targetReached() const
    {
        return !path_received_;
    }

    bool getResult()
    {
        std::lock_guard<std::mutex> lock(m_);
        return set_path_result_;
    }

private:
    void processPath(const Target &goal)
    {
        std::unique_lock<std::mutex> lock(m_);
        while (run_)
        {
            while (run_ && !path_received_)
                target_cv_.wait(lock);

            if (!run_)
                break;

            // send waypoint to controls and get result
            set_path_result_ = control_client_->sendPathToControls(spline_path_, goal);
            path_received_ = false;
        }
    }

    std::atomic<bool> run_;
    std::mutex m_;
    std::condition_variable target_cv_;
    ControlActionClient *control_client_;  // interface with control client
    bool path_received_;  // whether a target is being worked on
    bool set_path_result_;  // whether last waypoint was successful or not
    const std::vector<Coordinate<double> >* spline_path_;
    // next path to send to controls (not necessarily what is currently being worked on)
    std::thread process_path_thread_;  // thread that takes next waypoints and send it to controls
};

// Handle running the entire path planner system
class PlannerHandler
{
public:
    /*
     * @param nh - pointer to node handle of planner
     */
    explicit PlannerHandler(ros::NodeHandle *nh, string controls_channel, bool debug = false):
        nh_(nh),
        path_planner_(nh, debug),
        task_server_(ros::this_node::getName(), nh,
                boost::bind(&PlannerHandler::run_planner, this),
                boost::bind(&PlannerHandler::stop_planner, this),
                boost::bind(&PathPlanner::setTarget, &path_planner_, _1)),
        control_client_(controls_channel, nh),
        run_(false),
        stop_barrier_(2),
        debug_(debug)
    {}

    ~PlannerHandler()
    {
        control_client_.stop();
    }

    /*
     * Set the target waypoint
     * Will montior path validity and running state and will cancel the waypoint if either of those becomes false
     * @param handler - waypoint handler to use
     * @param target_waypoint - next waypoint in path to send
     * @param path - Current path being worked on. Used to make sure path is valid.
     * @return bool - the waypoint was successfully reached and the path is still valid
     */
    bool setPathAndCheckValid(PathHandler *path_handler, const std::vector<Coordinate<double> >* spline_path)
    {
        // send waypoint to controls
        path_handler->setTargetPath(spline_path);
        bool path_valid = true;
        ros::Rate rate(3);
        // while waiting, montitor the running state of the planner and the path validity
        while (run_ && path_valid && !path_handler->targetReached())
        {
            // cancel the current spline array and restart the planner if the path is not valid
            // checking path validity also updates the costmap in the background
            // this is to ensure we're checking against the latest one
            if (ignore_obstacles_ || !(path_valid = path_planner_.isSplinePathValid()))
            {
                // if we realize the path is invalid, cancel it and make a new one immediatly
                control_client_.cancel();
                return false;
            }
            rate.sleep();
        }
        return path_handler->getResult();
    }

    /*
     * Run an iteration of the planner and send the path to controls
     */
    void run_planner()
    {
        // start set to running and start control client
        run_ = true;
        control_client_.start();  // Block until the control server is started

        // get target and initialize waypoint handler
        Target target = path_planner_.getTarget();
        PathHandler path_handler(&control_client_, target);

        bool target_reached = false;
        Waypoint w;
        ROS_INFO("[PATH_PLANNER] Starting path planner...");
        const std::vector<Coordinate<double> >* spline_path;

        while (ros::ok() && !target_reached && run_)
        {
            // create a path within the current costmap
            ROS_INFO("[PATH_PLANNER] Creating path...");
            // generate path and check if valid
            std::pair<bool, bool> run_return = path_planner_.run();
            ignore_obstacles_ = run_return.second;
            if (!run_return.first)
            {
                // something went wrong so try again
                ROS_WARN("[PATH_PLANNER] Failed to create valid path! Trying again...");
                ros::Duration(3).sleep();
                continue;
            }
            // send created path to controls
            ROS_INFO("[PATH_PLANNER] Path created! Sending to controls...");

            spline_path = path_planner_.getSplinePath();

            // check if last coords in array are the destination
            bool target_in_range = false;

            Waypoint w(spline_path->back().x, spline_path->back().y, target.yaw);
            target_in_range = waypointWithinBounds(target, w, target.error, target.error_yaw);

            if (target_in_range)
                ROS_INFO("[PATH_PLANNER] Target is in costmap range");
            else
                ROS_INFO("[PATH_PLANNER] Target is out of costmap range");

            if (!setPathAndCheckValid(&path_handler, spline_path))
            {   // if path is invalid or if controls doesn't make it to the final waypoint
                if (!run_)
                    // we're being told to stop
                    break;

                ROS_INFO("[PATH_PLANNER] Remaking path!");
                continue;
            }
            else
            {
                // Controls successfully made it to the target waypoint
                ROS_INFO("[PATH_PLANNER] MADE IT TO THE TARGET. SUCCESS");
                target_reached = true;
            }
        }
        ROS_INFO("[PATH_PLANNER] Planner spin finished!");
        bool still_running = run_.exchange(false);
        if (target_reached && still_running)
        {
            // we reached so notify of success
            ROS_INFO("[PATH_PLANNER] Target reached!");
            Waypoint w(spline_path->back().x, spline_path->back().y, target.yaw);
            task_server_.goalReached(w);
        }
        else if (still_running)
        {
            // abort task if we didn't succeed and weren't told to preempt
            ROS_INFO("[PATH_PLANNER] Task aborted!");
            Waypoint w(spline_path->back().x, spline_path->back().y, target.yaw);
            task_server_.abort(w);
        }
        if (!still_running)
        {
            // if we were told to stop from external call, signal you're done and wait
            ROS_INFO("[PATH_PLANNER] Stopping run_planner callback...");
            stop_barrier_.wait();
        }
    }

    void stop_planner()
    {
        bool still_running = run_.exchange(false);
        // stop control client to wake up waiting threads and so we stop sending waypoints
        control_client_.stop();
        // if still running, set to false and wait until done
        if (still_running)
        {
            ROS_INFO("[PATH_PLANNER] Stopping planner...");
            stop_barrier_.wait();
        }
    }

private:
    ros::NodeHandle *nh_;  // pointer to node handle of planner
    PathPlanner path_planner_;
    TaskActionServer task_server_;
    ControlActionClient control_client_;

    std::atomic<bool> run_;  // thread safe way to see if we're still running
    boost::barrier stop_barrier_;  // syncrhonize stopping accross multiple threads

    bool ignore_obstacles_;
    bool debug_;
};

int main(int argc, char *argv[])
{
    namespace po = boost::program_options;
    po::options_description options("Path Planner");
    options.add_options()
        ("help,h", "Print help message")
        ("debug,d", "Set to debug mode. Will publish and show additional information.")
        ("controls-topic,c", "Set which channel the path planner sends waypoints to the controls system on");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, options), vm);
    po::notify(vm);
    string controls_channel("/navigation/path");

    bool debug_mode = false;
    if (vm.count("help"))
    {
        std::cout << options << std::endl;
        return 1;
    }
    if (vm.count("debug"))
        debug_mode = true;
    if (vm.count("controls-topic"))
        controls_channel = vm["controls-topic"].as<string>();

    // init node
    ros::init(argc, argv, "path_planning");
    ros::NodeHandle nh;

    // create planner handler
    PlannerHandler planner_handler(&nh, controls_channel, debug_mode);

    // create async spinners to handler callbacks and run them
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
