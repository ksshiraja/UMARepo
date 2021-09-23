#ifndef UMA_CONTROLS_INDOOR_POSE_H
#define UMA_CONTROLS_INDOOR_POSE_H

#include <mutex>
#include <string>
#include <boost/thread/thread.hpp>

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "marvelmind_nav/hedge_pos_a.h"

class IndoorPose
{
public:
    IndoorPose(ros::NodeHandle *nh, int hedgehog_id, int publish_rate);
private:
    void indoorGpsCallback(const marvelmind_nav::hedge_pos_a &hedge_pos_msg);
    void publishPose();

    // Thr multi eading
    std::mutex mutex_;

    // State
    int hedgehog_id_;
    int publish_rate_;
    geometry_msgs::Point pos_;

    ros::Subscriber indoor_gps_pub_;
    ros::Publisher pose_pub_;
};

#endif  // UMA_CONTROLS_INDOOR_POSE_H
