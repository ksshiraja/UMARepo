#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>

class PathVisualizer
{
public:
    PathVisualizer()
    {
        pose_array_sub_ = nh_.subscribe("path_waypoints", 1000, &PathVisualizer::pathCallback, this);

        markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("path_visualizer", 100);
    }

    void pathCallback(const geometry_msgs::PoseArray &poses)
    {
        visualization_msgs::MarkerArray path_arrows;

        for (auto pose : poses.poses)
        {
            visualization_msgs::Marker path_arrow;
            path_arrow.header.frame_id = "map";
            path_arrow.header.stamp = ros::Time::now();
            path_arrow.ns = "path_arrows";
            path_arrow.action = visualization_msgs::Marker::ADD;
            path_arrow.pose = pose;

            path_arrow.id = 0;

            path_arrow.type = visualization_msgs::Marker::ARROW;

            path_arrow.lifetime = ros::Duration(1.0);

            path_arrow.scale.x = 1;
            path_arrow.scale.y = 0.3;
            path_arrow.scale.z = 0.1;

            // Alpha channel is 1
            path_arrow.color.a = 1.0;

            // Assign a random color
            path_arrow.color.r = 1;
            path_arrow.color.g = 0;
            path_arrow.color.b = 1;

            path_arrows.markers.push_back(path_arrow);
        }

        markers_pub_.publish(path_arrows);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher markers_pub_;
    ros::Subscriber pose_array_sub_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    PathVisualizer path_visualizer;

    ros::spin();

    return 0;
}
