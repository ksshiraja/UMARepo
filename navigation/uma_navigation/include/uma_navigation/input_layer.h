#ifndef UMA_NAVIGATION_INPUT_LAYER_H
#define UMA_NAVIGATION_INPUT_LAYER_H

#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <message_filters/subscriber.h>
#include <dynamic_reconfigure/server.h>
#include <costmap_2d/GenericPluginConfig.h>

#include <string>

class InputLayer : public costmap_2d::CostmapLayer
{
public:
    InputLayer();
    virtual ~InputLayer();
    virtual void onInitialize();
    virtual void activate();
    virtual void deactivate();
    virtual void reset();
    virtual void updateBounds(double, double, double, double *, double *, double *, double *);
    virtual void updateCosts(costmap_2d::Costmap2D &master_grid, int, int, int, int);  // NOLINT(runtime/references)
    virtual void matchSize();

    unsigned int getWidth() const { return width; }
    unsigned int getHeight() const { return height; }
    double getResolution() const { return resolution_; }

private:
    void incomingMap(const nav_msgs::OccupancyGridConstPtr &);
    void incomingUpdate(const map_msgs::OccupancyGridUpdateConstPtr &);
    void reconfigureCB(costmap_2d::GenericPluginConfig &, uint32_t);
    unsigned char interpretValue(unsigned char);

    std::string global_frame_;
    std::string map_frame_;
    bool map_received_;
    bool has_updated_data_;
    bool subscribe_to_updates_;
    bool follow_bounds_;
    unsigned int x_, y_, width_, height_;
    ros::Subscriber map_sub_, map_update_sub_;
    unsigned char lethal_threshold_;
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;

    unsigned int width, height;
};

#endif  // UMA_NAVIGATION_INPUT_LAYER_H
