#ifndef UMA_NAVIGATION_COSTMAP_HANDLER_H
#define UMA_NAVIGATION_COSTMAP_HANDLER_H

#include <ros/ros.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <tf/transform_listener.h>

#include <lib/errors/errors.h>
#include <uma_navigation/planner_input.h>
#include <uma_navigation/input_layer.h>

#include <boost/shared_ptr.hpp>

class CostmapHandler
{
public:
    explicit CostmapHandler(ros::NodeHandle *nh, tf::TransformListener *tf, bool debug = true);
    void update(const Orientation &);
    costmap_2d::LayeredCostmap *getCostmap() { return &costmap_; }
    void init();
    void start();
    void stop();
    void printCostmap();
private:
    tf::TransformListener *tf_;
    costmap_2d::LayeredCostmap costmap_;
    boost::shared_ptr<InputLayer> costmap_input_layer_;
    boost::shared_ptr<costmap_2d::Layer> costmap_planner_layer_;
    boost::shared_ptr<costmap_2d::Costmap2DPublisher> publisher_;
    bool debug_;
};

#endif  // UMA_NAVIGATION_COSTMAP_HANDLER_H
