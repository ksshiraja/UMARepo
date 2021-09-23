#include "uma_navigation/costmap_handler.h"
#include <sstream>
#include <iostream>

#include <uma_navigation/planner_layer.h>
#include <uma_navigation/input_layer.h>

using std::make_shared;

CostmapHandler::CostmapHandler(ros::NodeHandle *nh, tf::TransformListener *tf, bool debug):
    tf_(tf),
    costmap_("map", true, true),
    costmap_input_layer_(new InputLayer()),
    costmap_planner_layer_(new PlannerLayer()),
    debug_(debug)
{
    if (debug_)
        publisher_ = boost::shared_ptr<costmap_2d::Costmap2DPublisher>(new costmap_2d::Costmap2DPublisher(nh,
                costmap_.getCostmap(), "map", "planner_costmap", true));
    costmap_.addPlugin(costmap_input_layer_);
    costmap_.addPlugin(costmap_planner_layer_);
}

void CostmapHandler::update(const Orientation &pose)
{
    costmap_.updateMap(pose.x, pose.y, pose.orientationAngle);
    if (debug_)
        publisher_->publishCostmap();
}

void CostmapHandler::init()
{
    costmap_input_layer_->initialize(&costmap_, "map/static", tf_);
    costmap_planner_layer_->initialize(&costmap_, "map/planner", tf_);

    auto size = costmap_input_layer_->getFootprint();
    auto size2 = costmap_planner_layer_->getFootprint();

    auto plugins = costmap_.getPlugins();
    int width = costmap_input_layer_->getWidth();
    int height = costmap_input_layer_->getHeight();
    double res = costmap_input_layer_->getResolution();
    costmap_.resizeMap(width, height, res, 0, 0);
}

void CostmapHandler::start()
{
    costmap_input_layer_->activate();
    costmap_planner_layer_->activate();
}

void CostmapHandler::stop()
{
    costmap_input_layer_->deactivate();
    costmap_planner_layer_->deactivate();
}

void CostmapHandler::printCostmap()
{
    std::stringstream ss;
    for (int y = costmap_.getCostmap()->getSizeInCellsY() - 1; y >= 0; y--)
    {
        for (unsigned int x = 0; x < costmap_.getCostmap()->getSizeInCellsX(); x++)
        {
            ss << (unsigned int)costmap_.getCostmap()->getCost(x, y) << ' ';
        }
        ss << std::endl;
    }
    std::cout << ss.str() << std::endl;
}
