#ifndef UMA_NAVIGATION_PLANNER_LAYER_H
#define UMA_NAVIGATION_PLANNER_LAYER_H
// Copyright 2019 UMAutonomy
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <iostream>
#include <sstream>
#include <vector>

struct IntPoint
{
    int x;
    int y;
};
struct DoublePoint
{
    double x;
    double y;
};

class PlannerLayer : public costmap_2d::Layer, public costmap_2d::Costmap2D
{
public:
    int horizonDistance = 3;
    double boatAngle = 0;  // same as yaw
    PlannerLayer();

    virtual void onInitialize();
    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y,
    double *max_x, double *max_y);

    virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j,  // NOLINT(runtime/references)
        int max_i, int max_j);
    virtual void updateDistanceCosts(costmap_2d::Costmap2D& master_grid,  // NOLINT(runtime/references)
        int min_i, int min_j, int max_i, int max_j, const std::vector<IntPoint> &boxes);
    virtual void updateMovementModelCosts(costmap_2d::Costmap2D& master_grid,  // NOLINT(runtime/references)
        int min_i, int min_j, int max_i, int max_j, const std::vector<IntPoint> &boxes);

    std::vector<DoublePoint> getCornerPoints();
    std::vector<IntPoint> getBoxesToWeigh(int min_x, int min_y, int max_x, int max_y);

    void matchSize();

private:
    void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);  // NOLINT(runtime/references)
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
};
#endif  // UMA_NAVIGATION_PLANNER_LAYER_H
