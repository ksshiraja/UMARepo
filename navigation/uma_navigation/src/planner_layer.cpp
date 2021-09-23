// Copyright 2019 UMAutonomy
#include "uma_navigation/planner_layer.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <memory>
#include <vector>

// TODO(lajohnst): use PLUGINLIB to add layers

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using costmap_2d::INSCRIBED_INFLATED_OBSTACLE;

PlannerLayer::PlannerLayer() {}

void PlannerLayer::updateCosts(costmap_2d::Costmap2D& master_grid,  // NOLINT(runtime/references)
    int min_i, int min_j, int max_i, int max_j)
{
  double update_radius;
  ros::param::param<double>("/update_costs_radius", update_radius, 3);
  horizonDistance = update_radius / master_grid.getResolution();
  std::vector<IntPoint> boxes = getBoxesToWeigh(min_i, min_j, max_i, max_j);
  updateDistanceCosts(master_grid, min_i, min_j, max_i, max_j, boxes);
  updateMovementModelCosts(master_grid, min_i, min_j, max_i, max_j, boxes);
}


void PlannerLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;
  default_value_ = NO_INFORMATION;
  matchSize();

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &PlannerLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

void PlannerLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
    double *min_x, double *min_y, double *max_x, double *max_y)
{
  boatAngle = robot_yaw;
}

void PlannerLayer::matchSize()
{
  Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
}


void PlannerLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void PlannerLayer::updateDistanceCosts(costmap_2d::Costmap2D& master_grid,  // NOLINT(runtime/references)
    int min_i, int min_j, int max_i, int max_j, const std::vector<IntPoint> &boxes)
{
  // radius around each object that will be weighted
  int radius = 0.5/master_grid.getResolution();
  // loop through the boxes
  for (const IntPoint &point : boxes)
  {
    // only does weighing if the box has LETHAL weight
    if (master_grid.getCost(point.x, point.y) != LETHAL_OBSTACLE)
      continue;
    // goes through the boxes that are a radius away but within the max and mins
    // uper and lower bounds of square around current box
    int lx = std::max(min_i, (point.x)-radius);
    int ux = std::min(max_i, (point.x)+radius+1);
    int ly = std::max(min_j, (point.y)-radius);
    int uy = std::min(max_j, point.y+radius+1);
    for (int sx = lx; sx < ux; sx++)
    {
      for (int sy = ly; sy < uy; sy++)
      {
        // if box is unkown or LETHAL then does not change weight
        if (master_grid.getCost(sx, sy) >= INSCRIBED_INFLATED_OBSTACLE)
          continue;

        // finds which box ring away a point is from the current coordinate working on
        double dist = std::max(abs(point.x-sx), abs(point.y-sy));

        int val = LETHAL_OBSTACLE*dist/(1+radius)+master_grid.getCost(sx, sy);
        // max value of a box without an object should be 252
        val = val >= INSCRIBED_INFLATED_OBSTACLE ? INSCRIBED_INFLATED_OBSTACLE - 1 : val;

        master_grid.setCost(sx, sy, val);
      }
    }
  }
}

// Makes cells in the opposite direction to where the boat is facing weigh more
// assumes that 0 <= boatAngle <= 2Pi
// boat angle 0 at positive x axis(East)
void PlannerLayer::updateMovementModelCosts(costmap_2d::Costmap2D& master_grid,  // NOLINT(runtime/references)
    int min_i, int min_j, int max_i, int max_j, const std::vector<IntPoint> &boxes)
{
// get the boxes that need to be weighted
  for (unsigned box = 0; box < boxes.size(); ++box)  // loop through every grid cell from getBoxesToWeigh function
  {
    if (master_grid.getCost(boxes.at(box).x, boxes.at(box).y) == NO_INFORMATION || master_grid.getCost(boxes.at(box).x,
        boxes.at(box).y) == costmap_2d::LETHAL_OBSTACLE)  // skips over already lethal or no information boxes
        continue;

    double angleBetween = M_PI/2;
    if (boxes.at(box).x != (max_i-min_i)/2)  // If x value is zero, sets angle to pi/2 to avoid undefined values
    {
        angleBetween = fabs(atan2(static_cast<double>(boxes.at(box).y-((max_j-min_j-1)/2.0)),
            static_cast<double>(boxes.at(box).x-((max_i-min_i)/2))) - boatAngle);
    }
    // how far the weight of the box is from 253
    double distFromLethal = costmap_2d::LETHAL_OBSTACLE - master_grid.getCost(boxes.at(box).x, boxes.at(box).y)-1;

// adds more weight for boxes behind the boat and does not add more than distFromLethal
    master_grid.setCost(boxes.at(box).x, boxes.at(box).y, master_grid.getCost(boxes.at(box).x, boxes.at(box).y) +
        std::min(std::round(75 * (sin(angleBetween/2-std::min(M_PI/8, angleBetween/2)))), distFromLethal));
  }
}



// returns a vector of four Point structs in order of front port, back port, back starboard, front starboard
// assumes that 0 <= boatAngle <= 2Pi
// boat angle 0 at positive x axis(East)
std::vector<DoublePoint> PlannerLayer::getCornerPoints()
{
    // the DoublePoints in adjustedPoints are going to be the points of the square in which the boat weighs boxes
    std::vector<DoublePoint> adjustedPoints;

    // x value of the point on the square to the front left from the boat's point of view
    double frontPortX = horizonDistance * sqrt(2) * cos(boatAngle + M_PI/4);

    // y value of the point on the square to the front left from the boat's point of view
    double frontPortY = horizonDistance * sqrt(2) * sin(boatAngle + M_PI/4);

    DoublePoint frontPortPoint = {frontPortX, frontPortY};  // creates a DoublePoint object for the front left point
    adjustedPoints.push_back(frontPortPoint);  // adds the corner point to the adjustedPoints list

    DoublePoint backPortPoint = {-frontPortY, frontPortX};
    adjustedPoints.push_back(backPortPoint);

    DoublePoint backStarboardPoint = {-frontPortX, -frontPortY};
    adjustedPoints.push_back(backStarboardPoint);

    DoublePoint frontStarboardPoint = {frontPortY, -frontPortX};
    adjustedPoints.push_back(frontStarboardPoint);

    return adjustedPoints;
}

// returns a vector of boxes that are within the horizon distance of the boat
// assumes that 0 <= boatAngle <= 2Pi
// boat angle 0 at positive x axis(East)
std::vector<IntPoint> PlannerLayer::getBoxesToWeigh(int min_x, int min_y, int max_x, int max_y)
{
    std::vector<IntPoint> pointsWithinSquare;
    if (fabs(remainder(boatAngle, M_PI/2)) < .01)  // if box is basically in line with grid, returns square of boxes
    {
        for (int x = -horizonDistance; x <= horizonDistance; ++x)
        {
            for (int y = -horizonDistance; y <= horizonDistance; ++y)
            {
                // all IntPoints created add (max-min)/2 because the function assumes the boat is at (0, 0),
                // but then must be adjusted to the actual local map
                int x_val = x + (max_x - min_x) / 2;
                int y_val = y + (max_y - min_y) / 2;

                if (x_val < min_x || x_val > max_x || y_val < min_y || y_val > max_y) continue;
                IntPoint point = {x_val, y_val};
                pointsWithinSquare.push_back(point);
            }
        }
        return pointsWithinSquare;
    }
    double slopeParallelOrientation = tan(boatAngle);  // slope of the line on the square with same angle as boatAngle
    double slopePerpOrientation = -1/slopeParallelOrientation;  // slope of line on square
    double slopeTopLeftSide = 0;  // value will be set to the slope of the line of the square in the top left

    // at different angles, the top left line of the square will have different positions in relation to the boat
    if (tan(boatAngle) < 0)
    {
        slopeTopLeftSide = slopePerpOrientation;
    }
    else
    {
        slopeTopLeftSide = slopeParallelOrientation;
    }
    double slopeBottomLeftSide = -1/slopeTopLeftSide;
    std::vector<DoublePoint> corners = getCornerPoints();  // find corner points of box
    double smallestX = corners.at(0).x;  // initially set smallestX to the first corner's x
    int indexOfLeftmost = 0;

    // iterate through 3 remaining points to find smallest X value and therefore the leftmost point index
    for (int i = 1; i < corners.size(); ++i)
    {
        if (corners.at(i).x < smallestX)
        {
            smallestX = corners.at(i).x;
            indexOfLeftmost = i;
        }
    }
    int indexOfBottommost = (indexOfLeftmost + 1)%4;  // find index of points in each of the following locations
    int indexOfRightmost = (indexOfBottommost + 1)%4;
    int indexOfUpmost = (indexOfRightmost + 1)%4;

    // size of box to test if within square, max distance from the origin that the tilted square is
    int testingSize = round(std::max(abs(corners.at(0).x), abs(corners.at(0).y)));

    double xPointOfTopHit = corners.at(indexOfUpmost).x;  // find x value of the point with the max y value
    double xPointOfBottomHit = -xPointOfTopHit;  // find x value of the point with the min y value
    for (int x = -testingSize; x <= testingSize; ++x)
    {
        for (int y = -testingSize; y <= testingSize; ++y)
        {
            double topYBound;  // given the box's x value, this will be set to the maximum y value it
            // can have to be within the tilted square

            double bottomYBound;  // same as topYBound but minimum y value
            if (xPointOfTopHit > x)  // box is to the left of the x value where the square reaches a maximum y value
            {
                topYBound = corners.at(indexOfLeftmost).y + slopeTopLeftSide * (x-corners.at(indexOfLeftmost).x);
            }
            else  // box is to the right of the x value where the square reaches a maximum y value
            {
                topYBound = corners.at(indexOfUpmost).y + slopeBottomLeftSide * (x - xPointOfTopHit);
            }
            if (xPointOfBottomHit > x)  // box is to the left of the x value where the square reaches a minimum y value
            {
                bottomYBound = corners.at(indexOfLeftmost).y + slopeBottomLeftSide * (x-corners.at(indexOfLeftmost).x);
            }
            else  // box is to the right of the x value where the square reaches a minimum y value
            {
                bottomYBound = corners.at(indexOfBottommost).y + slopeTopLeftSide * (x - xPointOfBottomHit);
            }
            if (y > bottomYBound && y < topYBound)  // y value of box is within range that the tilted box covers
            {
                IntPoint point = {x + (max_x-min_x)/2, y + (max_y-min_y)/2};
                if (point.x < min_x || point.x > max_x || point.y < min_y || point.y > max_y) continue;
                pointsWithinSquare.push_back(point);
            }
        }
    }
    return pointsWithinSquare;
}

