#ifndef UMA_NAVIGATION_PATH_GEN_H
#define UMA_NAVIGATION_PATH_GEN_H
//  Copyright 2019 <UMAutonomy>
#include <uma_navigation/planner_input.h>
#include <uma_navigation/path.h>
#include <uma_navigation/planner_layer.h>
#include <math.h>
#include <vector>
#include <memory>

template <class T>
struct MapCell
{
  MapCell()
    : x(0), y(0) {}

  MapCell(T _x, T _y)
      : x(_x), y(_y) {}

  bool operator!=(const MapCell &other) const
  {
    return (x != other.x || y != other.y);
  }

  bool operator==(const MapCell &other) const
  {
    return (x == other.x && y == other.y);
  }

  T x, y;
};

using GridCell = MapCell<int>;

// runs the Theta* algorithm to find a path from startCell to goalCell
void thetaStarSearch(Path *path, const costmap_2d::Costmap2D *map,
  const GridCell &start_cell, const Waypoint &destination);

#endif  // UMA_NAVIGATION_PATH_GEN_H
