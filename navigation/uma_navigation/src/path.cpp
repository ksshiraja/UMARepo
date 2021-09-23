/*
 * Copyright 2019 UM::Autonomy
 */

#include "uma_navigation/path.h"
#include "uma_navigation/spline.h"

#include <iostream>
#include <cmath>
#include <costmap_2d/cost_values.h>
#include <ros/console.h>
#include <vector>
#include <deque>

using std::vector;
using std::deque;
using costmap_2d::Costmap2D;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::INSCRIBED_INFLATED_OBSTACLE;

// costmap value for an obstacle in the grid
const unsigned char CIRCUMSCRIBED_INFLATED_OBSTACLE = 128;
// constant for how close coordinates in spline path should be to each other
const double POINT_SEPERATION = .01;

double distanceToLine(const Waypoint &segA, const Waypoint &segB, const Waypoint &point)
{
    double numerator = abs((segB.y - segA.y) * point.x - (segB.x - segA.x) * point.y + segB.x * segA.y -
            segB.y * segA.x);
    double denominator = sqrt(pow(segB.y - segA.y, 2) + pow(segB.x - segA.x, 2));
    return numerator / denominator;
}

// Euclidean distance
double distanceTwoPoints(const double x1, const double y1, const double x2, const double y2)
{
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

// Deprecated: This was implemented when we used A* instead of theta*
// I don't see a use for it now, but it may be useful later
// this recursive method is used to simplify our path
// reference: https://en.wikipedia.org/wiki/Ramer%E2%80%93Douglas%E2%80%93Peucker_algorithm
std::deque<Waypoint> douglasPeuckerMethod(const std::deque<Waypoint> &points, double epsilon)
{
    // Find the point with the maximum distance
    double d_max = 0.0;
    size_t index = 0;

    for (size_t idx = 0; idx < points.size(); ++idx)
    {
        double distance = distanceToLine(points.front(), points.back(), points[idx]);

        if (distance > d_max)
        {
            index = idx;
            d_max = distance;
        }
    }

    std::deque<Waypoint> results;
    ROS_INFO("dmax: %f", d_max);

    // if max distance is greater than epsilon, recursively simplify
    if (d_max > epsilon)
    {
        // recursive call
        std::deque<Waypoint> first_seg(points.begin(), points.begin() + index + 1);
        std::deque<Waypoint> second_seg(points.begin() + index, points.end());
        std::deque<Waypoint> results_1 = douglasPeuckerMethod(first_seg, epsilon);
        std::deque<Waypoint> results_2 = douglasPeuckerMethod(second_seg, epsilon);

        // build the results list
        results.insert(results.end(), results_1.begin(), results_1.begin() + results_1.size() - 1);
        results.insert(results.end(), results_2.begin(), results_2.end());
    }
    else
    {
        results.push_back(points.front());
        results.push_back(points.back());
    }
    // return the result
    return results;
}


Path::Path(const costmap_2d::Costmap2D *grid_in):
    grid_(grid_in)
{}

/*
 * Check whether the current path is still valid given the current Occupancy grid
 * (ie check whether path intersects an obstacle)
 */
bool Path::isSplinePathValid() const
{
    return isSplinePathValidPrivate(spline_path_.begin(), spline_path_.end());
}

/*
 * Add waypoint to the end of the path
 */
void Path::addWaypoint(const Waypoint &wp)
{
    path_.push_back(wp);
}

bool Path::insertWaypoint(const Waypoint &wp, size_t index)
{
    // common case: add waypoint at the front of the path
    if (index == 0)
    {
        path_.push_front(wp);
        return true;
    }
    else if (index == path_.size())
    {
        path_.push_back(wp);
        return true;
    }
    else if (index > path_.size()) return false;
    else
    {
        path_.insert(path_.begin() + index, wp);

        return true;
    }
}

/*
 * Remove a waypoint from the path
 * 
 * Returns: whether the removal was successful
 */
bool Path::removeWaypoint(const Waypoint &wp)
{
    for (deque<Waypoint>::iterator cur = path_.begin(); cur != path_.end(); ++cur)
    {
        if (*cur == wp)
        {
            path_.erase(cur);
            return true;
        }
    }

    return false;
}

 /*
* Remove a waypoint from the path
* 
* Returns: whether the removal was successful
*/
bool Path::removeWaypoint(size_t index)
{
    deque<Waypoint>::iterator pos = path_.begin() + index;
    path_.erase(pos);

    return true;
}

/*
* Removes all waypoints from the current path
*/
void Path::clearPath()
{
    spline_path_.clear();
    path_.clear();
}

// deprecated (but may be used in future implementations)
const std::deque<Waypoint>& Path::getPath() const
{
    return path_;
}

 /*
* Gets path (pre-spline version)
*/
const std::vector<Coordinate<double> >* Path::getSplinePath() const
{
    return &spline_path_;
}

/*
* We don't want to add a spline, but we want the path to be the same format as a spline path.
* So we fill in between the waypoints with a straight line of equidistant points with the 
* same density of a spline path.
*/
void Path::noSplinePath()
{
     for (int i = 0; i < path_.size() - 1; ++i)
    {
        double distance = distanceTwoPoints(path_[i].x, path_[i].y, path_[i+1].x, path_[i+1].y);
        int num_points = distance / POINT_SEPERATION;

        double temp_x_value = path_[i].x;
        double temp_y_value = path_[i].y;

        double x_distance = path_[i+1].x - path_[i].x;
        double y_distance = path_[i+1].y - path_[i].y;
        double delta_x = x_distance / num_points;
        double delta_y = y_distance / num_points;
        for (int j = 0; j < num_points; j++)
        {
            temp_x_value += delta_x;
            temp_y_value += delta_y;
            spline_path_.push_back({temp_x_value, temp_y_value}); //NOLINT
        }
    }
}

/*
* We currently have a path of points generated by theta* that are not necessarily
* equidistant from each other. This overlays a spline onto this path using a spline library.
* The spline path has equidistant points to allow motion planning to hit all the points smoothly.
*/
void Path::interpolateSpline()
{
    tk::spline sx;
    tk::spline sy;
    std::vector<double> st;  // parameter
    std::vector<double> x_temp;
    std::vector<double> y_temp;

    // path must be at least three points to make spline
    while (path_.size() <= 2)
    {
        // add a middle point
        Waypoint w((path_.front().x + path_.back().x)/2, (path_.front().y + path_.back().y)/2, path_.back().yaw);
        path_.insert(path_.begin() + 1, w);
    }

    x_temp.reserve(path_.size());
    y_temp.reserve(path_.size());
    // x_temp is filled with all x path_ values
    // y_temp is filled with all y path_ values
    for (auto wayPoint : path_)
    {
        x_temp.push_back(wayPoint.x);
        y_temp.push_back(wayPoint.y);
    }

    st.reserve(path_.size());
    for (double t = 0; t < path_.size(); ++t)
    {
        st.push_back(t);
    }

    // set up the splines for x and y points
    sx.set_points(st, x_temp);
    sy.set_points(st, y_temp);

    for (int i = 0; i < path_.size() - 1; ++i)
    {
        double distance = distanceTwoPoints(path_[i].x, path_[i].y, path_[i+1].x, path_[i+1].y);
        int num_points = distance / POINT_SEPERATION;
        for (int j = 0; j < num_points; j++)
        {
            double denser_index = i + j * 1/static_cast<double>(num_points);
            spline_path_.push_back({sx(denser_index), sy(denser_index)}); //NOLINT
        }
    }
}

// iterates through each segment and returns whether the whole path is valid
bool Path::isSplinePathValidPrivate(vec_const_iterator begin, vec_const_iterator end) const
{
    while (begin != end)
    {
        auto it = begin;
        if (++it == end)
            // we don't want to check validity of last point in case we were directed there
            // we'll assume the people telling us what to do have their reasons
            break;
        Coordinate<double> w = *begin;
        unsigned int grid_x, grid_y;
        // converts waypoints coords to costMap coords
        if (!grid_->worldToMap(w.x, w.y, grid_x, grid_y))
        {
            // If the boat shifts away from the destination, the path may go beyond
            // the bounds of the map because the costmap is centered on the boat
            /*NOLINT*/ROS_WARN("The path point (%f, %f) is not in the costmap! \
                    This is likely due to the boat drifting away from its target waypoint.", w.x, w.y); //NOLINT
            ++begin;
            continue;
        }
        uint8_t cost = grid_->getCost(grid_x, grid_y);
        if (cost == costmap_2d::LETHAL_OBSTACLE
            || cost == costmap_2d::NO_INFORMATION
            || cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
        {
            return false;
        }
        ++begin;
    }
    return true;
}

// get pre-spline path size
size_t Path::getSize()
{
    return path_.size();
}
