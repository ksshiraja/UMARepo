#ifndef UMA_NAVIGATION_VISUALIZATION_H
#define UMA_NAVIGATION_VISUALIZATION_H

#include <uma_navigation/path.h>
#include <nav_msgs/Odometry.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <uma_navigation/path_gen.h>
#include <vector>
#include <deque>
#include <iostream>

using costmap_2d::Costmap2D;
using std::vector;
using std::deque;
using std::cout;
using std::endl;

void printCostmap(costmap_2d::Costmap2D *costmap, const PlannerInput &input_, const Path &path)
{
    MapCell<unsigned int> cell;
    costmap->worldToMap(input_.pose.x, input_.pose.y, cell.x, cell.y);
    cout << "Costmap: Origin: (" << costmap->getOriginX() << ", " << costmap->getOriginY() << ")\n";
    for (int y = costmap->getSizeInCellsY() - 1; y >= 0; --y)
    {
        for (int x = 0; x < costmap->getSizeInCellsX(); ++x)
        {
            if (x == cell.x && y == cell.y)
            {
                cout << "\033[1;44m";
                cout << "BOAT";
                cout << "\t\033[0m";
            }
            else
            {
                bool is_waypoint = false;
                const deque<Waypoint> &p = path.getPath();

                for (int pos = 0; pos < p.size(); ++pos)
                {
                    const Waypoint &w = p[pos];
                    MapCell<unsigned int> cell;
                    costmap->worldToMap(w.x, w.y, cell.x, cell.y);
                    if (cell.x == x && cell.y == y)
                    {
                        is_waypoint = true;
                        cout << "\033[1;45m";
                        cout << "X" << pos;
                        cout << "\t\033[0m";
                        break;
                    }
                }
                if (is_waypoint) continue;

                int cost = costmap->getCost(x, y);
                if (cost == costmap_2d::LETHAL_OBSTACLE)
                {
                    cout << "\033[1;41m";
                    cout << cost;
                    cout << "\t\033[0m";
                }
                else if (cost > 150)
                {
                    cout << "\033[1;43m";
                    cout << cost;
                    cout << "\t\033[0m";
                }
                else if (cost < 50)
                {
                    cout << "\033[1;42m";
                    cout << cost;
                    cout << "\t\033[0m";
                }
                else
                {
                    cout << static_cast<int>(costmap->getCost(x, y)) << '\t';
                }
            }
        }
        cout << '\n';
    }
    cout << endl;
}

#endif  // UMA_NAVIGATION_VISUALIZATION_H
