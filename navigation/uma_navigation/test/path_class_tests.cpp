/*
 * Copyright 2019 UM::Autonomy
 */

#include "gtest/gtest.h"
#include "uma_navigation/path.h"
#include "uma_navigation/path_gen.h"
#include "uma_navigation/visualization.h"

#include <utility>
#include <vector>
#include <deque>
#include <memory>
#include <limits>
#include <costmap_2d/cost_values.h>

using std::make_pair;
using std::pair;
using std::vector;
using std::deque;
using std::array;
using std::shared_ptr;

using costmap_2d::Costmap2D;
using costmap_2d::LETHAL_OBSTACLE;
using GridCell = MapCell<int>;


void EXPECT_WAYPOINT_EQ(const Waypoint &a, const Waypoint &b)
{
    EXPECT_NEAR(a.x, b.x, 0.1);
    EXPECT_NEAR(a.y, b.y, 0.1);
}

void EXPECT_PATH_EQ(const deque<Waypoint> &a, const deque<Waypoint> &b)
{
    EXPECT_EQ(a.size(), b.size());

    for (size_t pt = 0; pt < a.size(); ++pt)
    {
        EXPECT_WAYPOINT_EQ(a[pt], b[pt]);
    }
}

TEST(PathClassTest, createPath)
{
    Costmap2D grid(5, 5, 1.0, 0, 0, 0);
    Path path(&grid);
    path.addWaypoint(Waypoint(0.5, 20));
    path.addWaypoint(Waypoint(0, 0));
    path.addWaypoint(Waypoint(100, 15.2));

    EXPECT_EQ(path.getPath().size(), 3);
    EXPECT_EQ(path.getPath()[0], Waypoint(0.5, 20));
    EXPECT_EQ(path.getPath()[1], Waypoint(0, 0));
    EXPECT_EQ(path.getPath()[2], Waypoint(100, 15.2));
}

TEST(PathClassTest, removeWaypointBasic)
{
    Costmap2D grid(5, 5, 1.0, 0, 0, 0);
    Path path(&grid);
    path.addWaypoint(Waypoint(0, 45.2));
    path.addWaypoint(Waypoint(50, 45));

    EXPECT_EQ(path.getPath().size(), 2);
    EXPECT_TRUE(path.removeWaypoint(0));

    EXPECT_EQ(path.getPath().size(), 1);
    EXPECT_EQ(path.getPath()[0], Waypoint(50, 45));
    EXPECT_TRUE(path.removeWaypoint(0));
    EXPECT_EQ(path.getPath().size(), 0);

    path.addWaypoint(Waypoint(45, 2));
    path.addWaypoint(Waypoint(0, 5));
    EXPECT_TRUE(path.removeWaypoint(Waypoint(0, 5)));
    EXPECT_TRUE(!path.removeWaypoint(Waypoint(0, 5)));
}

TEST(PathClassTest, insertWaypointBasic)
{
    Costmap2D grid(5, 5, 1.0, 0, 0, 0);
    Path path(&grid);
    path.addWaypoint(Waypoint(0, 0));
    path.addWaypoint(Waypoint(100, 0));
    EXPECT_TRUE(path.insertWaypoint(Waypoint(50, 4.5), 1));

    EXPECT_EQ(path.getPath().size(), 3);
    EXPECT_EQ(path.getPath()[1], Waypoint(50, 4.5));

    EXPECT_TRUE(path.insertWaypoint(Waypoint(12, 4), 3));
    EXPECT_EQ(path.getPath()[0], Waypoint(0, 0));
    EXPECT_EQ(path.getPath()[1], Waypoint(50, 4.5));
    EXPECT_EQ(path.getPath()[2], Waypoint(100, 0));
    EXPECT_EQ(path.getPath()[3], Waypoint(12, 4));
    EXPECT_EQ(path.getPath().size(), 4);

    EXPECT_TRUE(path.insertWaypoint(Waypoint(5, 5), 0));
    EXPECT_EQ(path.getPath()[0], Waypoint(5, 5));
    EXPECT_EQ(path.getPath().size(), 5);

    EXPECT_TRUE(!path.insertWaypoint(Waypoint(1, 2), 20));
    EXPECT_TRUE(!path.insertWaypoint(Waypoint(2, 3), 6));
    EXPECT_EQ(path.getPath().size(), 5);
}

// Tests where the center of the boat either passes through an obstacle
// or there are no intersections with the path of the boat.
TEST(PathClassTest, PathValidityTestBasic)
{
    Costmap2D grid(5, 5, 1.0, 0, 0, 0);
    grid.setCost(0, 3, LETHAL_OBSTACLE);
    grid.setCost(2, 2, LETHAL_OBSTACLE);

    Path path(&grid);

    Waypoint start_pos(0, 0);
    Waypoint A(4, 0);
    Waypoint B(4, 4);
    Waypoint C(0, 4);

    // path.addWaypoint(A);
    path.addWaypoint(Waypoint(1.5, 0.5));
    path.addWaypoint(Waypoint(2.5, 0.5));
    path.addWaypoint(Waypoint(3.5, 0.5));
    path.addWaypoint(Waypoint(4, 0));

    EXPECT_TRUE(path.isPathValid());
    path.clearPath();

    path.addWaypoint(Waypoint(1.5, 0.5));
    path.addWaypoint(Waypoint(1.5, 1.5));
    path.addWaypoint(Waypoint(2.5, 1.5));
    path.addWaypoint(Waypoint(2.5, 2.5));
    path.addWaypoint(Waypoint(2.5, 3.5));
    path.addWaypoint(Waypoint(3.5, 3.5));
    path.addWaypoint(Waypoint(4.5, 3.5));
    path.addWaypoint(Waypoint(4, 4));

    // path.addWaypoint(B);
    EXPECT_FALSE(path.isPathValid());
    path.clearPath();

    path.addWaypoint(Waypoint(0.5, 1.5));
    path.addWaypoint(Waypoint(0.5, 2.5));
    path.addWaypoint(Waypoint(0.5, 3.5));
    path.addWaypoint(Waypoint(0, 4));

    // path.addWaypoint(C);
    EXPECT_FALSE(path.isPathValid());
    path.clearPath();

    path.addWaypoint(Waypoint(1.5, 0.5));
    path.addWaypoint(Waypoint(2.5, 0.5));
    path.addWaypoint(Waypoint(3.5, 0.5));
    path.addWaypoint(Waypoint(4.5, 0.5));
    path.addWaypoint(Waypoint(4.5, 1.5));
    path.addWaypoint(Waypoint(4.5, 2.5));
    path.addWaypoint(Waypoint(4.5, 3.5));
    path.addWaypoint(Waypoint(4, 4));

    // path.addWaypoint(A);
    // path.addWaypoint(B);
    EXPECT_TRUE(path.isPathValid());
    path.clearPath();

    path.addWaypoint(Waypoint(1.5, 0.5));
    path.addWaypoint(Waypoint(2.5, 0.5));
    path.addWaypoint(Waypoint(3.5, 0.5));
    path.addWaypoint(Waypoint(4.5, 0.5));
    path.addWaypoint(Waypoint(3.5, 0.5));
    path.addWaypoint(Waypoint(3.5, 1.5));
    path.addWaypoint(Waypoint(2.5, 1.5));
    path.addWaypoint(Waypoint(2.5, 2.5));
    path.addWaypoint(Waypoint(1.5, 2.5));
    path.addWaypoint(Waypoint(1.5, 3.5));
    path.addWaypoint(Waypoint(0.5, 3.5));
    path.addWaypoint(Waypoint(0, 4));

    // path.addWaypoint(A);
    // path.addWaypoint(C);
    EXPECT_FALSE(path.isPathValid());
    path.clearPath();

    path.addWaypoint(Waypoint(1.5, 0.5));
    path.addWaypoint(Waypoint(2.5, 0.5));
    path.addWaypoint(Waypoint(3.5, 0.5));
    path.addWaypoint(Waypoint(4.5, 0.5));
    path.addWaypoint(Waypoint(4.5, 1.5));
    path.addWaypoint(Waypoint(4.5, 2.5));
    path.addWaypoint(Waypoint(4.5, 3.5));
    path.addWaypoint(Waypoint(4.5, 4.5));
    path.addWaypoint(Waypoint(4.5, 4.5));
    path.addWaypoint(Waypoint(3.5, 4.5));
    path.addWaypoint(Waypoint(2.5, 4.5));
    path.addWaypoint(Waypoint(1.5, 4.5));
    path.addWaypoint(Waypoint(0.5, 4.5));

    // path.addWaypoint(A);
    // path.addWaypoint(B);
    // path.addWaypoint(C);
    EXPECT_TRUE(path.isPathValid());
}

TEST(PathClassTest, WaypointAngleTest)
{
    Costmap2D grid(30, 30, 1.0, 0, 0, 0);

    Path path(&grid);

    path.addWaypoint(Waypoint(1, 1));
    path.addWaypoint(Waypoint(2, 2));
    path.addWaypoint(Waypoint(3, 1));
    path.addWaypoint(Waypoint(4, 1));
    path.addWaypoint(Waypoint(5, 2));
    path.addWaypoint(Waypoint(6, 3));

    path.calculateWaypointYaw(M_PI_4);
    const deque<Waypoint> waypt_lst = path.getPath();

    EXPECT_NEAR(waypt_lst[0].yaw, M_PI_4, 0.001);
    EXPECT_NEAR(waypt_lst[1].yaw, 0, 0.001);
    EXPECT_NEAR(waypt_lst[2].yaw, (-M_PI_4 / 2), 0.001);
    EXPECT_NEAR(waypt_lst[3].yaw, M_PI_4 / 2, 0.001);
    EXPECT_NEAR(waypt_lst[4].yaw, M_PI_4, 0.001);
    EXPECT_NEAR(waypt_lst[5].yaw, M_PI_4, 0.001);

    // new path
    path.clearPath();

    path.addWaypoint(Waypoint(0, 0));
    path.addWaypoint(Waypoint(1, 0));
    path.addWaypoint(Waypoint(1, 1));
    path.addWaypoint(Waypoint(1, 2));
    path.addWaypoint(Waypoint(1, 0));

    path.calculateWaypointYaw(-M_PI_2);

    EXPECT_NEAR(path.getPath()[0].yaw, 0, 0.001);
    EXPECT_NEAR(path.getPath()[1].yaw, M_PI_4, 0.001);
    EXPECT_NEAR(path.getPath()[2].yaw, M_PI_2, 0.001);
    EXPECT_NEAR(path.getPath()[3].yaw, 0, 0.001);
    EXPECT_NEAR(path.getPath()[4].yaw, -M_PI_2, 0.001);
}

TEST(PathClassTest, TrimPathTests)
{
    Costmap2D grid(30, 30, 1.0, 0, 0, 0);

    Path p(&grid);

    // test case 1
    p.addWaypoint(Waypoint(0, 0));
    p.addWaypoint(Waypoint(0, 1));
    p.addWaypoint(Waypoint(0, 2));
    p.addWaypoint(Waypoint(1, 2));
    p.addWaypoint(Waypoint(1, 3));
    p.addWaypoint(Waypoint(2, 3));
    p.addWaypoint(Waypoint(3, 3));

    Path correct_trim(&grid);
    correct_trim.addWaypoint(Waypoint(0, 0));
    correct_trim.addWaypoint(Waypoint(0, 2));
    correct_trim.addWaypoint(Waypoint(1, 2));
    correct_trim.addWaypoint(Waypoint(1, 3));
    correct_trim.addWaypoint(Waypoint(3, 3));

    p.trimPath(0.0);

    EXPECT_PATH_EQ(p.getPath(), correct_trim.getPath());

    p.clearPath();
    correct_trim.clearPath();

    // test case 2
    p.addWaypoint(Waypoint(0, 0));
    p.addWaypoint(Waypoint(0, 1));
    p.addWaypoint(Waypoint(0, 2));
    p.addWaypoint(Waypoint(1, 2));
    p.addWaypoint(Waypoint(2, 2));
    p.addWaypoint(Waypoint(2, 3));
    p.addWaypoint(Waypoint(3, 3));
    p.addWaypoint(Waypoint(4, 3));

    correct_trim.addWaypoint(Waypoint(0, 0));
    correct_trim.addWaypoint(Waypoint(0, 2));
    correct_trim.addWaypoint(Waypoint(2, 2));
    correct_trim.addWaypoint(Waypoint(2, 3));
    correct_trim.addWaypoint(Waypoint(4, 3));

    p.trimPath(0.0);

    EXPECT_PATH_EQ(p.getPath(), correct_trim.getPath());

    p.clearPath();
    correct_trim.clearPath();

    // test case 3
    p.addWaypoint(Waypoint(0, 0));
    p.addWaypoint(Waypoint(0, 1));
    p.addWaypoint(Waypoint(1, 1));
    p.addWaypoint(Waypoint(1, 2));
    p.addWaypoint(Waypoint(2, 2));
    p.addWaypoint(Waypoint(2, 3));
    p.addWaypoint(Waypoint(3, 3));

    correct_trim.addWaypoint(Waypoint(0, 0));
    correct_trim.addWaypoint(Waypoint(0, 1));
    correct_trim.addWaypoint(Waypoint(1, 1));
    correct_trim.addWaypoint(Waypoint(1, 2));
    correct_trim.addWaypoint(Waypoint(3, 3));

    p.trimPath(0.0, std::numeric_limits<double>::infinity(), 0.5);

    EXPECT_PATH_EQ(p.getPath(), correct_trim.getPath());

    p.clearPath();
    correct_trim.clearPath();

    // test case 4
    p.addWaypoint(Waypoint(0, 0));
    p.addWaypoint(Waypoint(0, 1));
    p.addWaypoint(Waypoint(0, 2));
    p.addWaypoint(Waypoint(1, 2));
    p.addWaypoint(Waypoint(2, 2));

    correct_trim.addWaypoint(Waypoint(0, 0));
    correct_trim.addWaypoint(Waypoint(0, 2));
    correct_trim.addWaypoint(Waypoint(2, 2));

    p.trimPath(0.0);

    EXPECT_PATH_EQ(p.getPath(), correct_trim.getPath());

    p.clearPath();
    correct_trim.clearPath();

    // test case 5
    p.addWaypoint(Waypoint(0, 0));
    p.addWaypoint(Waypoint(1, 0));
    p.addWaypoint(Waypoint(2, 0));
    p.addWaypoint(Waypoint(2, 1));
    p.addWaypoint(Waypoint(3, 1));
    p.addWaypoint(Waypoint(4, 1));

    correct_trim.addWaypoint(Waypoint(0, 0));
    correct_trim.addWaypoint(Waypoint(2, 0));
    correct_trim.addWaypoint(Waypoint(2, 1));
    correct_trim.addWaypoint(Waypoint(4, 1));

    p.trimPath(0.0);

    EXPECT_PATH_EQ(p.getPath(), correct_trim.getPath());
}

TEST(PathClassTest, TrimPathVisualization)
{
    Costmap2D grid(15, 15, 1.0, 0, 0, 0);

    grid.setCost(1, 1, LETHAL_OBSTACLE);
    grid.setCost(2, 1, LETHAL_OBSTACLE);
    grid.setCost(3, 1, LETHAL_OBSTACLE);
    grid.setCost(0, 1, LETHAL_OBSTACLE);
    grid.setCost(14, 13, LETHAL_OBSTACLE);
    grid.setCost(13, 13, LETHAL_OBSTACLE);
    grid.setCost(12, 13, LETHAL_OBSTACLE);
    grid.setCost(11, 13, LETHAL_OBSTACLE);

    Path path(&grid);
    GridCell start(0, 0);
    Waypoint end(14, 14);
    thetaStarSearch(&path, &grid, start, end);

    PlannerInput input_;
    input_.target.x = end.x;
    input_.target.y = end.y;
    input_.target.yaw = 0.0;
    input_.target.error = 0.5;
    input_.target.error_yaw = 0.5;
    input_.pose.x = start.x;
    input_.pose.y = start.y;
    input_.pose.orientationAngle = 0.0;
    input_.pose.xVel = 0.0;
    input_.pose.yVel = 0.0;

    Orientation pose;  // current boat orientation
    Target target;  // current target set by task planner

    printCostmap(&grid, input_, path);

    path.trimPath(0.0, std::numeric_limits<double>::infinity(), 1.2);

    printCostmap(&grid, input_, path);
}

TEST(PathClassTest, DouglasPeuckerTrimming)
{
    Costmap2D grid(15, 15, 1.0, 0, 0, 0);
    Path path(&grid);

    path.addWaypoint(Waypoint(0, 0));
    path.addWaypoint(Waypoint(1, 1));
    path.addWaypoint(Waypoint(0, 2));
    path.addWaypoint(Waypoint(42, 3));
    path.addWaypoint(Waypoint(0, 4));
    path.addWaypoint(Waypoint(0.5, 5));

    path.trimPath(0.0, std::numeric_limits<double>::infinity(), 1.5);

    std::deque<Waypoint> correct_path;
    correct_path.push_back(Waypoint(0, 0));
    correct_path.push_back(Waypoint(0, 2));
    correct_path.push_back(Waypoint(42, 3));
    correct_path.push_back(Waypoint(0.5, 5));

    EXPECT_PATH_EQ(path.getPath(), correct_path);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
