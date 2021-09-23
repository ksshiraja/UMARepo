#include "gtest/gtest.h"
#include "uma_navigation/path.h"
#include "uma_navigation/planner_layer.h"

#include <utility>
#include <vector>
#include <costmap_2d/cost_values.h>

using std::make_pair;
using std::pair;
using std::vector;
using std::array;

using costmap_2d::Costmap2D;
using costmap_2d::LETHAL_OBSTACLE;

int min_x = 0;
int max_x = 11;
int min_y = 0;
int max_y = 11;
PlannerLayer gridLayer;
TEST(MovementModelTest, StaysSameAt0Degrees)
{
    costmap_2d::Costmap2D master_grid(11, 11, 1, 0, 0, 0);
    master_grid.setCost(4, 5, 75);
    int initialValue = master_grid.getCost(4, 5);
    gridLayer.updateBounds(0, 0, M_PI, 0, 0, 0, 0);
    gridLayer.updateCosts(master_grid, min_x, min_y, max_x, max_y);
    int finalValue = master_grid.getCost(4, 5);
    EXPECT_EQ(initialValue, finalValue);
}

TEST(PathClassTest, insertWaypointBasic)
{
    costmap_2d::Costmap2D master_grid(11, 11, 1, 0, 0, 0);
    master_grid.setCost(7, 7, 75);
    int initialValue = master_grid.getCost(7, 7);
    gridLayer.updateBounds(0, 0, 0, 0, 0, 0, 0);
    gridLayer.updateCosts(master_grid, min_x, min_y, max_x, max_y);
    int finalValue = master_grid.getCost(7, 7);
    EXPECT_EQ(initialValue, finalValue);
}

// ----------------- PRIVATE INTERFACE TESTS ------------------- //
TEST(PathClassTest, GetCoordinateTest)
{
    costmap_2d::Costmap2D master_grid(11, 11, 1, 0, 0, 0);
    master_grid.setCost(6, 5, 254);
    int initialValue = master_grid.getCost(6, 5);  // get value at any index
    gridLayer.updateBounds(0, 0, M_PI, 0, 0, 0, 0);
    gridLayer.updateCosts(master_grid, min_x, min_y, max_x, max_y);
    int finalValue = master_grid.getCost(6, 5);
    EXPECT_EQ(initialValue, finalValue);
}

// currently failing
TEST(PathClassTest, GetNextCoordinateTest)
{
    // initial value IV, 155<= IV <254
    costmap_2d::Costmap2D master_grid(11, 11, 1, 0, 0, 0);
    master_grid.setCost(6, 5, 200);
    int initialValue = master_grid.getCost(6, 5);
    gridLayer.updateBounds(0, 0, M_PI, 0, 0, 0, 0);
    gridLayer.updateCosts(master_grid, min_x, min_y, max_x, max_y);
    int finalValue = master_grid.getCost(6, 5);
    EXPECT_EQ(253, finalValue);
}

TEST(PathClassTest, GetPossibleIntersectionSidesTest)
{
    costmap_2d::Costmap2D master_grid(11, 11, 1, 0, 0, 0);
    master_grid.setCost(2, 5, 75);  // Set the initial value to something less than 155
    int initialValue = master_grid.getCost(2, 5);  // could be anywhere, just choses 5,5
    gridLayer.updateBounds(0, 0, 0, 0, 0, 0, 0);
    gridLayer.updateCosts(master_grid, min_x, min_y, max_x, max_y);
    int finalValue = master_grid.getCost(2, 5);
    EXPECT_EQ(initialValue + 69, finalValue);
}

TEST(PathClassTest, LineIntersectionTest)
{
    costmap_2d::Costmap2D master_grid(11, 11, 1, 0, 0, 0);
    master_grid.setCost(5, 7, 75);  // Set the initial value to something less than 155
    int initialValue = master_grid.getCost(5, 7);  // could be anywhere, just choses 5,5
    gridLayer.updateBounds(0, 0, M_PI, 0, 0, 0, 0);
    gridLayer.updateCosts(master_grid, min_x, min_y, max_x, max_y);
    int finalValue = master_grid.getCost(5, 7);
    EXPECT_EQ(initialValue + 29, finalValue);
}

TEST(PathClassTest, GetNextGridSquareTest)
{
    costmap_2d::Costmap2D master_grid(11, 11, 1, 0, 0, 0);
    master_grid.setCost(3, 3, 75);  // Set the initial value to something less than 155
    int initialValue = master_grid.getCost(3, 3);  // could be anywhere, just choses 5,5
    gridLayer.updateBounds(0, 0, 0, 0, 0, 0, 0);
    gridLayer.updateCosts(master_grid, min_x, min_y, max_x, max_y);
    int finalValue = master_grid.getCost(3, 3);
    EXPECT_EQ(initialValue + 53, finalValue);
}

TEST(PathClassTest, DetTest)
{
    costmap_2d::Costmap2D master_grid(11, 11, 1, 0, 0, 0);
    gridLayer.updateBounds(0, 0, -.6, 0, 0, 0, 0);
    std::vector<DoublePoint> points = gridLayer.getCornerPoints();
    std::ostringstream values;
    std::ostringstream correct;
    correct << " 4.169930.782079 -0.7820794.16993 -4.16993-0.782079 0.782079-4.16993";
     for (unsigned i = 0; i < points.size(); i++)
    {
        values << ' ' << points.at(i).x << points.at(i).y;
    }

     EXPECT_EQ(values.str(), correct.str());
}
// -----------------------------End Private Interface tests-------------------------------- //

// Tests where the center of the boat either passes through an obstacle
// or there are no intersections with the path of the boat.
TEST(PathClassTest, PathValidityTestBasic)
{
    costmap_2d::Costmap2D master_grid(11, 11, 1, 0, 0, 0);
    gridLayer.updateBounds(0, 0, M_PI/2, 0, 0, 0, 0);
    std::vector<IntPoint> points = gridLayer.getBoxesToWeigh(min_x, min_y, max_x, max_y);
    std::ostringstream values;
    std::ostringstream correct;
    correct << " 2 2 2 3 2 4 2 5 2 6 2 7 2 8 3 2 3 3 3 4 3 5 3 6 3 7 3 8 4 2 4 3 4 4 4 5 4 6 4 7 4 8 5 2 5 3";
    correct << " 5 4 5 5 5 6 5 7 5 8 6 2 6 3 6 4 6 5 6 6 6 7 6 8 7 2 7 3 7 4 7 5 7 6 7 7 7 8 8 2 8 3 8 4";
    correct << " 8 5 8 6 8 7 8 8";
    for (unsigned i = 0; i < points.size(); ++i)
    {
        values << " " << points.at(i).x << " " << points.at(i).y;
    }

    EXPECT_EQ(values.str(), correct.str());
}

// Tests where the boat's center may not pass through a grid square, but
// the boat intersects an obstacle because one of its sides hit an obstacle
TEST(PathClassTest, PathValidityTestClipping)
{
    costmap_2d::Costmap2D master_grid(11, 11, 1, 0, 0, 0);
    gridLayer.updateBounds(0, 0, M_PI/4, 0, 0, 0, 0);
    std::vector<IntPoint> points = gridLayer.getBoxesToWeigh(min_x, min_y, max_x, max_y);
    std::ostringstream values;
    std::ostringstream correct;
    correct << " 1 5 2 4 2 5 2 6 3 3 3 4 3 5 3 6 3 7 4 2 4 3 4 4 4 5 4 6 4 7 4 8 5 1 5 2 5 3 5 4 5 5 5 6 5 7 5 8 5 9";
    correct << " 6 2 6 3 6 4 6 5 6 6 6 7 6 8 7 3 7 4 7 5 7 6 7 7 8 4 8 5 8 6 9 5";
    for (unsigned i = 0; i < points.size(); ++i)
    {
        values << " " << points.at(i).x << " " << points.at(i).y;
    }

    EXPECT_EQ(values.str(), correct.str());
}

// Tests where the grid square size is small relative to the boat, and
// the boat intersects obstacles between its edges and the center. In other
// words, the boat's center does not touch the obstacle, the boat's edge
// does not touch the obstacle, but part of the boat between the two
// does touch the obstacle. Make sure the resolution of the paths you
// check is tight enough, and that an obstacle doesn't slip through unnoticed
// because of a small threshold error.
TEST(PathClassTest, PathValidityTestResolution)
{
    costmap_2d::Costmap2D master_grid(11, 11, 1, 0, 0, 0);
    gridLayer.updateBounds(0, 0, M_PI/8, 0, 0, 0, 0);
    std::vector<IntPoint> points = gridLayer.getBoxesToWeigh(min_x, min_y, max_x, max_y);
    std::ostringstream values;
    std::ostringstream correct;
    correct << " 2 5 2 6 2 7 3 2 3 3 3 4 3 5 3 6 3 7 4 2 4 3 4 4 4 5 4 6 4 7 5 2 5 3 5 4 5 5 5 6 5 7 5 8";
    correct << " 6 3 6 4 6 5 6 6 6 7 6 8 7 3 7 4 7 5 7 6 7 7 7 8 8 3 8 4 8 5";
    for (unsigned i = 0; i < points.size(); ++i)
    {
        values << " " << points.at(i).x << " " << points.at(i).y;
    }

    EXPECT_EQ(values.str(), correct.str());
}


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
