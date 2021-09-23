// Copyright 2019 UMAutonomy

#include "gtest/gtest.h"
#include "uma_navigation/planner_layer.h"
#include <iostream>
#include <vector>
#include <string>

int min_x = 0;
int max_x = 10;
int min_y = 0;
int max_y = 10;
GridLayer gridLayer;

// checks cost map with array
void GLOBAL_EXPECT_EQ(const int ans[11][11], const costmap_2d::Costmap2D &master_grid)
{
    for (int i = 0; i < 11; i++)
    {
        for (int j = 0; j < 11; j++)
        {
            EXPECT_EQ(ans[j][i], master_grid.getCost(i, j));
        }
    }
}

// test doing nothing
TEST(DistanceModel, nothing)
{
    costmap_2d::Costmap2D master_grid(11, 11, 1, 0, 0, 0);
    std::vector<IntPoint> boxes = gridLayer.getBoxesToWeigh(min_x, min_y, max_x, max_y);
    gridLayer.updateDistanceCosts(master_grid, min_x, min_y, max_x, max_y, boxes);
    int ans[11][11] =
    {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    };
    GLOBAL_EXPECT_EQ(ans, master_grid);
}

// test test function
TEST(DistanceModel, testFunction)
{
    costmap_2d::Costmap2D master_grid(11, 11, 1, 0, 0, 0);
    master_grid.setCost(1, 1, 1);
    master_grid.setCost(0, 4, 2);
    master_grid.setCost(10, 10, 3);
    int ans[11][11] =
    {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3,
    };
    EXPECT_EQ(1, ans[1][1]);
    EXPECT_EQ(2, ans[4][0]);
    EXPECT_EQ(3, ans[10][10]);
    EXPECT_EQ(1, master_grid.getCost(1, 1));
    EXPECT_EQ(2, master_grid.getCost(0, 4));
    EXPECT_EQ(3, master_grid.getCost(10, 10));
    GLOBAL_EXPECT_EQ(ans, master_grid);
}

TEST(DistanceModel, oneCenter)
{
    costmap_2d::Costmap2D master_grid(11, 11, 1, 0, 0, 0);
    master_grid.setCost(5, 5, 254);  // center object
    master_grid.setCost(4, 5, 255);  // unknown around object
    master_grid.setCost(5, 6, 10);  // small noise around object
    master_grid.setCost(5, 7, 253);  // max outside range of object
    master_grid.setCost(0, 0, 255);  // max outside range of boat
    master_grid.setCost(0, 1, 254);  // max outside range of boat
    master_grid.setCost(5, 3, 255);  // unknown within range of boat
    std::vector<IntPoint> boxes = gridLayer.getBoxesToWeigh(min_x, min_y, max_x, max_y);
    gridLayer.updateDistanceCosts(master_grid, min_x, min_y, max_x, max_y, boxes);
    int ans[11][11] =
    {
    255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    254, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 255, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 127, 127, 127, 0, 0, 0, 0,
    0, 0, 0, 0, 255, 254, 127, 0, 0, 0, 0,
    0, 0, 0, 0, 127, 137, 127, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 253, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    };
    GLOBAL_EXPECT_EQ(ans, master_grid);
}


// two objects apart
TEST(DistanceModel, apart)
{
    costmap_2d::Costmap2D master_grid(11, 11, 1, 0, 0, 0);
    master_grid.setCost(5, 2, 254);  // edge
    master_grid.setCost(2, 8, 254);  // corner
    std::vector<IntPoint> boxes = gridLayer.getBoxesToWeigh(min_x, min_y, max_x, max_y);
    gridLayer.updateDistanceCosts(master_grid, min_x, min_y, max_x, max_y, boxes);

    int ans[11][11] =
    {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 127, 127, 127, 0, 0, 0, 0,
    0, 0, 0, 0, 127, 254, 127, 0, 0, 0, 0,
    0, 0, 0, 0, 127, 127, 127, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 127, 127, 127, 0, 0, 0, 0, 0, 0, 0,
    0, 127, 254, 127, 0, 0, 0, 0, 0, 0, 0,
    0, 127, 127, 127, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    };
    GLOBAL_EXPECT_EQ(ans, master_grid);
}

// test two objects close to each other
TEST(DistanceModel, twoObjectsClose)
    {
    costmap_2d::Costmap2D master_grid(11, 11, 1, 0, 0, 0);
    master_grid.setCost(3, 3, 254);
    master_grid.setCost(4, 3, 254);
    master_grid.setCost(3, 5, 254);
    std::vector<IntPoint> boxes = gridLayer.getBoxesToWeigh(min_x, min_y, max_x, max_y);
    gridLayer.updateDistanceCosts(master_grid, min_x, min_y, max_x, max_y, boxes);
    int ans[11][11] =
    {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 127, 253, 253, 127, 0, 0, 0, 0, 0,
    0, 0, 127, 254, 254, 127, 0, 0, 0, 0, 0,
    0, 0, 253, 253, 253, 127, 0, 0, 0, 0, 0,
    0, 0, 127, 254, 127, 0, 0, 0, 0, 0, 0,
    0, 0, 127, 127, 127, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    };
    GLOBAL_EXPECT_EQ(ans, master_grid);
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
