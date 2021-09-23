#include "gtest/gtest.h"
#include "uma_navigation/path.h"
#include "uma_navigation/path_gen.h"

#include <costmap_2d/cost_values.h>
#include <memory>

using std::shared_ptr;
using costmap_2d::Costmap2D;

void Global_Expect_Eq(const Path &correct, const Path &ans)
{
  ASSERT_EQ(correct.getPath().size(), ans.getPath().size());
  for (int i = 0; i < correct.getPath().size(); i++)
  {
    EXPECT_EQ(correct.getPath()[i], ans.getPath()[i]);
  }
}

TEST(Path_GenClassTest, navigate_uniformly_weighted_grid)
{
    Costmap2D master_grid(10, 10, 1.0, 0.0, 0.0, 8);

    for (unsigned int i = 0; i < master_grid.getSizeInCellsX(); ++i)
    {
        for (unsigned int j = 0; j < master_grid.getSizeInCellsY(); ++j)
        {
            master_grid.setCost(i, j, 5);
        }
    }

    GridCell boatStart(0, 0);
    GridCell boatEnd(3, 3);
    Path path_(&master_grid);
    Path correct(&master_grid);
    unsigned int mapX;
    unsigned int mapY;
    thetaStarSearch(&path_, &master_grid, boatStart, Waypoint(3, 3));

    // top to bottom, left to right
    correct.addWaypoint(Waypoint(0.5, 1.5));
    correct.addWaypoint(Waypoint(1.5, 1.5));
    correct.addWaypoint(Waypoint(1.5, 2.5));
    correct.addWaypoint(Waypoint(2.5, 2.5));
    correct.addWaypoint(Waypoint(3.5, 2.5));
    correct.addWaypoint(Waypoint(3, 3));
    Global_Expect_Eq(path_, correct);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
