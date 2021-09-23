#include "gtest/gtest.h"
#include "uma_navigation/path.h"
#include "uma_navigation/path_gen.h"

#include <utility>
#include <vector>
#include <costmap_2d/cost_values.h>
#include <memory>

using costmap_2d::Costmap2D;
using std::shared_ptr;
using costmap_2d::LETHAL_OBSTACLE;

void Global_Expect_Eq(const Path &correct, const Path &ans)
{
  ASSERT_EQ(correct.getPath().size(), ans.getPath().size());
  for (int i = 0; i < correct.getPath().size(); i++)
  {
    EXPECT_EQ(correct.getPath()[i], ans.getPath()[i]);
  }
}

TEST(PathUpdateTests, navigate_null_grid)
{
  Costmap2D null_grid(10, 10, 1.0, 0, 0, 0);
  Path empty_grid(&null_grid);
  Path expected_empty_grid(&null_grid);
  // these are here because there are theoretically multiple optimal paths
  // we want to ensure there is only one to make test cases determinant
  null_grid.setCost(1, 0, 1);
  null_grid.setCost(3, 2, 1);

  thetaStarSearch(&empty_grid, &null_grid, GridCell(4, 4), Waypoint(0, 0));

  // path starting from cur location (in world coordinates)
  expected_empty_grid.addWaypoint(Waypoint(3.5, 4.5));
  expected_empty_grid.addWaypoint(Waypoint(3.5, 3.5));
  expected_empty_grid.addWaypoint(Waypoint(2.5, 3.5));
  expected_empty_grid.addWaypoint(Waypoint(2.5, 2.5));
  expected_empty_grid.addWaypoint(Waypoint(1.5, 2.5));
  expected_empty_grid.addWaypoint(Waypoint(1.5, 1.5));
  expected_empty_grid.addWaypoint(Waypoint(0.5, 1.5));
  expected_empty_grid.addWaypoint(Waypoint(0, 0));

  Global_Expect_Eq(empty_grid, expected_empty_grid);
}

TEST(PathUpdateTests, navigate_weighted_5x5_grid)
{
  Costmap2D five_x_five(10, 10, 1.0, 0, 0, 0);
  Path five(&five_x_five);
  Path expected_five(&five_x_five);
  five_x_five.setCost(3, 2, LETHAL_OBSTACLE);
  five_x_five.setCost(3, 3, LETHAL_OBSTACLE);
  five_x_five.setCost(4, 3, LETHAL_OBSTACLE);
  five_x_five.setCost(0, 1, 2);
  five_x_five.setCost(0, 2, 6);
  five_x_five.setCost(0, 3, 9);
  five_x_five.setCost(0, 4, 11);

  five_x_five.setCost(1, 0, 2);
  five_x_five.setCost(1, 1, 1);
  five_x_five.setCost(1, 2, 4);
  five_x_five.setCost(1, 3, 8);
  five_x_five.setCost(1, 4, 10);

  five_x_five.setCost(2, 0, 6);
  five_x_five.setCost(2, 1, 5);
  five_x_five.setCost(2, 2, 3);
  five_x_five.setCost(2, 3, 5);
  five_x_five.setCost(2, 4, 6);

  five_x_five.setCost(3, 0, 9);
  five_x_five.setCost(3, 1, 8);
  five_x_five.setCost(3, 4, 7);

  five_x_five.setCost(4, 0, 18);
  five_x_five.setCost(4, 1, 15);
  five_x_five.setCost(4, 2, 16);
  five_x_five.setCost(4, 4, 8);

  thetaStarSearch(&five, &five_x_five, GridCell(4, 4), Waypoint(0, 0));

  expected_five.addWaypoint(Waypoint(3.5, 4.5));
  expected_five.addWaypoint(Waypoint(2.5, 4.5));
  expected_five.addWaypoint(Waypoint(2.5, 3.5));
  expected_five.addWaypoint(Waypoint(2.5, 2.5));
  expected_five.addWaypoint(Waypoint(1.5, 2.5));
  expected_five.addWaypoint(Waypoint(1.5, 1.5));
  expected_five.addWaypoint(Waypoint(0.5, 1.5));
  expected_five.addWaypoint(Waypoint(0, 0));

  Global_Expect_Eq(five, expected_five);
}

TEST(PathUpdateTests, short_long)
{
  Costmap2D grid(5, 5, 1.0, 0, 0);
  Path path(&grid);
  Path correct(&grid);

  grid.setCost(1, 1, LETHAL_OBSTACLE);
  grid.setCost(1, 2, LETHAL_OBSTACLE);
  grid.setCost(1, 3, LETHAL_OBSTACLE);

  grid.setCost(3, 0, LETHAL_OBSTACLE);
  grid.setCost(3, 1, LETHAL_OBSTACLE);
  grid.setCost(3, 2, LETHAL_OBSTACLE);
  grid.setCost(3, 3, LETHAL_OBSTACLE);

  thetaStarSearch(&path, &grid, GridCell(0, 2), Waypoint(4, 2));

  correct.addWaypoint(Waypoint(0.5, 3.5));
  correct.addWaypoint(Waypoint(0.5, 4.5));
  correct.addWaypoint(Waypoint(1.5, 4.5));
  correct.addWaypoint(Waypoint(2.5, 4.5));
  correct.addWaypoint(Waypoint(3.5, 4.5));
  correct.addWaypoint(Waypoint(4.5, 4.5));
  correct.addWaypoint(Waypoint(4.5, 3.5));
  correct.addWaypoint(Waypoint(4, 2));

  Global_Expect_Eq(path, correct);
}

TEST(PathUpdateTests, costly_moves)
{
  Costmap2D grid(5, 5, 1.0, 0, 0);
  Path path(&grid);
  Path correct(&grid);

  grid.setCost(4, 3, LETHAL_OBSTACLE);
  grid.setCost(3, 2, LETHAL_OBSTACLE);
  grid.setCost(3, 1, LETHAL_OBSTACLE);
  grid.setCost(2, 2, LETHAL_OBSTACLE);
  grid.setCost(1, 2, LETHAL_OBSTACLE);
  grid.setCost(3, 3, LETHAL_OBSTACLE);

  grid.setCost(2, 3, 127);
  grid.setCost(2, 0, 127);
  grid.setCost(1, 1, 127);

  thetaStarSearch(&path, &grid, GridCell(4, 4), Waypoint(4, 2));

  correct.addWaypoint(Waypoint(3.5, 4.5));
  correct.addWaypoint(Waypoint(2.5, 4.5));
  correct.addWaypoint(Waypoint(2.5, 3.5));
  correct.addWaypoint(Waypoint(1.5, 3.5));
  correct.addWaypoint(Waypoint(0.5, 3.5));
  correct.addWaypoint(Waypoint(0.5, 2.5));
  correct.addWaypoint(Waypoint(0.5, 1.5));
  correct.addWaypoint(Waypoint(1.5, 1.5));
  correct.addWaypoint(Waypoint(2.5, 1.5));
  correct.addWaypoint(Waypoint(2.5, 0.5));
  correct.addWaypoint(Waypoint(3.5, 0.5));
  correct.addWaypoint(Waypoint(4.5, 0.5));
  correct.addWaypoint(Waypoint(4.5, 1.5));
  correct.addWaypoint(Waypoint(4, 2));

  Global_Expect_Eq(path, correct);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
