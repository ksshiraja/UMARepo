//  Copyright 2019 <UMAutonomy>

#include <uma_navigation/path_gen.h>
#include <uma_navigation/path.h>
#include <uma_navigation/path_planner.h>

#include <array>
#include <vector>
#include <cassert>
#include <queue>
#include <set>
#include <limits>
#include <iostream>

using costmap_2d::Costmap2D;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using std::vector;
using std::shared_ptr;
using std::set;
using std::pow;

// not sure why this isn't also a constant in costmap_2d
// it represents the boundary between not colliding and colliding based on orientation
const unsigned char CIRCUMSCRIBED_INFLATED_OBSTACLE = 128;


template <class IndexType, class CostType>
class Node
{
  public:
    Node(CostType distance_to_goal, CostType distance_from_start, std::shared_ptr<Node<IndexType, CostType> >
        previous_node, MapCell<IndexType> cell_coors)
        : distance_to_goal_(distance_to_goal), distance_from_start_(distance_from_start),
            previous_node_(previous_node), cell_coors_(cell_coors),
            total_cost_(distance_to_goal + distance_from_start), closed_(false) {}

    Node()
      : total_cost_(0), distance_from_start_(0), previous_node_(nullptr), cell_coors_(0, 0), closed_(false)  {}

    MapCell<IndexType> getMapCell()
    {
      return this->cell_coors_;
    }

    IndexType x() const { return cell_coors_.x; }
    IndexType y() const { return cell_coors_.y; }

    std::shared_ptr<Node<IndexType, CostType> > getPrevious()
    {
      return previous_node_;
    }

    void updateParentNode(std::shared_ptr<Node<IndexType, CostType> > parent_in)
    {
      previous_node_ = parent_in;
    }

    bool operator==(const Node<IndexType, CostType> &other)
    {
      return cell_coors_ == other.cell_coors_;
    }

    bool operator!=(const Node<IndexType, CostType> &other)
    {
      return (cell_coors_.x != other.cell_coors_.x || cell_coors_.y != other.cell_coors_.y);
    }

    bool isClosed() const { return closed_; }

    void close() { closed_ = true; }

    CostType getDistanceFromStart() const { return distance_from_start_; }

    void updateDistanceFromStart(const CostType start_distance_in)
    {
      // update the total cost to account for the new start distance
      distance_from_start_ = start_distance_in;
      total_cost_ = distance_from_start_ + distance_to_goal_;
    }

    CostType getTotalCost() const { return total_cost_; }

  private:
    MapCell<IndexType> cell_coors_;
    CostType total_cost_;
    CostType distance_to_goal_;
    CostType distance_from_start_;
    std::shared_ptr<Node<IndexType, CostType> > previous_node_;
    bool closed_;
};

// used to make lowest value, the top of the priority_queue
template <class IndexType, class CostType>
struct NodeCompare
{
  bool operator()(const std::shared_ptr<Node<IndexType, CostType> > l,
    const std::shared_ptr<Node<IndexType, CostType> > r)
  {
    if (l->getTotalCost() == r->getTotalCost())
    {
      if (l->x() == r->x())
      {
        return l->y() < r->y();
      }
      return l->x() < r->x();
    }
    return l->getTotalCost() < r->getTotalCost();
  }
};

using GridNode = Node<int, double>;
using GridNodeCompare = NodeCompare<int, double>;

// Heuristic: distance from cell to goal
double calcHeuristic(const GridCell &cell, const GridCell &goal);
// backtracks from the destination to create a path for the boat
void genPath(Path *path, const Costmap2D *map, const shared_ptr<GridNode> start_node,
    shared_ptr<GridNode> cur_in, const Waypoint &destination);

inline bool cellInMap(const GridCell &cell, const Costmap2D *map)
{
    return (cell.x >= 0 && cell.x < map->getSizeInCellsX()) && (cell.y >= 0 && cell.y < map->getSizeInCellsY());
}

inline bool cellValid(const GridCell &cell, const GridCell &dest, const Costmap2D *map)
{
    return (map->getCost(cell.x, cell.y) < CIRCUMSCRIBED_INFLATED_OBSTACLE);
}

double calcHeuristic(const GridCell &cell, const GridCell &goal)
{
  // this is the distance from cell to goal
  double a = pow(static_cast<double>(cell.x) - static_cast<double>(goal.x), 2);
  double b = pow(static_cast<double>(cell.y) - static_cast<double>(goal.y), 2);
  return sqrt(a + b);
}

bool onBorder(const Costmap2D *map, const GridNode &node)
{
  return (node.x() == 0 || node.x() == map->getSizeInCellsX() - 1
        || node.y() == 0 || node.y() == map->getSizeInCellsY() - 1);
}

bool lineOfSight(const GridCell &point1, const GridCell &point2, const GridCell &goal_cell, const Costmap2D *map)
{
  // T. Uras and S. Koenig,  2015. An Empirical Comparison of Any-Angle
  // Path-Planning Algorithms. In: Proceedings of the 8th Annual Symposium on Combinatorial
  // Search. Code available at: http://idm-lab.org/anyangle

  // This line of sight check uses only integer values. First it checks whether the
  // movement along the x or the y axis is longer and moves along the longer
  // one cell by cell. dx and dy specify how many cells to move in each direction.
  // Suppose dx is longer and we are moving along the x axis. For each
  // cell we pass in the x direction, we increase variable f by dy, which is initially 0.
  // When f >= dx, we move along the y axis and set f -= dx. This way,
  // after dx movements along the x axis, we also move dy moves along the y axis.

    // x and y values correspond to corners, not cells.
    int x1 = point1.x;  // Originate from this cell.
    int y1 = point1.y;

    int x2 = point2.x;  // Move to this cell.
    int y2 = point2.y;

    int dy = point2.y - point1.y;
    int dx = point2.x - point1.x;

    int f = 0;

    // Direction of movement. Value can be either 1 or -1.
    int sy, sx;
    // The x and y locations correspond to corners, not cells. We might need to check
    // different surrounding cells depending on the direction we do the
    // line of sight check. The following values are usedto determine which cell to check to see if it is unblocked.
    int x_offset, y_offset;

    if (dy < 0)
    {
      dy = -dy;
      sy = -1;
      // Cell is to the North
      y_offset = 0;
    }
    else
    {
      sy = 1;
      // Cell is to the South
      y_offset = 1;
    }

    if (dx < 0)
    {
      dx = -dx;
      sx = -1;
      // Cell is to the West
      x_offset = 0;
    }
    else
    {
      sx = 1;
      // Cell is to the East
      x_offset = 1;
    }

    if (dx >= dy)
    {
      // Move along the x axis and increment/decrement y when f >= dx.
      while (x1 != x2)
      {
        f = f + dy;
        if (f >= dx)
        {
          // We are changing rows, we might need to check two cells this iteration.
          if (!cellValid(GridCell(x1 + x_offset, y1 + y_offset), goal_cell, map))
            return false;

          y1 = y1 + sy;
          f = f - dx;
        }

        if (f != 0)
        {
          //  If f == 0, then we are crossing the row at a corner point and we don't need to check both cells.
          if (!cellValid(GridCell(x1 + x_offset, y1 + y_offset), goal_cell, map))
            return false;
        }

        if (dy == 0)
        {
          // If we are moving along a horizontal line, either the north or the south cell should be unblocked.
          if (!cellValid(GridCell(x1 + x_offset, y1), goal_cell, map) &&
              !cellValid(GridCell(x1 + x_offset, y1 + 1), goal_cell, map))
            return false;
        }

        x1 += sx;
      }
    }

    else
    {
      // if (dx < dy). Move along the y axis and increment/decrement x when f >= dy.
      while (y1 != y2)
      {
        f = f + dx;
        if (f >= dy)
        {
          if (!cellValid(GridCell(x1 + x_offset, y1 + y_offset), goal_cell, map))
            return false;

          x1 = x1 + sx;
          f = f - dy;
        }

        if (f != 0)
        {
          if (!cellValid(GridCell(x1 + x_offset, y1 + y_offset), goal_cell, map))
            return false;
        }

        if (dx == 0)
        {
          if (!cellValid(GridCell(x1, y1 + y_offset), goal_cell, map) &&
              !cellValid(GridCell(x1 + 1, y1 + y_offset), goal_cell, map))
            return false;
        }

        y1 += sy;
      }
    }
    return true;
}

void thetaStarSearch(Path *path, const Costmap2D *map, const GridCell &start_cell, const Waypoint &destination)
{
  // Based on this Paper from Nash, Daniel, and Koenig. http://idm-lab.org/bib/abstracts/papers/aaai07a.pdf
  // The paper has pretty good pseudocode
  GridCell goal_cell;
  bool goal_in_map = false;
  {
      MapCell<unsigned int> goal_map_cell;
      // need not be in map because we start in the center and we can end when we reach the border,
      // which should be an optimal intermediate point
      // we do want to know if the target is in the map though to avoid ending at border point early
      if (!(goal_in_map = map->worldToMap(destination.x, destination.y, goal_map_cell.x, goal_map_cell.y)))
          map->worldToMapNoBounds(destination.x, destination.y, goal_cell.x, goal_cell.y);
      else
          goal_cell = { static_cast<int>(goal_map_cell.x), static_cast<int>(goal_map_cell.y) };
  }
  std::cout << "goal in Map: " << goal_in_map << "\n";

  std::cout << "goal_cell, x: " << goal_cell.x << ", y: " << goal_cell.y <<  "\n";

  set<shared_ptr<GridNode>, GridNodeCompare> nodes_to_visit;
  vector<shared_ptr<GridNode> > closed_list;

  // create goal and start cells in map coors
  shared_ptr<GridNode> start_node = std::make_shared<GridNode>(calcHeuristic(goal_cell, start_cell),
      0, shared_ptr<GridNode>(nullptr), start_cell);

  shared_ptr<GridNode> goal_node = std::make_shared<GridNode>(0, 0,
      shared_ptr<GridNode>(nullptr), goal_cell);


  // store which spaces have been considered already
  std::cout << "map origin: " << map->getOriginX() << ", y: " << map->getOriginY() << "\n";
  std::cout << "x size:  " << map->getSizeInCellsX() << ", y size: " << map->getSizeInCellsY() << "\n";
  int max_width = map->getSizeInCellsX();
  int max_height = map->getSizeInCellsY();

  vector<vector<shared_ptr<GridNode>> > reached(map->getSizeInCellsX(),
      vector<shared_ptr<GridNode>>(map->getSizeInCellsY(), nullptr));


  // add node of zero weight to priority queue --> contains MapCell coordinates of boat
  nodes_to_visit.insert(start_node);
  reached[start_cell.x][start_cell.y] = start_node;

  // the node we will be looking at
  shared_ptr<GridNode> cur;


  // for if we hit the border and the target is in the map
  // we store a border point in case the target ends up not being reachable
  bool reached_border_point = false;
  shared_ptr<GridNode> optimal_border_point;

  // conditions for while loop to break:
  // 1. Create a valid path to the destination
  // 2. Destination is outside the map and the path reaches the map border
  // 3. all possible nodes are visited without creating a valid path:
  //     If we reached the map border, we'll exit there, else, we're stuck
  while (!nodes_to_visit.empty())
  {
    // an array containing all the MapCells adjacent to current node
    vector<GridCell> neighbor_cells;
    neighbor_cells.reserve(8);
    // cur points to GridNode of lowest cost
    cur = *(nodes_to_visit.begin());
    // take current off the open list --> move to closed
    closed_list.push_back(cur);
    nodes_to_visit.erase(nodes_to_visit.begin());
    reached[cur->x()][cur->y()]->close();

    // breaks if the algorithm reaches the destination
    bool is_on_border = onBorder(map, *cur);
    if (*cur == *goal_node)
    {
      std::cout << "reached goal node\n";
      break;
    }
    else if (is_on_border)
    {
        if (!reached_border_point)
        {
            // store border point as potential target if the actual goal isn't reachable
            reached_border_point = true;
            // we only update this once as the optimal exit point is probably the first encountered
            optimal_border_point = cur;
        }
        if (!goal_in_map)
        {
            // break at the optimal border point if the target is outside the costmap
            optimal_border_point = cur;
            std::cout << "breaking at optimal border point\n";
            break;
        }
    }
    // get the current cell coordinates
    int row = cur->getMapCell().x;
    int col = cur->getMapCell().y;

    // check bounds and find up to eight neighbors: top to bottom, left to right (column by column)
    std::array<GridCell, 8> neighboring_cells =
    {
        GridCell(row - 1, col), GridCell(row, col - 1),
        GridCell(row, col + 1), GridCell(row + 1, col),
        GridCell(row - 1, col - 1), GridCell(row + 1, col - 1),
        GridCell(row - 1, col + 1), GridCell(row + 1, col + 1)
    };
    for (const GridCell &neighbor : neighboring_cells)
    {
        if (cellValid(neighbor, goal_cell, map) && (neighbor.x < max_width && neighbor.x >= 0)
            && (neighbor.y < max_height && neighbor.y >= 0))
        {
          neighbor_cells.push_back(neighbor);
        }
    }

    // holds the heuristic cost from each cell to goal --> prioritizes cells closer (by Euclidean distance) to goal
    double heuristic_cost;

    // iterate over neighbors
    shared_ptr<GridNode> previous_node = cur->getPrevious();
    for (int i = 0; i < neighbor_cells.size(); ++i)
    {
      shared_ptr<GridNode> neighbor;
      // if neighbors is a valid cell in the map
      if (reached[neighbor_cells[i].x][neighbor_cells[i].y] == nullptr ||
          !reached[neighbor_cells[i].x][neighbor_cells[i].y]->isClosed())
      {
        bool neighbor_already_opened = false;
        if (reached[neighbor_cells[i].x][neighbor_cells[i].y] == nullptr)
        {
          // we haven't considered this node yet
          neighbor = std::make_shared<GridNode>(calcHeuristic(neighbor_cells[i], goal_cell),
              std::numeric_limits<double>::infinity(), shared_ptr<GridNode>(nullptr), neighbor_cells[i]);

          reached[neighbor_cells[i].x][neighbor_cells[i].y] = neighbor;
        }
        else
        {
          // this cell is already open
          neighbor = reached[neighbor_cells[i].x][neighbor_cells[i].y];
          neighbor_already_opened = true;
        }
        bool temp = previous_node != nullptr;
        // Check if possible to get to neighbor node from current node's parent
        if (previous_node != nullptr && lineOfSight(previous_node->getMapCell(), neighbor_cells[i], goal_cell, map))
        {
          // is the path cheaper if the current node is cut out
          double cost_without_current_node = previous_node->getDistanceFromStart() +
              calcHeuristic(neighbor_cells[i], previous_node->getMapCell());

          if (cost_without_current_node < neighbor->getDistanceFromStart())
          {
            // so we don't invalidate the set, we must erase the node before we change it
            if (neighbor_already_opened)
            {
              nodes_to_visit.erase(neighbor);
            }
            neighbor->updateParentNode(previous_node);
            neighbor->updateDistanceFromStart(cost_without_current_node);
            // re-add the node to the set
            nodes_to_visit.insert(neighbor);
          }
        }
        // check if neighbors distance is better with current node as parent
        else
        {
          // is the path cheaper if the current node
          double cost_with_current_node = cur->getDistanceFromStart() +
              calcHeuristic(neighbor_cells[i], cur->getMapCell());

          if (cost_with_current_node < neighbor->getDistanceFromStart())
          {
            // so we don't invalidate the set, we must erase the node before we change it
            if (neighbor_already_opened)
            {
              nodes_to_visit.erase(neighbor);
            }
            neighbor->updateParentNode(cur);
            neighbor->updateDistanceFromStart(cost_with_current_node);
            // re-add the node to the set
            nodes_to_visit.insert(neighbor);
          }
        }
      }
    }
  }

  Waypoint dest = destination;
  // we stopped at the edge and the target is outside the costmap
  if (*cur != *goal_node)
  {
    if (!reached_border_point)
    {
        // we haven't reached a border point but we haven't reached the target
        // the target isn't reachable and there's no way around it
        // we're likely trapped
        throw PathPlannerException("We are trapped!");
    }
    map->mapToWorld(optimal_border_point->x(), optimal_border_point->y(), dest.x, dest.y);
  }

  // generate the Path in World Coors --> add to Path Container
  genPath(path, map, start_node, closed_list.back(), dest);
}

void genPath(Path *path, const costmap_2d::Costmap2D *map, const shared_ptr<GridNode> start_node,
  shared_ptr<GridNode> cur_in, const Waypoint &destination)
{
  shared_ptr<GridNode> cur = cur_in;
  double world_coors_x;
  double world_coors_y;

  path->insertWaypoint(destination, 0);
  cur = cur->getPrevious();
  if (!cur)
      // we are already at the destination
      return;

  // find path to destination by backtracking from the destination, going through previousNode
  while (cur != start_node)
  {
    map->mapToWorld(cur->getMapCell().x, cur->getMapCell().y, world_coors_x, world_coors_y);
    Waypoint point(world_coors_x, world_coors_y);
    path->insertWaypoint(point, 0);
    cur = cur->getPrevious();
  }
  map->mapToWorld(cur->getMapCell().x, cur->getMapCell().y, world_coors_x, world_coors_y);
  Waypoint point(world_coors_x, world_coors_y);
  path->insertWaypoint(point, 0);
}
