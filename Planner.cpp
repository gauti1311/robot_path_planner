#include "Planner.h"
#include "Terrain.h"
#include <iostream>
#include <vector>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <memory>
#include <algorithm>
#include <cmath>
#include <limits>

/**
 * @brief Compare struct for priority queue
 * 
 * Compares two nodes using their f_cost.
 */
struct Compare
{
  /**
   * @brief Compares two nodes by f_cost
   * 
   * @param a First node
   * @param b Second node
   * @return true If the first node has a higher f_cost, false Otherwise
   */
  bool operator()(const Node& a, const Node& b) const
  {
    return a.f_cost() > b.f_cost();
  }
};


Planner::Planner(Terrain* terrain) : terrain(terrain)
{
  grid_size_ = { this->terrain->get_width(), this->terrain->get_height() };
  std::cout << " A star Path planner initialized " << std::endl;
}

inline std::size_t Planner::get_cell_id(const RobotCoordinate& coord) const
{
   return  coord.first * grid_size_.first + coord.second;
}

int Planner::terrain_cost(const RobotCoordinate& coord) const
{
  auto terrain_value = this->terrain->get_value(coord.first, coord.second);
  switch (terrain_value)
  {
    case TerrainValue::WATER:
      return 0;
    case TerrainValue::LEVEL_1:
      return 1;
    case TerrainValue::LEVEL_2:
      return 2;
    case TerrainValue::LEVEL_3:
      return 3;
    case TerrainValue::LEVEL_4:
      return 4;
    case TerrainValue::UNKNOWN:
      return -1;
    default:
      return std::numeric_limits<int>::max();
  }
}

inline int Planner::heuristic(const RobotCoordinate& a, const RobotCoordinate& b) const
{
  /**
   * Using Manhattan distance as heuristic cost
   **/
  return std::abs(static_cast<int>(a.first - b.first)) + std::abs(static_cast<int>(a.second - b.second));
}

inline bool Planner::is_valid(const RobotCoordinate& coord) const
{
  if (coord.first >= 0 && coord.first < grid_size_.first && coord.second >= 0 && coord.second < grid_size_.second)
  {
    auto cost = terrain_cost(coord);
    return cost > 0 && cost != -1;  // Exclude non-traversable cells
  }
  return false;
}

std::vector<RobotCoordinate> Planner::get_neighbors(const RobotCoordinate& coord) const
{
    static const std::vector<RobotCoordinate> directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}}; // left, right, up, down directions
    std::vector<RobotCoordinate> neighbors;
    for (const auto& dir : directions) {
        RobotCoordinate neighbor = {coord.first + dir.first, coord.second + dir.second};
        if (is_valid(neighbor)) {
            neighbors.emplace_back(neighbor);
        }
    }
    return neighbors;
}

RobotPath Planner::plan_path_to_target(const RobotCoordinate start, const RobotCoordinate destination)
{
  RobotPath path{};
  if (!is_valid(start) || !is_valid(destination))
  {
    std::cout << "Invalid start or destination co-ordinates " << std::endl;
    return {};  // Return empty path if inputs are not valid
  }
	/**
		Planning Loop
      1. Add start node to open list, set initial cost = 0. 
      2. LOOP :
        a- set the nodde as current node with lowest f_cost in open list. 
        b- get the all neighbors of each node (4 or less depending on terrain value and grid boundary)
        c- for each neighbor:
          i - if it is in the closed list, ignore it and continue
          ii - if it is already added in cost_map or its new_cost is lesser than current cost update its g_cost,
               and make the current node as parent node to that neighbor, 
               add the node to the open list
      3. If destination is reached, construct the path by tracing parent node starting from destination,
         else continue until destination is reached or open_list is empty (i.e path can not be found)
  **/

  using PriorityQueue =  std::priority_queue<Node, std::vector<Node>, Compare>;
  PriorityQueue open_list;  // A min heap of Nodes (f_cost to compare)
  std::unordered_set<std::size_t> closed_list; // Set to store visited nodes
  std::unordered_map<std::size_t, RobotCoordinate> node_parents; // Map to store the parent of each node based on lowest cost
  std::unordered_map<std::size_t, int> cost_map; // Map to store the cost to reach each node

  auto start_id = get_cell_id(start);
  auto destination_id = get_cell_id(destination);
  open_list.push(Node(start, heuristic(start, destination), 0)); // Add strating node to open_list
  cost_map[start_id] = 0;

  while (!open_list.empty())
  {
    auto current = open_list.top();
    open_list.pop();
    auto current_id = get_cell_id(current.coord_);

    if (current.coord_ == destination)   // End Condition
    {
      std::cout << "Destination reached " << std::endl;
      break;
    }

    if (closed_list.find(current_id) != closed_list.end())
    {
      continue;
    }
    closed_list.insert(current_id);  // Add node as visited

    // Evaluate all valid neighbors of the current node, discard if already visited once.
    for (const auto& n_coord : get_neighbors(current.coord_))
    {
      auto neighbor_id = get_cell_id(n_coord);
      if (closed_list.find(neighbor_id) != closed_list.end())
      {
        continue;
      int new_cost = cost_map[current_id] + terrain_cost(n_coord);
      }
      // Add neighbor to open_list if it not in cost_map or update its cost if new_cost < current_cost
      if (!cost_map.count(neighbor_id) || new_cost < cost_map[neighbor_id]) 
      {
        cost_map[neighbor_id] = new_cost;
        open_list.push(Node(n_coord, heuristic(n_coord, destination), new_cost));
        node_parents[neighbor_id] = current.coord_;
      }
    }
  }

  // Reconstruct the path by tracing parent from destination to start 
  auto node_id = destination_id;
  while (node_id != start_id)
  {
    path.emplace_back(node_parents.at(node_id));
    node_id =  get_cell_id(node_parents.at(node_id));
  }
  path.emplace_back(start);   // Add the start node at end
  // std::reverse(path.begin(), path.end());  // Reverse the path from start -> destination
  return path;
}
