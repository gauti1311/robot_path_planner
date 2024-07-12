#pragma once

#include "Terrain.h"
#include <limits>

/**
 * @brief Node structure for A* algorithm
 * 
 * Represents a node in the A* pathfinding algorithm
 */
struct Node{
    RobotCoordinate coord_;  ///< x,y : coordincates of node
    int h_cost_;            ///< heuristic value of (x,y) to end node
    int g_cost_;           ///< exact cost from this node to the start_node
  
    /**
     * @brief Constructor for Node struct
     * @param coord (x,y) coordincates of node
     * @param h_cost heuristic value of (x,y) to end node
     * @param g_cost exact cost from this node to the start_node
     */
    Node(const RobotCoordinate coord, const int h_cost=0, int g_cost = std::numeric_limits<int>::max()): coord_(coord), h_cost_(h_cost), g_cost_(g_cost) {}
    
    /**
     * @brief Get the total cost (f_cost)
     * @return The sum of g_cost and h_cost
     */
    int f_cost() const { return g_cost_ + h_cost_; } 
};

/// Plans a path from start to destination
/// @note use this class as an entrypoint for your solution
/// @note you can add any member functions and variables you need
/// @note you may also modify the constructor, but do not change the signature
/// of the plan_path_to_target function

/**
 * @brief Planner class for A* algorithm implementation
 * 
 * This class provides methods to perform A* pathfinding on a given terrain.
 */
class Planner
{
public:
  /**
   * @brief Construct a new Planner object
   * 
   * @param terrain Pointer to the Terrain object
   */
  Planner(Terrain* terrain);

  /**
   * @brief Plans a path from start to destination  
   * @param start The starting coordinate
   * @param destination The destination coordinate
   * @return Shortest path from start to destination if one exist
   */
  RobotPath plan_path_to_target(const RobotCoordinate start, const RobotCoordinate destination);

private:
  Terrain* terrain; ///< Pointer to terrain object
  RobotCoordinate grid_size_; ///< size of terrain object in (width, height)

  /**
   * @brief Get the cell id of a coordinate
   * 
   * @param coord The coordinate
   * @return The cell index
   */
  std::size_t get_cell_id(const RobotCoordinate& coord) const;

  /**
   * @brief Get the terrain cost for a coordinate
   * 
   * @param coord The coordinate
   * @return The terrain cost for different levels 
   */
  int terrain_cost(const RobotCoordinate& coord) const;

  /**
   * @brief Calculate the heuristic cost between two coordinates
   * 
   * @param a The first coordinate
   * @param b The second coordinate
   * @return The heuristic cost
   */
  int heuristic(const RobotCoordinate& a, const RobotCoordinate& b) const;

  /**
   * @brief Check if a coordinate is valid
   * its invalid if coordinate is out of boundary of map, or if it is unknown/water region 
   * @param coord The coordinate
   * @return True If the coordinate is valid, false otherwise
   */
  bool is_valid(const RobotCoordinate& coord) const;

  /**
   * @brief Get the valid neighbors of a coordinate
   * max. 4 neighbors in 4 directions if all are valid
   * @param coord The coordinate
   * @return The valid neighbors
   */
  std::vector<RobotCoordinate> get_neighbors(const RobotCoordinate& coord) const;
  };
