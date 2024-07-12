# Project Description

This project implements an A* pathfinding algorithm to find the shortest path on a terrain represented by a `.ppm` object. The goal is to navigate from a starting point to a destination while considering different terrain costs.

## A* Implementation

The A* algorithm is a widely used for pathfinding and graph traversal. It uses best-first search strategy to explore the graph. It is efficient and guarantees the shortest path if the heuristic is admissible.

It always expands towards minumum value for the cost function which is defined as f(n)= g(n) + h(n). where g(n) is exact cost of the path from starting node to current node and h(n) is heuristic cost. In this case heuristic is a manhatten distance between current node to the Goal.

### Terrain Representation

The terrain is represented as a grid, with each cell having a specific cost based on the type of terrain (e.g., water, different cost levels). The algorithm takes these costs into account when finding the shortest path.

### Files

- `Planner.h` and `Planner.cpp`: Contains the main A* algorithm implementation and its helper functions.
- `Terrain.h` and `Terrain.cpp`: Contains class to represent the terrain and methods to get structure of terrain and particular cell's TerrainValue.

### Key Concepts

- **Open List**: A priority queue that stores nodes to be evaluated, sorted by the total estimated cost (`f_cost`). Node with the lowest cost is processed first.
- **Closed List**: A set of nodes that have already been visited once.
- **Heuristic Function**: Estimates the minimum distance from the current node to the destination. Here, using Manhattan distance.
- **Cost Function**: Represents the actual cost to reach a node from the start node.

### Code Documentation

Code documentation is done using Doxygen generator tool. It can be found at [documentation](html/index.html). It can be open in any browser.

## Build the project

```bash
mkdir build && cd build
cmake .. && make
```

## Usage

Enter start and destination coordinates values inside `main.cpp` and build the project.

Inside `build` directory run the program.

```bash
./motion_planner ../terrain.ppm ../output.ppm
```
