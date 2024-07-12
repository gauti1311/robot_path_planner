cmake_minimum_required(VERSION 3.14) # FetchContent is available in 3.11 onwards, but 3.14 is recommended
project(MotionPlanningChallenge)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(motion_planner main.cpp  Planner.cpp Terrain.cpp)
