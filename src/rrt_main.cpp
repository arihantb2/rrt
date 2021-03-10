/*
* Author: Arihant Lunawat
* File: rrt_main.cpp
* This file and its contents are confidential and owned by the author of this document. it is prohibited
* from usage by anyone other than the author
*/

#include <rrt.h>
#include <yaml-cpp/yaml.h>

int main(int argc, char const *argv[])
{
    world::SearchGrid2Config gridConfig;

    gridConfig.xres_ = 1.0;
    gridConfig.yres_ = 1.0;

    world::ObstaclePtrMap obstacleMap;
    // world::ObstaclePtr lineObstacle = std::make_shared<world::Line>(world::Vector2(-100, 0), world::Vector2(100, 0));
    // obstacleMap.insert(std::make_pair(lineObstacle->id(), lineObstacle));
    world::ObstaclePtr circle = std::make_shared<world::Circle>(world::Vector2(0.0, 0.0), 25.0);
    obstacleMap.insert(std::make_pair(circle->id(), circle));

    world::SearchGrid2 grid(world::Vector2(-100.0, 100.0), world::Vector2(-100.0, 100.0), gridConfig, obstacleMap);

    planner::RRTConfig rrtConfig;
    rrtConfig.goalClosenessThreshold_ = 7.5;
    rrtConfig.minIterations_ = 100;
    rrtConfig.maxIterations_ = 1000;
    rrtConfig.maxSearchDistance_ = 40.0;

    world::Vector2 start = grid.getGridPointClosestTo(world::Vector2(-100.0, -100.0));
    world::Vector2 goal = grid.getGridPointClosestTo(world::Vector2(100.0, 100.0));

    std::cout << "Start Point: " << start.transpose() << std::endl;
    std::cout << "Goal Point:  " << goal.transpose() << std::endl;

    planner::RRT rrt(grid, rrtConfig);
    world::Path2 path;
    double distance;
    bool found = rrt.findPath(start, goal, path, distance);
    if (found && path.size() > 0)
    {
        std::cout << "path found: \n";
        // for (auto point : path)
        //     std::cout << "\t" << point.transpose() << std::endl;
        for (unsigned int i = 0; i < path.size() - 1; i++)
        {
            std::cout << "[" << path[i].transpose() << "] --> [" << path[i + 1].transpose() << "] distance: [" << math_lib::euclideandist2(path[i], path[i + 1]) << "] collision? " << std::boolalpha << grid.collisionCheck(path[i], path[i + 1]) << std::endl;
        }
        std::cout << "distance: " << distance << std::endl;
    }
    else
        std::cout << "path not found\n";

    return 0;
}
