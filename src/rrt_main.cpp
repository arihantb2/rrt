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

    gridConfig.xlim_ << -100.0, 100.0;
    gridConfig.ylim_ << -100.0, 100.0;
    gridConfig.xres_ = 1.0;
    gridConfig.yres_ = 1.0;

    world::SearchGrid2 grid(gridConfig);

    planner::RRT rrt;

    world::ObstaclePtr circle = std::make_shared<world::Circle>(world::Vector2(0.0, 0.0), 3.0);
    std::cout << "intersection test: " << std::boolalpha << circle->isColliding(world::Vector2(-5.0, 0.0), world::Vector2(-10.0, 0.0)) << std::endl;

    return 0;
}
