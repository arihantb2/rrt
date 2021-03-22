#include <yaml-cpp/yaml.h>

#include <rrt.h>
#include <yaml_interface.h>

int main(int argc, char const *argv[])
{
    assert((argc == 2) && "Exactly one argument required after executable name. Usage: ./rrt <yaml-config-file-path>");

    world::ObstaclePtrMap obstacleMap = config_loader::loadObstacleMapFromYAML(std::string(argv[1]));
    world::SearchGrid2Config gridConfig = config_loader::loadGridConfigFromYAML(std::string(argv[1]));
    planner::RRTConfig rrtConfig = config_loader::loadRRTConfigFromYAML(std::string(argv[1]));

    for (auto obstacle : obstacleMap)
    {
        obstacle.second->print();
        std::cout << "\n";
    }

    gridConfig.print();
    std::cout << "\n";
    rrtConfig.print();
    std::cout << "\n";

    world::SearchGrid2 grid(gridConfig, obstacleMap);

    world::Vector2 startPose = config_loader::loadStartPoseFromYAML(std::string(argv[1]), grid);
    world::Vector2 goalPose = config_loader::loadGoalPoseFromYAML(std::string(argv[1]), grid);

    std::cout << "Start Pose: [" << startPose.transpose() << "]\n";
    std::cout << "Goal Pose : [" << goalPose.transpose() << "]\n";

    planner::RRT rrt(grid, rrtConfig);
    bool found = rrt.findPath(startPose, goalPose);
    if (found)
    {
        world::Path2 path = rrt.path();
        double distance = rrt.pathLength();
        std::cout << "Path found. Path length: " << distance << std::endl;
    }
    else
        std::cout << "Path not found\n";

    path_writer::writeRRTToYAML("config/rrt_output.yaml", rrt);

    return 0;
}
