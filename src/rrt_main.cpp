#include <yaml-cpp/yaml.h>

#include <rrt.h>
#include <config_loader.h>

int main(int argc, char const *argv[])
{
    assert((argc == 2) && "Exactly one argument required after executable name. Usage: ./rrt <yaml-config-file-path>");

    YAML::Node config = YAML::LoadFile(argv[1]);
    assert(config.IsDefined() && config.IsMap() && "Error in reading YAML config file. Undefined or format mismatch");

    world::ObstaclePtrMap obstacleMap = config_loader::getObstacleMap(config);
    world::SearchGrid2Config gridConfig = config_loader::getGridConfig(config);
    planner::RRTConfig rrtConfig = config_loader::getRRTConfig(config);

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

    world::Vector2 startPose;
    try
    {
        std::vector<double> pose;
        if (config["start_pose"].IsSequence())
        {
            pose = config["start_pose"].as<std::vector<double>>();
            assert(pose.size() == 2);
            startPose << pose[0], pose[1];
        }

        else if (config["start_pose"].IsScalar() && std::string("generate_random").compare(config["start_pose"].as<std::string>()) == 0)
            startPose = grid.getRandomGridPoint();

        else
            startPose = grid.getRandomGridPoint();
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error in reading start_pose from YAML config file. Error: " << e.what() << "Generating random pose as input\n";
        startPose = grid.getRandomGridPoint();
    }

    world::Vector2 goalPose;
    try
    {
        std::vector<double> pose;
        if (config["goal_pose"].IsSequence())
        {
            pose = config["goal_pose"].as<std::vector<double>>();
            assert(pose.size() == 2);
            goalPose << pose[0], pose[1];
        }

        else if (config["goal_pose"].IsScalar() && std::string("generate_random").compare(config["goal_pose"].as<std::string>()) == 0)
            goalPose = grid.getRandomGridPoint();

        else
            goalPose = grid.getRandomGridPoint();
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error in reading start_pose from YAML config file. Error: " << e.what() << "Generating random pose as input\n";
        goalPose = grid.getRandomGridPoint();
    }

    std::cout << "Start Pose: [" << startPose.transpose() << "]\n";
    std::cout << "Goal Pose:  [" << goalPose.transpose() << "]\n";

    planner::RRT rrt(grid, rrtConfig);
    bool found = rrt.findPath(startPose, goalPose);
    if (found)
    {
        world::Path2 path = rrt.path();
        double distance = rrt.pathLength();
        std::cout << "path found: \n";
        for (unsigned int i = 0; i < path.size() - 1; i++)
            std::cout << "[" << path[i].transpose() << "] --> [" << path[i + 1].transpose() << "] distance: [" << math_lib::euclideandist2(path[i], path[i + 1]) << "] collision? " << std::boolalpha << grid.collisionCheck(path[i], path[i + 1]) << std::endl;
        std::cout << "path length: " << distance << std::endl;
    }
    else
        std::cout << "path not found\n";

    return 0;
}
