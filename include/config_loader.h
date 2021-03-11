#pragma once

#include <yaml-cpp/yaml.h>
#include <bits/stdc++.h>
#include <search_space.h>
#include <state_space.h>
#include <obstacle.h>
#include <rrt.h>

namespace config_loader
{
    static world::ObstaclePtrMap getObstacleMap(const YAML::Node &config)
    {
        world::ObstaclePtrMap obstacleMap;
        try
        {
            YAML::Node obstacleConfig = config["obstacles"];
            assert(obstacleConfig.IsSequence() && "Error in reading obstacles from YAML config file. Format mismatch");
            for (auto obsConfig : obstacleConfig)
            {
                assert(obsConfig.IsMap() && "Error in reading obstacles from YAML config file. Format mismatch");
                assert(obsConfig["type"].IsScalar() && "Error in reading obstacle type from YAML config file. Format mismatch");

                std::string type = obsConfig["type"].as<std::string>();
                if (std::string("Line").compare(type) == 0)
                {
                    assert(obsConfig["start"].IsSequence() && "Error in reading line obstacle start config from YAML config file. Format mismatch");
                    assert(obsConfig["end"].IsSequence() && "Error in reading line obstacle end config  from YAML config file. Format mismatch");
                    world::ObstaclePtr line = std::make_shared<world::Line>(obsConfig["start"].as<std::vector<double>>(), obsConfig["end"].as<std::vector<double>>());
                    obstacleMap.insert(std::make_pair(line->id(), line));
                }
                else if (std::string("Circle").compare(type) == 0)
                {
                    assert(obsConfig["center"].IsSequence() && "Error in reading circle obstacle center config from YAML config file. Format mismatch");
                    assert(obsConfig["radius"].IsScalar() && "Error in reading circle obstacle radius from YAML config file. Format mismatch");
                    world::ObstaclePtr circle = std::make_shared<world::Circle>(obsConfig["center"].as<std::vector<double>>(), obsConfig["radius"].as<double>());
                    obstacleMap.insert(std::make_pair(circle->id(), circle));
                }
                else
                    std::cout << "Obstacle of type: [] found in config. Not recognized!! Allowed types are: [Line, Circle]\n";
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error in reading obstacles config. Error: " << e.what() << '\n';
        }

        return obstacleMap;
    }

    static world::SearchGrid2Config getGridConfig(const YAML::Node &config)
    {
        world::SearchGrid2Config gridConfig;
        try
        {
            YAML::Node gridNode = config["grid"];
            assert(gridNode.IsMap() && "Error in reading grid config from YAML config file. Format mismatch");

            assert(gridNode["xlimits"].IsSequence() && "Error in reading xlimits config from YAML config file. Format mismatch");
            std::vector<double> xlimits = gridNode["xlimits"].as<std::vector<double>>();
            assert(xlimits.size() == 2);

            assert(gridNode["ylimits"].IsSequence() && "Error in reading ylimits config from YAML config file. Format mismatch");
            std::vector<double> ylimits = gridNode["ylimits"].as<std::vector<double>>();
            assert(ylimits.size() == 2);

            assert(gridNode["spacing"].IsSequence() && "Error in reading spacing config from YAML config file. Format mismatch");
            std::vector<double> spacing = gridNode["spacing"].as<std::vector<double>>();
            assert(spacing.size() == 2);

            gridConfig.xlim_ << xlimits[0], xlimits[1];
            gridConfig.ylim_ << ylimits[0], ylimits[1];
            gridConfig.xres_ = spacing[0];
            gridConfig.yres_ = spacing[1];
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error in reading grid config. Error: " << e.what() << '\n';
            exit(0);
        }

        return gridConfig;
    }

    static planner::RRTConfig getRRTConfig(const YAML::Node &config)
    {
        planner::RRTConfig rrtConfig;
        try
        {
            YAML::Node rrtNode = config["rrt"];
            assert(rrtNode.IsMap() && "Error in reading rrt config from YAML config file. Format mismatch");

            assert(rrtNode["max_step_size"].IsScalar() && "Error in reading max_step_size config from YAML config file. Format mismatch");
            rrtConfig.maxStepSize_ = rrtNode["max_step_size"].as<double>();

            assert(rrtNode["min_iterations"].IsScalar() && "Error in reading min_iterations config from YAML config file. Format mismatch");
            rrtConfig.minIterations_ = rrtNode["min_iterations"].as<double>();

            assert(rrtNode["max_iterations"].IsScalar() && "Error in reading max_iterations config from YAML config file. Format mismatch");
            rrtConfig.maxIterations_ = rrtNode["max_iterations"].as<double>();

            assert(rrtNode["goal_closeness_threshold"].IsScalar() && "Error in reading goal_closeness_threshold config from YAML config file. Format mismatch");
            rrtConfig.goalClosenessThreshold_ = rrtNode["goal_closeness_threshold"].as<double>();
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }

        return rrtConfig;
    }
}