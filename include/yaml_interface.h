#pragma once

#include <yaml-cpp/yaml.h>
#include <bits/stdc++.h>
#include <search_space.h>
#include <state_space.h>
#include <obstacle.h>
#include <rrt.h>

namespace config_loader
{
    static world::ObstaclePtrMap loadObstacleMapFromYAML(std::string filename)
    {
        YAML::Node config = YAML::LoadFile(filename);
        assert(config.IsDefined() && config.IsMap() && "Error in reading YAML config file. Undefined or format mismatch");

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
                    world::ObstaclePtr obstacle = std::make_shared<world::LineObstacle>(obsConfig["start"].as<std::vector<double>>(), obsConfig["end"].as<std::vector<double>>());
                    obstacleMap.insert(std::make_pair(obstacle->id(), obstacle));
                }
                else if (std::string("Circle").compare(type) == 0)
                {
                    assert(obsConfig["center"].IsSequence() && "Error in reading circle obstacle center config from YAML config file. Format mismatch");
                    assert(obsConfig["radius"].IsScalar() && "Error in reading circle obstacle radius from YAML config file. Format mismatch");
                    world::ObstaclePtr obstacle = std::make_shared<world::CircleObstacle>(obsConfig["center"].as<std::vector<double>>(), obsConfig["radius"].as<double>());
                    obstacleMap.insert(std::make_pair(obstacle->id(), obstacle));
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

    static world::SearchGrid2Config loadGridConfigFromYAML(std::string filename)
    {
        YAML::Node config = YAML::LoadFile(filename);
        assert(config.IsDefined() && config.IsMap() && "Error in reading YAML config file. Undefined or format mismatch");

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

    static planner::RRTConfig loadRRTConfigFromYAML(std::string filename)
    {
        YAML::Node config = YAML::LoadFile(filename);
        assert(config.IsDefined() && config.IsMap() && "Error in reading YAML config file. Undefined or format mismatch");

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

    static world::Vector2 loadStartPoseFromYAML(std::string filename, const world::SearchGrid2 &grid)
    {
        YAML::Node config = YAML::LoadFile(filename);
        assert(config.IsDefined() && config.IsMap() && "Error in reading YAML config file. Undefined or format mismatch");

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

        return startPose;
    }

    static world::Vector2 loadGoalPoseFromYAML(std::string filename, const world::SearchGrid2 &grid)
    {
        YAML::Node config = YAML::LoadFile(filename);
        assert(config.IsDefined() && config.IsMap() && "Error in reading YAML config file. Undefined or format mismatch");

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

        return goalPose;
    }
}

namespace path_writer
{
    void writePathToYAML(std::string filename, const world::Path2 &path)
    {
        using namespace YAML;

        Emitter out;

        out << BeginMap;
        out << Key << "path";
        out << Value;
        out << BeginSeq;
        for (const auto waypoint : path)
        {
            std::vector<double> point{waypoint[0], waypoint[1]};
            out << Flow << point;
        }
        out << EndSeq;
        out << EndMap;

        if (!out.good())
        {
            std::cout << out.GetLastError() << std::endl;
            return;
        }

        std::ofstream outWriter(filename);
        outWriter << out.c_str();
        outWriter.close();
    }

    void writeRRTToYAML(std::string filename, const planner::RRT &rrt)
    {
        using namespace YAML;

        Emitter out;

        std::shared_ptr<planner::Tree> tree = rrt.tree();

        out << BeginMap;
        out << Key << "tree";
        out << BeginSeq;
        for (auto node : tree->nodes_)
        {
            if (node.second->parent_ == nullptr)
                continue;

            out << BeginMap;
            out << Key << "id" << Value << node.second->id_;
            out << Key << "data" << Value << Flow << std::vector<double>{node.second->data_[0], node.second->data_[1]};
            out << Key << "parent" << Value << Flow << std::vector<double>{node.second->parent_->data_[0], node.second->parent_->data_[1]};
            out << Key << "cost" << Value << node.second->cost_;
            out << EndMap;
        }
        out << EndSeq;

        if (rrt.pathFound())
        {
            out << Key << "path";
            out << Value;
            out << BeginSeq;
            for (const auto waypoint : rrt.path())
            {
                std::vector<double> point{waypoint[0], waypoint[1]};
                out << Flow << point;
            }
            out << EndSeq;
        }

        out << EndMap;

        if (!out.good())
        {
            std::cout << out.GetLastError() << std::endl;
            return;
        }

        std::ofstream outWriter(filename);
        outWriter << out.c_str();
        outWriter.close();
    }
}