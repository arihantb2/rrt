/*
* Author: Arihant Lunawat
* File: rrt.h
* This file and its contents are confidential and owned by the author of this document. it is prohibited
* from usage by anyone other than the author
*/

#pragma once

#include <bits/stdc++.h>

#include <search_space.h>
#include <state_space.h>

namespace planner
{
    class Node
    {
    public:
        Node() = delete;
        Node(const world::Vector2 &data) : data_(data), parent_(nullptr), cost_(std::numeric_limits<double>::max()) {}
        Node(const Node& other) : data_(other.data_), parent_(other.parent_), cost_(other.cost_), children_(other.children_) {}
        ~Node()
        {
            children_.clear();
        }

        world::Vector2 data_;
        double cost_;

        std::shared_ptr<Node> parent_;
        std::vector<std::shared_ptr<Node>> children_;
    };

    class Tree
    {
    public:
        Tree() {}
        ~Tree()
        {
            nodes_.clear();
        }

        std::shared_ptr<Node> getNodeClosestTo(const world::Vector2 &, double &);
        bool nodeExists(const world::Vector2 &, std::shared_ptr<Node> &);
        std::shared_ptr<Node> addNode(const world::Vector2 &);

        std::shared_ptr<Node> root_;
        std::map<std::size_t, std::shared_ptr<Node>> nodes_;
    };

    class RRTConfig
    {
    public:
        RRTConfig()
        {
            maxSearchDistance_ = 20.0;
            maxIterations_ = 100;
            minIterations_ = 10;
            goalClosenessThreshold_ = 1.0;
        }

        RRTConfig(const RRTConfig &other)
        {
            maxSearchDistance_ = other.maxSearchDistance_;
            maxIterations_ = other.maxIterations_;
            minIterations_ = other.minIterations_;
            goalClosenessThreshold_ = other.goalClosenessThreshold_;
        }

        void print()
        {
            std::cout << "--------RRT CONFIG--------\n";
            std::cout << "Max Iterations          : [" << maxIterations_ << "]\n";
            std::cout << "Min Iterations          : [" << minIterations_ << "]\n";
            std::cout << "Max Search Distance     : [" << maxSearchDistance_ << "]\n";
            std::cout << "Goal Closeness Threshold: [" << goalClosenessThreshold_ << "]\n";
        }

        double maxSearchDistance_;
        unsigned int maxIterations_;
        unsigned int minIterations_;
        double goalClosenessThreshold_;
    };

    class RRT
    {
    public:
        RRT() = delete;
        RRT(const world::SearchGrid2 &, const RRTConfig &);
        ~RRT() {}

        void reset();
        bool findPath(const world::Vector2 &, const world::Vector2 &, world::Path2 &, double &);

    private:
        std::shared_ptr<world::SearchGrid2> searchGridPtr;
        RRTConfig config_;
        unsigned int numIterations_;
        bool pathFound_;
        std::shared_ptr<Node> goalNode_;
    };
}
