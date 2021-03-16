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
        Node(const world::Vector2 &data) : data_(data), parent_(nullptr), cost_(std::numeric_limits<double>::max()), id_(idCounter_++) {}
        Node(const Node &other) : data_(other.data_), parent_(other.parent_), cost_(other.cost_), children_(other.children_), id_(idCounter_++) {}
        ~Node()
        {
            children_.clear();
        }

        world::Vector2 data_;
        double cost_;

        static unsigned int idCounter_;
        unsigned int id_;
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
            root_.reset();
            root_ = nullptr;
        }

        void reset()
        {
            nodes_.clear();
            root_.reset();
            root_ = nullptr;
        }

        bool nodeExists(const world::Vector2 &, std::shared_ptr<Node> &);
        bool nodeExists(const world::Vector2 &, std::shared_ptr<Node> &) const;

        std::shared_ptr<Node> addNode(const world::Vector2 &);

        std::shared_ptr<Node> root_;
        std::map<std::size_t, std::shared_ptr<Node>> nodes_;
    };

    class RRTConfig
    {
    public:
        RRTConfig()
        {
            maxStepSize_ = 20.0;
            maxIterations_ = 100;
            minIterations_ = 10;
            goalClosenessThreshold_ = 1.0;
        }

        RRTConfig(const RRTConfig &other)
        {
            maxStepSize_ = other.maxStepSize_;
            maxIterations_ = other.maxIterations_;
            minIterations_ = other.minIterations_;
            goalClosenessThreshold_ = other.goalClosenessThreshold_;
        }

        void print()
        {
            std::cout << "\n--------RRT CONFIG--------\n";
            std::cout << "Max Iterations          : [" << maxIterations_ << "]\n";
            std::cout << "Min Iterations          : [" << minIterations_ << "]\n";
            std::cout << "Max Step Size           : [" << maxStepSize_ << "]\n";
            std::cout << "Goal Closeness Threshold: [" << goalClosenessThreshold_ << "]\n";
            std::cout << "--------------------------\n";
        }

        void print() const
        {
            std::cout << "\n--------RRT CONFIG--------\n";
            std::cout << "Max Iterations          : [" << maxIterations_ << "]\n";
            std::cout << "Min Iterations          : [" << minIterations_ << "]\n";
            std::cout << "Max Step Size           : [" << maxStepSize_ << "]\n";
            std::cout << "Goal Closeness Threshold: [" << goalClosenessThreshold_ << "]\n";
            std::cout << "--------------------------\n";
        }

        double maxStepSize_;
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

        bool pathFound() { return pathFound_; }
        bool pathFound() const { return pathFound_; }

        world::Path2 path();
        world::Path2 path() const;

        double pathLength();
        double pathLength() const;

        std::shared_ptr<Tree> tree() { return std::make_shared<Tree>(tree_); }
        std::shared_ptr<Tree> tree() const { return std::make_shared<Tree>(tree_); }

        void reset();
        bool findPath(const world::Vector2 &, const world::Vector2 &);

        std::shared_ptr<Node> getNodeClosestTo(const world::Vector2 &, double &);

    private:
        Tree tree_;
        std::shared_ptr<world::SearchGrid2> searchGridPtr;
        RRTConfig config_;
        unsigned int numIterations_;
        bool pathFound_;
        std::shared_ptr<Node> goalNode_;
    };
}
