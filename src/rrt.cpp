/*
* Author: Arihant Lunawat
* File: rrt.cpp
* This file and its contents are confidential and owned by the author of this document. it is prohibited
* from usage by anyone other than the author
*/

#include <rrt.h>

namespace planner
{
    RRT::RRT(const world::SearchGrid2 &grid, const RRTConfig &config)
    {
        searchGridPtr = std::make_shared<world::SearchGrid2>(grid);
        config_ = config;

        reset();
    }

    void RRT::reset()
    {
        numIterations_ = 0;
        goalNode_ = nullptr;
        pathFound_ = false;
    }

    bool RRT::findPath(const world::Vector2 &start, const world::Vector2 &goal, world::Path2 &path, double &distance)
    {
        assert(searchGridPtr->isPointValid(start) && "Start point is not contained inside the search space");
        assert(searchGridPtr->isPointValid(goal) && "Goal point is not contained inside the search space");

        reset();

        Tree tree;
        tree.root_ = tree.addNode(start);
        tree.root_->parent_ = nullptr;
        tree.root_->children_.clear();
        tree.root_->cost_ = 0.0;

        numIterations_ = 0;
        while (numIterations_ < config_.maxIterations_)
        {
            numIterations_++;

            world::Vector2 randomPoint = searchGridPtr->getRandomGridPoint();

            double dist;
            std::shared_ptr<Node> closestNode = tree.getNodeClosestTo(randomPoint, dist);

            if (dist > config_.maxSearchDistance_)
            {
                randomPoint = (config_.maxSearchDistance_ / dist) * (randomPoint - closestNode->data_) + closestNode->data_;
                randomPoint = searchGridPtr->getGridPointClosestTo(randomPoint);
                dist = math_lib::euclideandist2(randomPoint, closestNode->data_);
            }

            if (searchGridPtr->collisionCheck(closestNode->data_, randomPoint))
                continue;

            std::shared_ptr<Node> node;
            if (tree.nodeExists(randomPoint, node) == false)
            {
                node = tree.addNode(randomPoint);
                node->parent_ = closestNode;
                closestNode->children_.push_back(node);
                node->cost_ = closestNode->cost_ + dist;
            }

            else if (closestNode->cost_ + dist < node->cost_)
            {
                closestNode->children_.push_back(node);
                node->parent_ = closestNode;
                node->cost_ = closestNode->cost_ + dist;
            }

            if (math_lib::euclideandist2(randomPoint, goal) <= config_.goalClosenessThreshold_)
            {
                pathFound_ = true;
                goalNode_ = node;
                if (numIterations_ > config_.minIterations_)
                    break;
            }
        }

        if (pathFound_)
        {
            distance = goalNode_->cost_;
            while (goalNode_ != nullptr)
            {
                path.push_front(goalNode_->data_);
                goalNode_ = goalNode_->parent_;
            }
        }

        else
            distance = std::numeric_limits<double>::max();

        return pathFound_;
    }

    std::shared_ptr<Node> Tree::getNodeClosestTo(const world::Vector2 &point, double &minDist)
    {
        std::shared_ptr<Node> minNode;
        minDist = std::numeric_limits<double>::max();
        for (auto node : nodes_)
        {
            double dist = math_lib::euclideandist2(node.second->data_, point);
            if (dist < minDist)
            {
                minDist = dist;
                minNode = node.second;
            }
        }

        return minNode;
    }

    std::shared_ptr<Node> Tree::addNode(const world::Vector2 &point)
    {
        std::size_t hash = world::vectorHash(point);
        std::shared_ptr<Node> node = std::make_shared<Node>(point);
        nodes_.insert(std::make_pair(hash, node));

        return node;
    }

    bool Tree::nodeExists(const world::Vector2 &point, std::shared_ptr<Node> &node)
    {
        std::size_t hash = world::vectorHash(point);

        if (nodes_.find(hash) == nodes_.end())
            return false;

        node = nodes_.at(hash);
        return true;
    }
}