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

    bool RRT::findPath(const world::Vector2 &start, const world::Vector2 &goal)
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

            double stepSize = math_lib::dRand(0.0, config_.maxStepSize_);

            world::Vector2 direction = randomPoint - closestNode->data_;
            double size = direction.norm();
            direction = direction / size;

            world::Vector2 newPoint = searchGridPtr->getGridPointClosestTo(closestNode->data_ + direction * stepSize);
            dist = math_lib::euclideandist2(newPoint, closestNode->data_);

            if (searchGridPtr->collisionCheck(world::Line2(closestNode->data_, newPoint)))
                continue;

            std::shared_ptr<Node> node;
            if (tree.nodeExists(newPoint, node) == false)
            {
                node = tree.addNode(newPoint);
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

            if (math_lib::euclideandist2(newPoint, goal) <= config_.goalClosenessThreshold_)
            {
                pathFound_ = true;
                goalNode_ = node;
                if (numIterations_ > config_.minIterations_)
                    break;
            }
        }

        return pathFound_;
    }

    world::Path2 RRT::path()
    {
        world::Path2 path;
        if (pathFound_)
        {
            std::shared_ptr<Node> node = goalNode_;
            while (node != nullptr)
            {
                path.push_front(node->data_);
                node = node->parent_;
            }
        }

        return path;
    }

    world::Path2 RRT::path() const
    {
        world::Path2 path;
        if (pathFound_)
        {
            std::shared_ptr<Node> node = goalNode_;
            while (node != nullptr)
            {
                path.push_front(node->data_);
                node = node->parent_;
            }
        }

        return path;
    }

    double RRT::pathLength()
    {
        if (pathFound_)
            return goalNode_->cost_;

        return -1.0;
    }

    double RRT::pathLength() const
    {
        if (pathFound_)
            return goalNode_->cost_;

        return -1.0;
    }

    std::shared_ptr<Node> Tree::getNodeClosestTo(const world::Vector2 &point, double &distance)
    {
        std::shared_ptr<Node> minNode;
        double heuristicDist = std::numeric_limits<double>::max();
        for (auto node : nodes_)
        {
            double dist = math_lib::euclideandist2(node.second->data_, point);
            if (dist < heuristicDist)
            {
                distance = dist;
                heuristicDist = dist;
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