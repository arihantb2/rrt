#include <rrt.h>

namespace planner
{
    unsigned int Node::idCounter_ = 0;

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
        tree_.reset();
    }

    bool RRT::findPath(const world::Vector2 &start, const world::Vector2 &goal)
    {
        assert(searchGridPtr->isPointValid(start) && "Start point is not contained inside the search space");
        assert(searchGridPtr->isPointValid(goal) && "Goal point is not contained inside the search space");

        reset();

        tree_.root_ = tree_.addNode(start);
        tree_.root_->parent_ = nullptr;
        tree_.root_->children_.clear();
        tree_.root_->cost_ = 0.0;

        numIterations_ = 0;
        while (numIterations_ < config_.maxIterations_)
        {
            world::Vector2 randomPoint = searchGridPtr->getRandomGridPoint();

            double dist;
            std::shared_ptr<Node> closestNode = getNodeClosestTo(randomPoint, dist);
            if (closestNode == nullptr)
                continue;

            world::Vector2 direction = randomPoint - closestNode->data_;
            double directionSize = direction.norm();
            if (directionSize < 1e-8)
                continue;
            direction = direction / directionSize;

            numIterations_++;

            double stepSize = math_lib::dRand(0.0, std::min(dist, config_.maxStepSize_));
            world::Vector2 newPoint = closestNode->data_ + direction * stepSize;
            newPoint = searchGridPtr->getGridPointClosestTo(newPoint);
            dist = math_lib::euclideandist2(newPoint, closestNode->data_);

            if (searchGridPtr->collisionCheck(world::Line2(closestNode->data_, newPoint)))
                continue;

            std::shared_ptr<Node> node;
            if (tree_.nodeExists(newPoint, node) == false)
            {
                node = tree_.addNode(newPoint);
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

            if (pathFound_ && numIterations_ > config_.minIterations_)
                break;
        }

        return pathFound_;
    }

    double RRT::getRandomStepSize()
    {
        return math_lib::dRand(config_.minStepSize_, config_.maxStepSize_);
    }

    std::shared_ptr<Node> RRT::getNodeClosestTo(const world::Vector2 &point, double &distance)
    {
        std::shared_ptr<Node> minNode = nullptr;
        double minHeuristic = std::numeric_limits<double>::max();
        for (auto node : tree_.nodes_)
        {
            double dist = math_lib::euclideandist2(node.second->data_, point);
            double heuristic = dist;
            if (heuristic < minHeuristic)
            {
                distance = dist;
                minHeuristic = heuristic;
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

    bool Tree::nodeExists(const world::Vector2 &point, std::shared_ptr<Node> &node) const
    {
        std::size_t hash = world::vectorHash(point);

        if (nodes_.find(hash) == nodes_.end())
            return false;

        node = nodes_.at(hash);
        return true;
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
}