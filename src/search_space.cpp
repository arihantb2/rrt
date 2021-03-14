#include <search_space.h>

namespace world
{
    bool SearchGrid2::isPointValid(const Vector2 &point)
    {
        if (point.x() < config_.xlim_[0] || point.x() > config_.xlim_[1])
            return false;

        if (point.y() < config_.ylim_[0] || point.y() > config_.ylim_[1])
            return false;

        return true;
    }

    Vector2 SearchGrid2::getGridPointClosestTo(const Vector2 &point)
    {
        Vector2 gridPoint;

        if (point.x() <= config_.xlim_[0])
            gridPoint.x() = config_.xlim_[0];
        else if (point.x() >= config_.xlim_[1])
            gridPoint.x() = config_.xlim_[1];
        
        if (point.y() <= config_.ylim_[0])
            gridPoint.y() = config_.ylim_[0];
        else if (point.y() >= config_.ylim_[1])
            gridPoint.y() = config_.ylim_[1];

        gridPoint.x() = config_.xres_ * round(point.x() / config_.xres_);
        gridPoint.y() = config_.yres_ * round(point.y() / config_.yres_);

        return gridPoint;
    }

    Vector2 SearchGrid2::getGridPointClosestTo(const Vector2 &point) const
    {
        Vector2 gridPoint;

        assert(point.x() >= config_.xlim_[0]);
        assert(point.x() <= config_.xlim_[1]);
        assert(point.y() >= config_.ylim_[0]);
        assert(point.y() <= config_.ylim_[1]);

        gridPoint.x() = config_.xres_ * round(point.x() / config_.xres_);
        gridPoint.y() = config_.yres_ * round(point.y() / config_.yres_);

        return gridPoint;
    }

    Vector2 SearchGrid2::getRandomGridPoint()
    {
        return getGridPointClosestTo(Vector2(math_lib::dRand(config_.xlim_[0], config_.xlim_[1]), math_lib::dRand(config_.ylim_[0], config_.ylim_[1])));
    }

    Vector2 SearchGrid2::getRandomGridPoint() const
    {
        return getGridPointClosestTo(Vector2(math_lib::dRand(config_.xlim_[0], config_.xlim_[1]), math_lib::dRand(config_.ylim_[0], config_.ylim_[1])));
    }

    bool SearchGrid2::collisionCheck(const Line2 &line)
    {
        for (auto obstacle : obstacles_)
        {
            if (obstacle.second->type() == world::ObstacleType::LINE)
            {
                std::shared_ptr<world::LineObstacle> lineObstacle = std::dynamic_pointer_cast<world::LineObstacle>(obstacle.second);
                if (lineObstacle->collisionCheck(line))
                    return true;
            }

            else if (obstacle.second->type() == world::ObstacleType::CIRCLE)
            {
                std::shared_ptr<world::CircleObstacle> circleOstacle = std::dynamic_pointer_cast<world::CircleObstacle>(obstacle.second);
                if (circleOstacle->collisionCheck(line))
                    return true;
            }
        }

        return false;
    }
}