/*
* Author: Arihant Lunawat
* File: search_space.cpp
* This file and its contents are confidential and owned by the author of this document. it is prohibited
* from usage by anyone other than the author
*/

#include <search_space.h>

namespace world
{
    unsigned int Obstacle::idCounter_ = 0;

    bool SearchGrid2::isPointValid(const Vector2 &point)
    {
        if (point.x() < xlim_[0] || point.x() > xlim_[1])
            return false;

        if (point.y() < ylim_[0] || point.y() > ylim_[1])
            return false;

        return true;
    }

    Vector2 SearchGrid2::getGridPointClosestTo(const Vector2 &point)
    {
        Vector2 gridPoint;

        assert(point.x() >= xlim_[0]);
        assert(point.x() <= xlim_[1]);
        assert(point.y() >= ylim_[0]);
        assert(point.y() <= ylim_[1]);

        gridPoint.x() = config_.xres_ * round(point.x() / config_.xres_);
        gridPoint.y() = config_.yres_ * round(point.y() / config_.yres_);

        return gridPoint;
    }

    Vector2 SearchGrid2::getGridPointClosestTo(const Vector2 &point) const
    {
        Vector2 gridPoint;

        assert(point.x() >= xlim_[0]);
        assert(point.x() <= xlim_[1]);
        assert(point.y() >= ylim_[0]);
        assert(point.y() <= ylim_[1]);

        gridPoint.x() = config_.xres_ * round(point.x() / config_.xres_);
        gridPoint.y() = config_.yres_ * round(point.y() / config_.yres_);

        return gridPoint;
    }

    Vector2 SearchGrid2::getRandomGridPoint()
    {
        return getGridPointClosestTo(Vector2(math_lib::dRand(xlim_[0], xlim_[1]), math_lib::dRand(ylim_[0], ylim_[1])));
    }

    Vector2 SearchGrid2::getRandomGridPoint() const
    {
        return getGridPointClosestTo(Vector2(math_lib::dRand(xlim_[0], xlim_[1]), math_lib::dRand(ylim_[0], ylim_[1])));
    }

    bool SearchGrid2::collisionCheck(const Vector2 &point1, const Vector2 &point2)
    {
        for (auto obstacle : obstacles_)
        {
            if (obstacle.second->type() == world::ObstacleType::LINE)
            {
                std::shared_ptr<world::Line> line = std::dynamic_pointer_cast<world::Line>(obstacle.second);
                if (line->isColliding(point1, point2))
                    return true;
            }

            else if (obstacle.second->type() == world::ObstacleType::CIRCLE)
            {
                std::shared_ptr<world::Circle> circle = std::dynamic_pointer_cast<world::Circle>(obstacle.second);
                if (circle->isColliding(point1, point2))
                    return true;
            }
        }

        return false;
    }
}