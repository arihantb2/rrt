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

    Vector2 SearchGrid2::getGridPointClosestTo(const Vector2 &point)
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
        return Vector2(math_lib::dRand(config_.xlim_[0], config_.xlim_[1]),
                       math_lib::dRand(config_.ylim_[0], config_.ylim_[1]));
    }
}