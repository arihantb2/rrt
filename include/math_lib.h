#pragma once

#include <math.h>
#include <random>
#include <eigen3/Eigen/Dense>

namespace math_lib
{
    inline double euclideandist2(const Eigen::Vector2d &point1, const Eigen::Vector2d &point2 = Eigen::Vector2d::Zero())
    {
        return (point1 - point2).norm();
    }

    inline double cross2(const Eigen::Vector2d &point1, const Eigen::Vector2d &point2)
    {
        return (point1.x() * point2.y() - point1.y() * point2.x());
    }

    inline double dot2(const Eigen::Vector2d &point1, const Eigen::Vector2d &point2)
    {
        return point1.adjoint() * point2;
    }

    inline double dRand(double min, double max)
    {
        // Source: StackOverflow :P
        std::uniform_real_distribution<double> unif(min, max);
        std::default_random_engine re;
        return unif(re);
    }
}