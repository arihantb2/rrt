#pragma once

#include <math.h>
#include <random>
#include <eigen3/Eigen/Dense>

namespace math_lib
{
    static double euclideandist2(const Eigen::Vector2d &point1, const Eigen::Vector2d &point2 = Eigen::Vector2d::Zero())
    {
        return (point1 - point2).norm();
    }

    static double cross2(const Eigen::Vector2d &point1, const Eigen::Vector2d &point2)
    {
        return (point1.x() * point2.y() - point1.y() * point2.x());
    }

    static double dot2(const Eigen::Vector2d &point1, const Eigen::Vector2d &point2)
    {
        return point1.adjoint() * point2;
    }

    static double dRand(double min, double max)
    {
        std::uniform_real_distribution<double> unif(min, max);
        std::random_device rd;
        std::mt19937 gen(rd());
        return unif(gen);
    }
}