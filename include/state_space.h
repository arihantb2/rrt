#pragma once

#include <bits/stdc++.h>
#include <eigen3/Eigen/Dense>

namespace world
{
    typedef Eigen::Vector2d Vector2;
    typedef std::deque<world::Vector2> Path2;

    class Line2
    {
    public:
        Line2() = delete;
        Line2(const Vector2 &point1, const Vector2 &point2 = Vector2::Zero()) : point1_(point1), point2_(point2) {}
        Line2(const std::vector<double> &point1, const std::vector<double> &point2 = std::vector<double>{0.0, 0.0})
        {
            assert(point1.size() == 2);
            point1_ << point1[0], point1[1];

            assert(point2.size() == 2);
            point2_ << point2[0], point2[1];
        }
        Line2(const Line2 &other) : point1_(other.point1()),
                                    point2_(other.point2()) {}

        ~Line2() {}

        Vector2 &point1() { return point1_; }
        Vector2 point1() const { return point1_; }

        Vector2 &point2() { return point2_; }
        Vector2 point2() const { return point2_; }

        virtual void print()
        {
            std::cout << "\n--------LINE CONFIG--------\n";
            std::cout << "Start: [" << point1_.transpose() << "]\n";
            std::cout << "End  : [" << point2_.transpose() << "]\n";
            std::cout << "---------------------------\n";
        }
        virtual void print() const
        {
            std::cout << "\n--------LINE CONFIG--------\n";
            std::cout << "Start: [" << point1_.transpose() << "]\n";
            std::cout << "End  : [" << point2_.transpose() << "]\n";
            std::cout << "---------------------------\n";
        }

        bool onLine(const Vector2 &point)
        {
            double cross = math_lib::cross2(point - point1_, point2_ - point1_);
            double dot = math_lib::dot2(point - point1_, point2_ - point1_);

            if (cross == 0.0 && dot > 0.0)
                return true;

            return false;
        }
        bool onLine(const Vector2 &point) const
        {
            double cross = math_lib::cross2(point - point1_, point2_ - point1_);
            double dot = math_lib::dot2(point - point1_, point2_ - point1_);

            if (cross == 0.0 && dot > 0.0)
                return true;

            return false;
        }

        Vector2 point1_;
        Vector2 point2_;
    };

    class Circle2
    {
    public:
        Circle2() = delete;
        Circle2(const Vector2 &center, double radius) : center_(center),
                                                        radius_(radius) {}
        Circle2(const std::vector<double> &center, double radius) : radius_(radius)
        {
            assert(center.size() == 2);
            center_ << center[0], center[1];
        }
        Circle2(const Circle2 &other) : center_(other.center()),
                                        radius_(other.radius()) {}

        ~Circle2() {}

        Vector2 &center() { return center_; }
        Vector2 center() const { return center_; }

        double &radius() { return radius_; }
        double radius() const { return radius_; }

        virtual void print()
        {
            std::cout << "\n--------CIRCLE CONFIG--------\n";
            std::cout << "Center: [" << center_.transpose() << "]\n";
            std::cout << "Radius: [" << radius_ << "]\n";
            std::cout << "-----------------------------\n";
        }
        virtual void print() const
        {
            std::cout << "\n--------CIRCLE CONFIG--------\n";
            std::cout << "Center: [" << center_.transpose() << "]\n";
            std::cout << "Radius: [" << radius_ << "]\n";
            std::cout << "-----------------------------\n";
        }

        Vector2 center_;
        double radius_;
    };
}