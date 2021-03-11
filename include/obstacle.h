/*
* Author: Arihant Lunawat
* File: obstacle.h
* This file and its contents are confidential and owned by the author of this document. it is prohibited
* from usage by anyone other than the author
*/

#pragma once

#include <bits/stdc++.h>
#include <eigen3/Eigen/Dense>

#include <math_lib.h>
#include <state_space.h>

namespace world
{
    enum class Orientation : int
    {
        CLOCKWISE = -1,
        COLINEAR = 0,
        COUNTERCLOCKWISE = 1,
    };

    static bool onSegment(const Vector2 &, const Vector2 &, const Vector2 &);
    static Orientation orientation(const Vector2 &, const Vector2 &, const Vector2 &);

    enum class ObstacleType : int
    {
        UNKNOWN = -1,
        CIRCLE = 0,
        LINE = 1,
    };

    static std::string to_string(ObstacleType type)
    {
        switch(type)
        {
            case ObstacleType::LINE:
                return std::string("LINE");
            case ObstacleType::CIRCLE:
                return std::string("CIRCLE");
            case ObstacleType::UNKNOWN:
            default:
                return std::string("UNKNOWN");
        }
    }

    class Obstacle
    {
    public:
        Obstacle() = delete;
        Obstacle(ObstacleType type) : type_(type),
                                      id_(idCounter_++) {}
        Obstacle(const Obstacle &other) : type_(other.type()),
                                          id_(other.id()) {}

        virtual ~Obstacle() {}

        unsigned int id() { return id_; }
        unsigned int id() const { return id_; }

        ObstacleType type() { return type_; }
        ObstacleType type() const { return type_; }

        virtual void print()
        {
            std::cout << "--------OBSTACLE CONFIG--------\n";
            std::cout << "ID  : [" << id_ << "]\n";
            std::cout << "Type: [" << to_string(type_) << "]\n";
        }

        virtual void print() const
        {
            std::cout << "--------OBSTACLE CONFIG--------\n";
            std::cout << "ID  : [" << id_ << "]\n";
            std::cout << "Type: [" << to_string(type_) << "]\n";
        }

        virtual bool isColliding(const Vector2 &, const Vector2 &) { return false; }
        virtual bool isColliding(const Vector2 &, const Vector2 &) const { return false; }

    private:
        static unsigned int idCounter_;
        unsigned int id_;
        ObstacleType type_;
    };

    class Circle : public Obstacle
    {
    public:
        Circle() = delete;
        Circle(const Vector2 &center, double radius) : center_(center),
                                                       radius_(radius),
                                                       Obstacle(ObstacleType::CIRCLE) {}
        Circle(const std::vector<double> &center, double radius) : radius_(radius), Obstacle(ObstacleType::CIRCLE)
        {
            assert(center.size() == 2);
            center_ << center[0], center[1];
        }
        Circle(const Circle &other) : center_(other.center()),
                                      radius_(other.radius()),
                                      Obstacle(other) {}

        ~Circle() {}

        Vector2 &center() { return center_; }
        Vector2 center() const { return center_; }

        double &radius() { return radius_; }
        double radius() const { return radius_; }

        virtual void print();
        virtual void print() const;

        virtual bool isColliding(const Vector2 &point1, const Vector2 &point2);
        virtual bool isColliding(const Vector2 &point1, const Vector2 &point2) const;

    private:
        Vector2 center_;
        double radius_;
    };

    class Line : public Obstacle
    {
    public:
        Line() = delete;
        Line(const Vector2 &point1, const Vector2 &point2) : Obstacle(ObstacleType::LINE),
                                                             point1_(point1),
                                                             point2_(point2) {}
        Line(const std::vector<double> &point1, const std::vector<double> &point2) : Obstacle(ObstacleType::LINE)
        {
            assert(point1.size() == 2);
            point1_ << point1[0], point1[1];

            assert(point2.size() == 2);
            point2_ << point2[0], point2[1];
        }
        Line(const Line &other) : Obstacle(other),
                                  point1_(other.point1()),
                                  point2_(other.point2()) {}

        ~Line() {}

        Vector2 &point1() { return point1_; }
        Vector2 point1() const { return point1_; }

        Vector2 &point2() { return point2_; }
        Vector2 point2() const { return point2_; }

        virtual void print();
        virtual void print() const;

        virtual bool isColliding(const Vector2 &, const Vector2 &);
        virtual bool isColliding(const Vector2 &, const Vector2 &) const;

    private:
        Vector2 point1_;
        Vector2 point2_;
    };

    typedef std::shared_ptr<Obstacle> ObstaclePtr;
    typedef std::vector<Obstacle> ObstacleList;
    typedef std::map<unsigned int, Obstacle> ObstacleMap;
    typedef std::map<unsigned int, ObstaclePtr> ObstaclePtrMap;
}