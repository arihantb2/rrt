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
    enum class ObstacleType : int
    {
        UNKNOWN = -1,
        CIRCLE = 0,
        LINE = 1,
    };

    enum class ORIENTATION : int
    {
        CLOCKWISE = -1,
        COLINEAR = 0,
        COUNTERCLOCKWISE = 1,
    };

    static bool onSegment(const Vector2 &, const Vector2 &, const Vector2 &);
    static ORIENTATION orientation(const Vector2 &, const Vector2 &, const Vector2 &);

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

        virtual bool isColliding(const Vector2 &, const Vector2 &) { return false; }

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
        Circle(const Circle &other) : center_(other.center()),
                                      radius_(other.radius()),
                                      Obstacle(other) {}

        ~Circle() {}

        Vector2 &center() { return center_; }
        Vector2 center() const { return center_; }

        double &radius() { return radius_; }
        double radius() const { return radius_; }

        virtual bool isColliding(const Vector2 &point1, const Vector2 &point2);

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
        Line(const Line &other) : Obstacle(other),
                                  point1_(other.point1()),
                                  point2_(other.point2()) {}

        ~Line() {}

        Vector2 &point1() { return point1_; }
        Vector2 point1() const { return point1_; }

        Vector2 &point2() { return point2_; }
        Vector2 point2() const { return point2_; }

        bool isColliding(const Vector2 &, const Vector2 &);

    private:
        Vector2 point1_;
        Vector2 point2_;
    };

    typedef std::shared_ptr<Obstacle> ObstaclePtr;
    typedef std::vector<Obstacle> ObstacleList;
    typedef std::map<unsigned int, Obstacle> ObstacleMap;
    typedef std::map<unsigned int, ObstaclePtr> ObstaclePtrMap;
}