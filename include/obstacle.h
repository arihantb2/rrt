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

    static std::string to_string(ObstacleType type)
    {
        switch (type)
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
            std::cout << "\n--------OBSTACLE CONFIG--------\n";
            std::cout << "ID  : [" << id_ << "]\n";
            std::cout << "Type: [" << to_string(type_) << "]\n";
            std::cout << "-------------------------------\n";
        }

        virtual void print() const
        {
            std::cout << "\n--------OBSTACLE CONFIG--------\n";
            std::cout << "ID  : [" << id_ << "]\n";
            std::cout << "Type: [" << to_string(type_) << "]\n";
            std::cout << "-------------------------------\n";
        }

        virtual bool collisionCheck(const Line2 &) { return false; }
        virtual bool collisionCheck(const Line2 &) const { return false; }

    private:
        static unsigned int idCounter_;
        unsigned int id_;
        ObstacleType type_;
    };

    class CircleObstacle : public Obstacle, public Circle2
    {
    public:
        CircleObstacle() = delete;
        CircleObstacle(const Vector2 &center, double radius) : Obstacle(ObstacleType::CIRCLE),
                                                               Circle2(center, radius) {}
        CircleObstacle(const std::vector<double> &center, double radius) : Circle2(center, radius),
                                                                           Obstacle(ObstacleType::CIRCLE) {}
        CircleObstacle(const CircleObstacle &other) : Obstacle(other), Circle2(other) {}

        ~CircleObstacle() {}

        virtual void print()
        {
            Obstacle::print();
            Circle2::print();
        }

        virtual void print() const
        {
            Obstacle::print();
            Circle2::print();
        }

        virtual bool collisionCheck(const Line2 &);
        virtual bool collisionCheck(const Line2 &) const;
    };

    class LineObstacle : public Obstacle, public Line2
    {
    public:
        LineObstacle() = delete;
        LineObstacle(const Vector2 &point1, const Vector2 &point2) : Obstacle(ObstacleType::LINE), 
                                                                     Line2(point1, point2) {}
        LineObstacle(const std::vector<double> &point1, const std::vector<double> &point2) : Obstacle(ObstacleType::LINE), 
                                                                                             Line2(point1, point2) {}
        LineObstacle(const LineObstacle &other) : Obstacle(other), Line2(other) {}

        ~LineObstacle() {}

        virtual void print()
        {
            Obstacle::print();
            Line2::print();
        }

        virtual void print() const
        {
            Obstacle::print();
            Line2::print();
        }

        virtual bool collisionCheck(const Line2 &);
        virtual bool collisionCheck(const Line2 &) const;
    };

    typedef std::shared_ptr<Obstacle> ObstaclePtr;
    typedef std::vector<Obstacle> ObstacleList;
    typedef std::map<unsigned int, Obstacle> ObstacleMap;
    typedef std::map<unsigned int, ObstaclePtr> ObstaclePtrMap;
}