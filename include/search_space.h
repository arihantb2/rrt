#pragma once

#include <bits/stdc++.h>
#include <eigen3/Eigen/Dense>

#include <obstacle.h>
#include <state_space.h>

namespace world
{
    static size_t vectorHash(const Vector2 &point)
    {
        // Source: https://stackoverflow.com/questions/2590677/how-do-i-combine-hash-values-in-c0x
        size_t seed = 0;
        std::hash<double> hasher;

        seed ^= hasher(point.x()) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= hasher(point.y()) + 0x9e3779b9 + (seed << 6) + (seed >> 2);

        return seed;
    }

    class SearchGrid2Config
    {
    public:
        SearchGrid2Config()
        {
            xlim_ << -100.0, 100.0;
            ylim_ << -100.0, 100.0;
            xres_ = 1.0;
            yres_ = 1.0;
        }

        SearchGrid2Config(const SearchGrid2Config &other)
        {
            xlim_ = other.xlim_;
            ylim_ = other.ylim_;
            xres_ = other.xres_;
            yres_ = other.yres_;
        }

        ~SearchGrid2Config() {}

        void print()
        {
            std::cout << "\n--------SEARCH GRID CONFIG--------\n";
            std::cout << "X limits: [" << xlim_.transpose() << "]\n";
            std::cout << "Y limits: [" << ylim_.transpose() << "]\n";
            std::cout << "Spacing : [" << xres_ << " " << yres_ << "]\n";
            std::cout << "----------------------------------\n";
        }

        void print() const
        {
            std::cout << "\n--------SEARCH GRID CONFIG--------\n";
            std::cout << "X limits: [" << xlim_.transpose() << "]\n";
            std::cout << "Y limits: [" << ylim_.transpose() << "]\n";
            std::cout << "Spacing : [" << xres_ << " " << yres_ << "]\n";
            std::cout << "----------------------------------\n";
        }

        Vector2 xlim_;
        Vector2 ylim_;
        double xres_;
        double yres_;
    };

    class SearchGrid2
    {
    public:
        SearchGrid2() = delete;
        SearchGrid2(const SearchGrid2Config &config,
                    const ObstaclePtrMap &obstacles = ObstaclePtrMap()) : config_(config),
                                                                          obstacles_(obstacles) {}
        SearchGrid2(const SearchGrid2 &other) : config_(other.config()),
                                                obstacles_(other.obstacles()) {}

        ~SearchGrid2() {}

        ObstaclePtrMap &obstacles() { return obstacles_; }
        ObstaclePtrMap obstacles() const { return obstacles_; }

        SearchGrid2Config config() { return config_; }
        SearchGrid2Config config() const { return config_; }

        bool isPointValid(const Vector2 &);
        bool isPointValid(const Vector2 &) const;

        bool collisionCheck(const Line2 &);
        bool collisionCheck(const Line2 &) const;

        Vector2 getGridPointClosestTo(const Vector2 &point);
        Vector2 getGridPointClosestTo(const Vector2 &point) const;

        Vector2 getRandomGridPoint();
        Vector2 getRandomGridPoint() const;

    private:
        SearchGrid2Config config_;
        world::ObstaclePtrMap obstacles_;
    };
}