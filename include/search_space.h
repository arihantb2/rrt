/*
* Author: Arihant Lunawat
* File: search_space.h
* This file and its contents are confidential and owned by the author of this document. it is prohibited
* from usage by anyone other than the author
*/

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
            xres_ = 1.0;
            yres_ = 1.0;
        }

        SearchGrid2Config(const SearchGrid2Config &other)
        {
            xres_ = other.xres_;
            yres_ = other.yres_;
        }

        ~SearchGrid2Config() {}

        double xres_;
        double yres_;
    };

    class SearchGrid2
    {
    public:
        SearchGrid2() = delete;
        SearchGrid2(const Vector2 &xlim,
                    const Vector2 &ylim,
                    const SearchGrid2Config &config,
                    const ObstaclePtrMap &obstacles = ObstaclePtrMap()) : config_(config),
                                                                          xlim_(xlim),
                                                                          ylim_(ylim),
                                                                          obstacles_(obstacles) {}
        SearchGrid2(const SearchGrid2 &other) : config_(other.config()),
                                                xlim_(other.xlim()),
                                                ylim_(other.ylim()),
                                                obstacles_(other.obstacles()) {}

        ~SearchGrid2() {}

        Vector2 &xlim() { return xlim_; }
        Vector2 xlim() const { return xlim_; }

        Vector2 &ylim() { return ylim_; }
        Vector2 ylim() const { return ylim_; }

        ObstaclePtrMap &obstacles() { return obstacles_; }
        ObstaclePtrMap obstacles() const { return obstacles_; }

        SearchGrid2Config config() { return config_; }
        SearchGrid2Config config() const { return config_; }

        bool isPointValid(const Vector2 &);
        bool isPointValid(const Vector2 &) const;

        bool collisionCheck(const Vector2 &, const Vector2 &);
        bool collisionCheck(const Vector2 &, const Vector2 &) const;

        Vector2 getGridPointClosestTo(const Vector2 &point);
        Vector2 getGridPointClosestTo(const Vector2 &point) const;

        Vector2 getRandomGridPoint();
        Vector2 getRandomGridPoint() const;

    private:
        SearchGrid2Config config_;
        world::ObstaclePtrMap obstacles_;
        Vector2 xlim_;
        Vector2 ylim_;
    };
}