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

        Vector2 xlim_;
        Vector2 ylim_;
        double xres_;
        double yres_;
    };

    class SearchGrid2
    {
    public:
        SearchGrid2() = delete;
        SearchGrid2(const SearchGrid2Config &config) : config_(config) {}
        SearchGrid2(const SearchGrid2 &other) : config_(other.config()) {}

        ~SearchGrid2() {}

        SearchGrid2Config config() { return config_; }
        SearchGrid2Config config() const { return config_; }

        bool isStateValid(const StateSpace2 &);
        bool isStateValid(const StateSpace2 &) const;

        Vector2 getGridPointClosestTo(const Vector2 &point);
        Vector2 getGridPointClosestTo(const Vector2 &point) const;
        Vector2 getRandomGridPoint();

    private:
        SearchGrid2Config config_;
    };
}