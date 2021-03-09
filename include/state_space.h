/*
* Author: Arihant Lunawat
* File: state_space.h
* This file and its contents are confidential and owned by the author of this document. it is prohibited
* from usage by anyone other than the author
*/

#pragma once

#include <bits/stdc++.h>
#include <eigen3/Eigen/Dense>

namespace world
{
    typedef Eigen::Vector2d Vector2;
    typedef std::vector<world::Vector2> Path2;

    class StateSpace2
    {
    public:
        StateSpace2() : position_(Vector2::Zero()) {}
        StateSpace2(double x, double y) { position_ << x, y; }
        StateSpace2(const Vector2 &position) : position_(position) {}
        StateSpace2(const StateSpace2 &other) : position_(other.position()) {}

        ~StateSpace2() {}

        Vector2 &position() { return position_; }
        Vector2 position() const { return position_; }

        double &x() { return position_.x(); }
        double x() const { return position_.x(); }

        double &y() { return position_.y(); }
        double y() const { return position_.y(); }

    private:
        Vector2 position_;
    };

    class Node : public StateSpace2
    {
    public:
        Node() = delete;
        Node(const StateSpace2 &state) : StateSpace2(state), parent_(nullptr), cost_(-1.0) {}

    private:
        Node *parent_;
        double cost_;
    };

    class Tree
    {
    public:
        Tree() {}
        ~Tree() {}

    private:
        Node *root;
    };
}