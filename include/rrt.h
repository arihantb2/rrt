/*
* Author: Arihant Lunawat
* File: rrt.h
* This file and its contents are confidential and owned by the author of this document. it is prohibited
* from usage by anyone other than the author
*/

#pragma once

#include <bits/stdc++.h>

#include <search_space.h>

namespace planner
{
    class RRT
    {
    public:
        RRT() {}
        ~RRT() {}

    private:
        std::shared_ptr<world::SearchGrid2> searchGridPtr;
        world::ObstacleList obstacles;
    };
}
