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
    typedef std::deque<world::Vector2> Path2;
}