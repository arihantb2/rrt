#include <obstacle.h>

namespace world
{
    unsigned int Obstacle::idCounter_ = 0;

    void Circle::print()
    {
        std::cout << "\n--------CIRCLE OBSTACLE CONFIG--------\n";
        std::cout << "ID    : [" << id() << "]\n";
        std::cout << "Type  : [" << to_string(type()) << "]\n";
        std::cout << "Center: [" << center_.transpose() << "]\n";
        std::cout << "Radius: [" << radius_ << "]\n";
        std::cout << "--------------------------------------\n";
    }

    void Circle::print() const
    {
        std::cout << "\n--------CIRCLE OBSTACLE CONFIG--------\n";
        std::cout << "ID    : [" << id() << "]\n";
        std::cout << "Type  : [" << to_string(type()) << "]\n";
        std::cout << "Center: [" << center_.transpose() << "]\n";
        std::cout << "Radius: [" << radius_ << "]\n";
        std::cout << "--------------------------------------\n";
    }

    bool Circle::isColliding(const Vector2 &point1, const Vector2 &point2)
    {
        double distanceSq = math_lib::dot2(point1 - point2, point1 - point2);
        if (distanceSq <= 1e-8)
            return false;

        double r = math_lib::dot2(point1 - point2, point1 - center_) / distanceSq;
        r = std::max(0.0, std::min(r, 1.0));

        Vector2 closestPointToCenter = r * (point2 - point1) + point1;
        double distToCenter = math_lib::dot2(closestPointToCenter - center_, closestPointToCenter - center_);

        return (distToCenter <= radius_);
    }

    bool Circle::isColliding(const Vector2 &point1, const Vector2 &point2) const
    {
        double distanceSq = math_lib::dot2(point1 - point2, point1 - point2);
        if (distanceSq <= 1e-8)
            return false;

        double r = math_lib::dot2(point1 - point2, point1 - center_) / distanceSq;
        r = std::max(0.0, std::min(r, 1.0));

        Vector2 closestPointToCenter = r * (point2 - point1) + point1;
        double distToCenter = math_lib::dot2(closestPointToCenter - center_, closestPointToCenter - center_);

        return (distToCenter <= radius_);
    }

    void Line::print()
    {
        std::cout << "\n--------LINE OBSTACLE CONFIG--------\n";
        std::cout << "ID   : [" << id() << "]\n";
        std::cout << "Type : [" << to_string(type()) << "]\n";
        std::cout << "Start: [" << point1_.transpose() << "]\n";
        std::cout << "End  : [" << point2_.transpose() << "]\n";
        std::cout << "------------------------------------\n";
    }

    void Line::print() const
    {
        std::cout << "\n--------LINE OBSTACLE CONFIG--------\n";
        std::cout << "ID   : [" << id() << "]\n";
        std::cout << "Type : [" << to_string(type()) << "]\n";
        std::cout << "Start: [" << point1_.transpose() << "]\n";
        std::cout << "End  : [" << point2_.transpose() << "]\n";
        std::cout << "------------------------------------\n";
    }

    bool Line::isColliding(const Vector2 &point3, const Vector2 &point4)
    {
        double D = ((point1_.x() - point2_.x()) * (point3.y() - point4.y())) - ((point1_.y() - point2_.y()) * (point3.x() - point4.x()));

        if (abs(D) < 1e-6)
        {
            if (onSegment(point1_, point3, point2_) || onSegment(point1_, point4, point2_))
                return true;

            if (onSegment(point3, point1_, point4) || onSegment(point3, point2_, point4))
                return true;

            return false;
        }

        double x1_x2 = point1_.x() - point2_.x();
        double y1_y2 = point1_.y() - point2_.y();
        double x3_x4 = point3.x() - point4.x();
        double y3_y4 = point3.y() - point4.y();
        double x1_x3 = point1_.x() - point3.x();
        double y1_y3 = point1_.y() - point3.y();

        double x1y2_y1x2 = point1_.x() * point2_.y() - point1_.y() * point2_.x();
        double x3y4_y3x4 = point3.x() * point4.y() - point3.y() * point4.x();

        world::Vector2 intPoint;
        intPoint.x() = (x1y2_y1x2 * x3_x4 - x1_x2 * x3y4_y3x4) / D;
        intPoint.y() = (x1y2_y1x2 * y3_y4 - y1_y2 * x3y4_y3x4) / D;

        double den = (x1_x2 * y3_y4 - y1_y2 * y3_y4);
        double t = (x1_x3 * y3_y4 - y1_y3 * x3_x4) / den;
        double u = (y1_y2 * x1_x3 - x1_x2 * y1_y3) / den;

        if (t <= 1.0 && t >= 0.0 && u <= 1.0 && u >= 0.0)
            return true;

        return false;
    }

    bool Line::isColliding(const Vector2 &point3, const Vector2 &point4) const
    {
        double D = ((point1_.x() - point2_.x()) * (point3.y() - point4.y())) - ((point1_.y() - point2_.y()) * (point3.x() - point4.x()));

        if (abs(D) < 1e-6)
        {
            if (onSegment(point1_, point3, point2_) || onSegment(point1_, point4, point2_))
                return true;

            if (onSegment(point3, point1_, point4) || onSegment(point3, point2_, point4))
                return true;

            return false;
        }

        double x1_x2 = point1_.x() - point2_.x();
        double y1_y2 = point1_.y() - point2_.y();
        double x3_x4 = point3.x() - point4.x();
        double y3_y4 = point3.y() - point4.y();
        double x1_x3 = point1_.x() - point3.x();
        double y1_y3 = point1_.y() - point3.y();

        double x1y2_y1x2 = point1_.x() * point2_.y() - point1_.y() * point2_.x();
        double x3y4_y3x4 = point3.x() * point4.y() - point3.y() * point4.x();

        world::Vector2 intPoint;
        intPoint.x() = (x1y2_y1x2 * x3_x4 - x1_x2 * x3y4_y3x4) / D;
        intPoint.y() = (x1y2_y1x2 * y3_y4 - y1_y2 * x3y4_y3x4) / D;

        double den = (x1_x2 * y3_y4 - y1_y2 * y3_y4);
        double t = (x1_x3 * y3_y4 - y1_y3 * x3_x4) / den;
        double u = (y1_y2 * x1_x3 - x1_x2 * y1_y3) / den;

        if (t <= 1.0 && t >= 0.0 && u <= 1.0 && u >= 0.0)
            return true;

        return false;
    }

    bool onSegment(const Vector2 &p, const Vector2 &q, const Vector2 &r)
    {
        // Source: https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/

        double cross = math_lib::cross2(q - p, r - p);
        double dot = math_lib::dot2(q - p, r - p);

        if (cross == 0.0 && dot > 0.0)
            return true;

        return false;
    }

    Orientation orientation(const Vector2 &p, const Vector2 &q, const Vector2 &r)
    {
        // Source: https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/

        int val = (q.y() - p.y()) * (r.x() - q.x()) -
                  (q.x() - p.x()) * (r.y() - q.y());

        if (val < 0.0)
            return Orientation::COUNTERCLOCKWISE;
        if (val == 0.0)
            return Orientation::COLINEAR;
        return Orientation::CLOCKWISE;
    }
}
