#include <obstacle.h>

namespace world
{
    unsigned int Obstacle::idCounter_ = 0;

    void Circle::print()
    {
        std::cout << "--------CIRCLE OBSTACLE CONFIG--------\n";
        std::cout << "ID    : [" << id() << "]\n";
        std::cout << "Type  : [" << to_string(type()) << "]\n";
        std::cout << "Center: [" << center_.transpose() << "]\n";
        std::cout << "Radius: [" << radius_ << "]\n";
    }

    void Circle::print() const
    {
        std::cout << "--------CIRCLE OBSTACLE CONFIG--------\n";
        std::cout << "ID    : [" << id() << "]\n";
        std::cout << "Type  : [" << to_string(type()) << "]\n";
        std::cout << "Center: [" << center_.transpose() << "]\n";
        std::cout << "Radius: [" << radius_ << "]\n";
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
        std::cout << "--------LINE OBSTACLE CONFIG--------\n";
        std::cout << "ID   : [" << id() << "]\n";
        std::cout << "Type : [" << to_string(type()) << "]\n";
        std::cout << "Start: [" << point1_.transpose() << "]\n";
        std::cout << "End  : [" << point2_.transpose() << "]\n";
    }

    void Line::print() const
    {
        std::cout << "--------LINE OBSTACLE CONFIG--------\n";
        std::cout << "ID   : [" << id() << "]\n";
        std::cout << "Type : [" << to_string(type()) << "]\n";
        std::cout << "Start: [" << point1_.transpose() << "]\n";
        std::cout << "End  : [" << point2_.transpose() << "]\n";
    }

    bool Line::isColliding(const Vector2 &point1, const Vector2 &point2)
    {
        world::Vector2 intPoint;

        double deltax = (point2.x() - point1.x());
        double deltay = (point2.y() - point1.y());

        double m1, c1;
        bool yline = false;
        bool xline = false;
        if (deltax == 0.0)
            xline = true;
        else if (deltay = 0.0)
        {
            yline = true;
            c1 = point1.y();
        }
        else
        {
            m1 = deltay / deltax;
            c1 = point1.y() - m1 * point1.x();
        }

        double deltax_ = (point2_.x() - point1_.x());
        double deltay_ = (point2_.y() - point1_.y());

        double m2, c2;
        bool yline_ = false;
        bool xline_ = false;
        if (deltax_ == 0.0)
            xline_ = true;
        else if (deltay_ == 0.0)
        {
            yline_ = true;
            c2 = point1_.y();
        }
        else
        {
            m2 = deltay_ / deltax_;
            c2 = point1_.y() - m2 * point1_.x();
        }

        if (xline)
        {
            intPoint.x() = point1.x();

            if (xline_)
            {
                if (point1.x() != point1_.x())
                    return false;

                if (onSegment(point1_, point1, point2_) || onSegment(point1_, point2, point2_))
                    return true;

                if (onSegment(point1, point1_, point2) || onSegment(point1, point2_, point2))
                    return true;
            }

            if (yline_)
                intPoint.y() = point1_.y();

            else
                intPoint.y() = m2 * point1_.x() + c2;

            bool o1 = onSegment(point1, intPoint, point2);
            bool o2 = onSegment(point1_, intPoint, point2_);

            if (o1 && o2)
                return true;

            return false;
        }

        else if (yline)
        {
            intPoint.y() = point1.y();

            if (yline_)
            {
                if (point1.y() != point1_.y())
                    return false;

                if (onSegment(point1_, point1, point2_) || onSegment(point1_, point2, point2_))
                    return true;

                if (onSegment(point1, point1_, point2) || onSegment(point1, point2_, point2))
                    return true;
            }

            if (xline_)
                intPoint.x() = point1_.x();

            else
                intPoint.x() = (point1_.y() - c2) / m2;

            bool o1 = onSegment(point1, intPoint, point2);
            bool o2 = onSegment(point1_, intPoint, point2_);

            if (o1 && o2)
                return true;

            return false;
        }
    }

    bool Line::isColliding(const Vector2 &point1, const Vector2 &point2) const
    {
        // Source: https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/

        Orientation o1 = orientation(point1, point1_, point2);
        Orientation o2 = orientation(point1, point1_, point2_);
        Orientation o3 = orientation(point2, point2_, point1);
        Orientation o4 = orientation(point2, point2_, point1_);

        // General case
        if (o1 != o2 && o3 != o4)
            return true;

        // Special Cases
        // point1, point2 and point1_ are colinear and point1_ lies on segment point1-point2
        if (o1 == Orientation::COLINEAR && onSegment(point1, point1_, point2))
            return true;

        // point1, point2 and point2_ are colinear and point2_ lies on segment point1-point2
        if (o2 == Orientation::COLINEAR && onSegment(point1, point2_, point2))
            return true;

        // point1_, point2_ and point1 are colinear and point1 lies on segment p2q2
        if (o3 == Orientation::COLINEAR && onSegment(point1_, point1, point2_))
            return true;

        // point1_, point2_ and point2 are colinear and point2 lies on segment p2q2
        if (o4 == Orientation::COLINEAR && onSegment(point1_, point2, point2_))
            return true;

        return false; // Doesn't fall in any of the above cases
    }

    bool onSegment(const Vector2 &p, const Vector2 &q, const Vector2 &r)
    {
        // Source: https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/

        if (q.x() <= std::max(p.x(), r.x()) &&
            q.x() >= std::min(p.x(), r.x()) &&
            q.y() <= std::max(p.y(), r.y()) &&
            q.y() >= std::min(p.y(), r.y()))
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
