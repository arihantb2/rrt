#include <obstacle.h>

namespace world
{
    bool Circle::isColliding(const Vector2 &point1, const Vector2 &point2)
    {
        // Source: https://mathworld.wolfram.com/Circle-LineIntersection.html#:~:text=In%20geometry%2C%20a%20line%20meeting,429).

        double distance = math_lib::euclideandist2(point1, point2);
        double abscross = math_lib::cross2(point1, point2);
        double discriminant = std::pow(radius_, 2) * std::pow(distance, 2) - std::pow(abscross, 2);

        return (discriminant > 0);
    }

    bool Line::isColliding(const Vector2 &point1, const Vector2 &point2)
    {
        // Source: https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/

        Line::ORIENTATION o1 = __orientation(point1, point1_, point2);
        Line::ORIENTATION o2 = __orientation(point1, point1_, point2_);
        Line::ORIENTATION o3 = __orientation(point2, point2_, point1);
        Line::ORIENTATION o4 = __orientation(point2, point2_, point1_);

        // General case
        if (o1 != o2 && o3 != o4)
            return true;

        // Special Cases
        // point1, point2 and point1_ are colinear and point1_ lies on segment point1-point2
        if (o1 == Line::ORIENTATION::COLINEAR && __onSegment(point1, point1_, point2))
            return true;

        // point1, point2 and point2_ are colinear and point2_ lies on segment point1-point2
        if (o2 == Line::ORIENTATION::COLINEAR && __onSegment(point1, point2_, point2))
            return true;

        // point1_, point2_ and point1 are colinear and point1 lies on segment p2q2
        if (o3 == Line::ORIENTATION::COLINEAR && __onSegment(point1_, point1, point2_))
            return true;

        // point1_, point2_ and point2 are colinear and point2 lies on segment p2q2
        if (o4 == Line::ORIENTATION::COLINEAR && __onSegment(point1_, point2, point2_))
            return true;

        return false; // Doesn't fall in any of the above cases
    }

    bool Line::__onSegment(const Vector2 &p, const Vector2 &q, const Vector2 &r)
    {
        // Source: https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/

        if (q.x() <= std::max(p.x(), r.x()) &&
            q.x() >= std::min(p.x(), r.x()) &&
            q.y() <= std::max(p.y(), r.y()) &&
            q.y() >= std::min(p.y(), r.y()))
            return true;

        return false;
    }

    Line::ORIENTATION Line::__orientation(const Vector2 &p, const Vector2 &q, const Vector2 &r)
    {
        // Source: https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/

        int val = (q.y() - p.y()) * (r.x() - q.x()) -
                  (q.x() - p.x()) * (r.y() - q.y());

        if (val < 0.0)
            return Line::ORIENTATION::COUNTERCLOCKWISE;
        if (val == 0.0)
            return Line::ORIENTATION::COLINEAR;
        return Line::ORIENTATION::CLOCKWISE;
    }
}
