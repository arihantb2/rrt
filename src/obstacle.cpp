#include <obstacle.h>

namespace world
{
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

    bool Line::isColliding(const Vector2 &point1, const Vector2 &point2)
    {
        // Source: https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/

        ORIENTATION o1 = orientation(point1, point1_, point2);
        ORIENTATION o2 = orientation(point1, point1_, point2_);
        ORIENTATION o3 = orientation(point2, point2_, point1);
        ORIENTATION o4 = orientation(point2, point2_, point1_);

        // General case
        if (o1 != o2 && o3 != o4)
            return true;

        // Special Cases
        // point1, point2 and point1_ are colinear and point1_ lies on segment point1-point2
        if (o1 == ORIENTATION::COLINEAR && onSegment(point1, point1_, point2))
            return true;

        // point1, point2 and point2_ are colinear and point2_ lies on segment point1-point2
        if (o2 == ORIENTATION::COLINEAR && onSegment(point1, point2_, point2))
            return true;

        // point1_, point2_ and point1 are colinear and point1 lies on segment p2q2
        if (o3 == ORIENTATION::COLINEAR && onSegment(point1_, point1, point2_))
            return true;

        // point1_, point2_ and point2 are colinear and point2 lies on segment p2q2
        if (o4 == ORIENTATION::COLINEAR && onSegment(point1_, point2, point2_))
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

    ORIENTATION orientation(const Vector2 &p, const Vector2 &q, const Vector2 &r)
    {
        // Source: https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/

        int val = (q.y() - p.y()) * (r.x() - q.x()) -
                  (q.x() - p.x()) * (r.y() - q.y());

        if (val < 0.0)
            return ORIENTATION::COUNTERCLOCKWISE;
        if (val == 0.0)
            return ORIENTATION::COLINEAR;
        return ORIENTATION::CLOCKWISE;
    }
}
