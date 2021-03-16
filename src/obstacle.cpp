#include <obstacle.h>

namespace world
{
    unsigned int Obstacle::idCounter_ = 0;

    bool CircleObstacle::collisionCheck(const Line2 &line)
    {
        double distanceSq = math_lib::dot2(line.point1() - line.point2(), line.point1() - line.point2());
        if (distanceSq <= 1e-8)
            return false;

        double r = math_lib::dot2(line.point1() - line.point2(), line.point1() - center()) / distanceSq;
        r = std::max(0.0, std::min(r, 1.0));

        Vector2 closestPointToCenter = r * (line.point2() - line.point1()) + line.point1();
        double distToCenter = math_lib::dot2(closestPointToCenter - center(), closestPointToCenter - center());

        return (distToCenter <= radius());
    }

    bool CircleObstacle::collisionCheck(const Line2 &line) const
    {
        double distanceSq = math_lib::dot2(line.point1() - line.point2(), line.point1() - line.point2());
        if (distanceSq <= 1e-8)
            return false;

        double r = math_lib::dot2(line.point1() - line.point2(), line.point1() - center()) / distanceSq;
        r = std::max(0.0, std::min(r, 1.0));

        Vector2 closestPointToCenter = r * (line.point2() - line.point1()) + line.point1();
        double distToCenter = math_lib::dot2(closestPointToCenter - center(), closestPointToCenter - center());

        return (distToCenter <= radius());
    }

    bool LineObstacle::collisionCheck(const Line2 &line)
    {
        // Source: https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection

        double den = math_lib::cross2(point1() - point2(), line.point1() - line.point2());
        if (abs(den) < 1e-6)
        {
            if (this->onLine(line.point1()) || this->onLine(line.point2()))
                return true;

            if (line.onLine(point1()) || line.onLine(point2()))
                return true;

            return false;
        }

        double t = math_lib::cross2(point1() - line.point1(), line.point1() - line.point2()) / den;
        double u = math_lib::cross2(point2() - point1(), point1() - line.point1()) / den;

        if (t <= 1.0 && t >= 0.0 && u <= 1.0 && u >= 0.0)
            return true;

        return false;
    }

    bool LineObstacle::collisionCheck(const Line2 &line) const
    {
        // Source: https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection

        double den = math_lib::cross2(point1() - point2(), line.point1() - line.point2());
        if (abs(den) < 1e-6)
        {
            if (this->onLine(line.point1()) || this->onLine(line.point2()))
                return true;

            if (line.onLine(point1()) || line.onLine(point2()))
                return true;

            return false;
        }

        double t = math_lib::cross2(point1() - line.point1(), line.point1() - line.point2()) / den;
        double u = math_lib::cross2(point2() - point1(), point1() - line.point1()) / den;

        if (t <= 1.0 && t >= 0.0 && u <= 1.0 && u >= 0.0)
            return true;

        return false;
    }
}
