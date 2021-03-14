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
        double D = ((point1().x() - point2_.x()) * (line.point1().y() - line.point2().y())) - ((point1().y() - point2_.y()) * (line.point1().x() - line.point2().x()));

        if (abs(D) < 1e-6)
        {
            if (this->onLine(line.point1()) || this->onLine(line.point2()))
                return true;

            if (line.onLine(point1()) || line.onLine(point2()))
                return true;

            return false;
        }

        double x1_x2 = point1().x() - point2_.x();
        double y1_y2 = point1().y() - point2_.y();
        double x3_x4 = line.point1().x() - line.point2().x();
        double y3_y4 = line.point1().y() - line.point2().y();
        double x1_x3 = point1().x() - line.point1().x();
        double y1_y3 = point1().y() - line.point1().y();

        double x1y2_y1x2 = point1().x() * point2_.y() - point1().y() * point2_.x();
        double x3y4_y3x4 = line.point1().x() * line.point2().y() - line.point1().y() * line.point2().x();

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

    bool LineObstacle::collisionCheck(const Line2 &line) const
    {
        double D = ((point1().x() - point2_.x()) * (line.point1().y() - line.point2().y())) - ((point1().y() - point2_.y()) * (line.point1().x() - line.point2().x()));

        if (abs(D) < 1e-6)
        {
            if (this->onLine(line.point1()) || this->onLine(line.point2()))
                return true;

            if (line.onLine(point1()) || line.onLine(point2()))
                return true;

            return false;
        }

        double x1_x2 = point1().x() - point2_.x();
        double y1_y2 = point1().y() - point2_.y();
        double x3_x4 = line.point1().x() - line.point2().x();
        double y3_y4 = line.point1().y() - line.point2().y();
        double x1_x3 = point1().x() - line.point1().x();
        double y1_y3 = point1().y() - line.point1().y();

        double x1y2_y1x2 = point1().x() * point2_.y() - point1().y() * point2_.x();
        double x3y4_y3x4 = line.point1().x() * line.point2().y() - line.point1().y() * line.point2().x();

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
}
