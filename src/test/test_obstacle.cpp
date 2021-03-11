#include <obstacle.h>

int main(int argc, char const *argv[])
{
    world::Line line(world::Vector2(-50.0, 0.0), world::Vector2(50.0, 0.0));

    world::Vector2 point1(0.0, 0.0);
    world::Vector2 point2(10.0, 10.0);
    world::Vector2 point3(10.0, -10.0);
    world::Vector2 point4(-10.0, 10.0);
    world::Vector2 point5(-10.0, -10.0);
    world::Vector2 point6(5.0, 5.0);
    world::Vector2 point7(5.0, -5.0);
    world::Vector2 point8(-5.0, 5.0);
    world::Vector2 point9(-5.0, -5.0);

    std::cout << std::boolalpha << (line.isColliding(point1, point2) == true) << "\n";
    std::cout << std::boolalpha << (line.isColliding(point1, point3) == true) << "\n";
    std::cout << std::boolalpha << (line.isColliding(point1, point4) == true) << "\n";
    std::cout << std::boolalpha << (line.isColliding(point1, point5) == true) << "\n";
    std::cout << std::boolalpha << (line.isColliding(point2, point3) == true) << "\n";
    std::cout << std::boolalpha << (line.isColliding(point4, point5) == true) << "\n";
    std::cout << std::boolalpha << (line.isColliding(point2, point5) == true) << "\n";
    std::cout << std::boolalpha << (line.isColliding(point3, point4) == true) << "\n";
    std::cout << std::boolalpha << (line.isColliding(point2, point4) == false) << "\n";
    std::cout << std::boolalpha << (line.isColliding(point3, point5) == false) << "\n";

    world::Circle circle(world::Vector2(0.0, 0.0), 5.0);

    std::cout << std::boolalpha << (circle.isColliding(point1, point2) == true) << "\n";
    std::cout << std::boolalpha << (circle.isColliding(point1, point3) == true) << "\n";
    std::cout << std::boolalpha << (circle.isColliding(point1, point4) == true) << "\n";
    std::cout << std::boolalpha << (circle.isColliding(point1, point5) == true) << "\n";
    std::cout << std::boolalpha << (circle.isColliding(point2, point3) == false) << "\n";
    std::cout << std::boolalpha << (circle.isColliding(point4, point5) == false) << "\n";
    std::cout << std::boolalpha << (circle.isColliding(point2, point5) == true) << "\n";
    std::cout << std::boolalpha << (circle.isColliding(point3, point4) == true) << "\n";
    std::cout << std::boolalpha << (circle.isColliding(point2, point4) == false) << "\n";
    std::cout << std::boolalpha << (circle.isColliding(point3, point5) == false) << "\n";

    return 0;
}
